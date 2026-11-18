// Stride.BepuPhysics 形状的物理层，内核是 Jolt：静态体、动态体、角色（CharacterComponent 的行为），
// 32 个碰撞层加碰撞矩阵，射线和球体扫掠，固定步长加插值。
#include <Core/Core.h>
#include <Core/Debug.h>
#include <Core/HandleTable.h>
#include <Runtime/Transforms.h>
#include <StrideNative/Physics.h>
#include <StrideNative/Plugin.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>
#include <Jolt/Physics/Character/Character.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include <Jolt/Physics/Collision/Shape/EmptyShape.h>
#include <Jolt/Physics/Collision/Shape/TriangleShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Constraints/DistanceConstraint.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/GearConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <vector>

SN_PLUGIN(physics)

namespace
{
	using namespace JPH;

	constexpr uint32_t cLayerCount = 32;
	/// 对象层 = 碰撞层（0..31）加"会动"的标志位：宽相按会不会动分两层，静态之间不检测
	constexpr ObjectLayer cMovingBit = 32;

	ObjectLayer ToObjectLayer(uint32_t inLayer, bool inMoving)
	{
		return ObjectLayer(inLayer | (inMoving ? cMovingBit : 0));
	}

	uint32_t CollisionLayerOf(ObjectLayer inLayer)
	{
		return uint32_t(inLayer) & (cLayerCount - 1);
	}

	bool IsMoving(ObjectLayer inLayer)
	{
		return (inLayer & cMovingBit) != 0;
	}

	class BroadPhaseLayers final : public BroadPhaseLayerInterface
	{
	public:
		uint GetNumBroadPhaseLayers() const override { return 2; }
		BroadPhaseLayer GetBroadPhaseLayer(ObjectLayer inLayer) const override { return BroadPhaseLayer(IsMoving(inLayer) ? 1 : 0); }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
		const char *GetBroadPhaseLayerName(BroadPhaseLayer inLayer) const override { return inLayer.GetValue() == 0 ? "Static" : "Moving"; }
#endif
	};

	class ObjectVsBroadPhaseFilter final : public ObjectVsBroadPhaseLayerFilter
	{
	public:
		bool ShouldCollide(ObjectLayer inLayer, BroadPhaseLayer inBroadPhaseLayer) const override
		{
			return IsMoving(inLayer) || inBroadPhaseLayer.GetValue() == 1;
		}
	};

	/// Stride 的 CollisionMatrix：两个层能不能碰撞；两个都不会动的物体不检测
	class CollisionMatrixFilter final : public ObjectLayerPairFilter
	{
	public:
		uint32_t mMasks[cLayerCount];

		bool ShouldCollide(ObjectLayer inA, ObjectLayer inB) const override
		{
			if (!IsMoving(inA) && !IsMoving(inB))
				return false;
			return (mMasks[CollisionLayerOf(inA)] >> CollisionLayerOf(inB) & 1) != 0;
		}
	};

	/// 查询用：Stride 的 CollisionMask
	class MaskFilter final : public ObjectLayerFilter
	{
	public:
		explicit MaskFilter(uint32_t inMask) : mMask(inMask) { }
		bool ShouldCollide(ObjectLayer inLayer) const override { return (mMask >> CollisionLayerOf(inLayer) & 1) != 0; }

	private:
		uint32_t mMask;
	};

	struct CharacterRecord
	{
		Ref<Character> mCharacter;
		sn_character mSettings;
		float mInverseMass = 1.0f;
		float mTargetX = 0.0f;
		float mTargetZ = 0.0f;
		bool mJump = false;
	};

	struct BodyRecord
	{
		bool mRemoved = false;
		BodyID mID;
		uint32_t mMotion = SN_MOTION_STATIC;
		uint32_t mFlags = 0;
		int32_t mSlot = 0;					///< 绑定的变换槽位，0 表示没有
		int32_t mCharacter = -1;			///< mCharacters 里的下标
		RVec3 mPreviousPosition, mPosition;	///< 插值用：上一步和这一步的位姿
		Quat mPreviousRotation = Quat::sIdentity(), mRotation = Quat::sIdentity();
	};

	/**
	 * 接触事件：Jolt 在步进的工作线程上回调，按子形状对记下接触，一对物体的第一个子形状对出现时报开始、最后一个消失时报结束。
	 * 和 Bepu 一样，物体睡着时接触还在：Jolt 在物体睡下时也回调 OnContactRemoved，这时忽略，醒来后重新加入的同一对不再计数。
	 * 只看打开了接触事件的物体；回调里只读 mWanted（步进期间主线程不改它）。
	 */
	class ContactCollector final : public ContactListener
	{
	public:
		/// 按 Jolt 的 BodyID 下标找我们的物体编号（OnContactRemoved 只给 BodyID）
		std::vector<int32_t>			mIndexOfBodyID;
		/// 每个物体（按编号）报不报接触事件
		std::vector<uint8_t>			mWanted;
		std::vector<sn_contact_event>	mEvents;
		const PhysicsSystem *			mSystem = nullptr;

		void OnContactAdded(const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, ContactSettings &) override
		{
			Update(int32_t(inBody1.GetUserData()), int32_t(inBody2.GetUserData()), SubShapes(inManifold.mSubShapeID1, inManifold.mSubShapeID2), true);
		}

		void OnContactRemoved(const SubShapeIDPair &inPair) override
		{
			// 移除过的物体（编号已经清掉）不再报告
			int32_t a = IndexOf(inPair.GetBody1ID()), b = IndexOf(inPair.GetBody2ID());
			if (a < 0 || b < 0)
				return;
			// 睡下引起的移除：接触还算在（同 Bepu）。静态物体从来不"醒着"，只看会动的那个
			if (Sleeping(inPair.GetBody1ID()) || Sleeping(inPair.GetBody2ID()))
				return;
			Update(a, b, SubShapes(inPair.GetSubShapeID1(), inPair.GetSubShapeID2()), false);
		}

		/// 物体被移除：和它有关的接触直接忘掉（不报结束，Stride 移除时也不发）
		void Forget(int32_t inBody)
		{
			std::lock_guard lock(mMutex);
			for (auto it = mPairs.begin(); it != mPairs.end();)
				it = (int32_t(it->first >> 32) == inBody || int32_t(it->first & 0xffffffffu) == inBody) ? mPairs.erase(it) : std::next(it);
		}

	private:
		std::mutex mMutex;
		/// 一对物体（报告的一方在高 32 位）→ 正在接触的子形状对
		std::unordered_map<uint64_t, std::unordered_set<uint64_t>> mPairs;

		bool Sleeping(BodyID inID) const
		{
			const BodyInterface &bodies = mSystem->GetBodyInterfaceNoLock();
			return bodies.GetMotionType(inID) != EMotionType::Static && !bodies.IsActive(inID);
		}

		static uint64_t SubShapes(const SubShapeID &inA, const SubShapeID &inB)
		{
			return uint64_t(inA.GetValue()) << 32 | inB.GetValue();
		}

		int32_t IndexOf(BodyID inID) const
		{
			uint32_t index = inID.GetIndex();
			return index < mIndexOfBodyID.size() ? mIndexOfBodyID[index] : -1;
		}

		bool Wanted(int32_t inBody) const
		{
			return inBody >= 0 && size_t(inBody) < mWanted.size() && mWanted[inBody] != 0;
		}

		void Update(int32_t inA, int32_t inB, uint64_t inSubShapes, bool inTouching)
		{
			if (!Wanted(inA) && !Wanted(inB))
				return;
			// 报告的一方在前：打开事件的那个，都打开时编号小的；子形状对跟着换
			if (!Wanted(inA) || (Wanted(inB) && inB < inA))
			{
				std::swap(inA, inB);
				inSubShapes = inSubShapes << 32 | inSubShapes >> 32;
			}
			uint64_t key = uint64_t(uint32_t(inA)) << 32 | uint32_t(inB);
			std::lock_guard lock(mMutex);
			std::unordered_set<uint64_t> &touching = mPairs[key];
			bool before = !touching.empty();
			if (inTouching)
				touching.insert(inSubShapes);
			else
				touching.erase(inSubShapes);
			if (!before && !touching.empty())
				mEvents.push_back({ SN_CONTACT_STARTED, inA, inB });
			else if (before && touching.empty())
				mEvents.push_back({ SN_CONTACT_STOPPED, inA, inB });
			if (touching.empty())
				mPairs.erase(key);
		}
	};

	/// 封闭网格按密度 1 的体积积分，见 IntegrateMesh
	struct MeshVolume
	{
		float	mVolume = 0.0f;
		Vec3	mFirst = Vec3::sZero();
		float	mSecond[6] = {};
	};

	struct World
	{
		BroadPhaseLayers				mBroadPhaseLayers;
		ObjectVsBroadPhaseFilter		mObjectVsBroadPhase;
		CollisionMatrixFilter			mObjectPairs;
		PhysicsSystem					mSystem;
		sn_physics_settings				mSettings;
		std::vector<BodyRecord>			mBodies;			///< 按编号（加入顺序）
		std::vector<CharacterRecord>	mCharacters;
		std::vector<RefConst<Shape>>	mMeshes;			///< 网格和凸包，句柄是下标加 1
		std::vector<MeshVolume>			mMeshVolumes;		///< 和 mMeshes 对应；凸包的是空的（Jolt 自己算质量）
		int32_t							mTransforms = 0;	///< 绑定用的变换存储句柄，0 表示没有
		std::vector<float>				mPositions;			///< 每个物体的 xyz；只分配一次，调用方可以一直拿着视图
		ContactCollector				mContacts;
		/// 约束，句柄是下标加 1；移除后留空
		struct ConstraintRecord
		{
			Ref<Constraint>	mConstraint;
			int32_t			mBodyA = -1, mBodyB = -1;	///< 单体约束的 mBodyB 是 -1
			/// 单体角马达的世界角速度：Jolt 的目标角速度在物体的约束空间里，每步按物体当前的朝向换算
			bool			mWorldAngularVelocity = false;
			Vec3			mTarget = Vec3::sZero();
		};
		/// 移除过的物体的编号，加物体时先复用
		std::vector<int32_t>			mFreeBodies;
		std::vector<ConstraintRecord>	mConstraints;
	};

	World *GetWorld(int32_t inHandle)
	{
		return StrideNative::Lookup<World>(inHandle, "physics world");
	}

	/// 世界绑定的变换存储；存储先被销毁时记下错误、返回 nullptr。
	StrideNative::TransformStore *BoundStore(const World &inWorld)
	{
		return StrideNative::LookupTransformStore(inWorld.mTransforms, "transform store (destroyed before the physics world?)");
	}

	bool Moves(uint32_t inMotion)
	{
		return inMotion == SN_MOTION_DYNAMIC || inMotion == SN_MOTION_CHARACTER;
	}

	/// 位姿由仿真推进、要写回变换的物体（运动学物体按速度走，也算）
	bool Integrates(uint32_t inMotion)
	{
		return inMotion != SN_MOTION_STATIC;
	}

	size_t Capacity(const World &inWorld)
	{
		return inWorld.mPositions.size() / 3;
	}

	void WritePositions(World &ioWorld, size_t inFirst)
	{
		const BodyInterface &bodies = ioWorld.mSystem.GetBodyInterfaceNoLock();
		for (size_t i = inFirst; i < ioWorld.mBodies.size(); ++i)
		{
			if (ioWorld.mBodies[i].mRemoved)
				continue;
			RVec3 position = bodies.GetPosition(ioWorld.mBodies[i].mID);
			float *out = &ioWorld.mPositions[i * 3];
			out[0] = float(position.GetX());
			out[1] = float(position.GetY());
			out[2] = float(position.GetZ());
		}
	}

	/// 会动的物体记下这一步的位姿，写回绑定的槽位（插值的物体由 sn_physics_interpolate 再写一次）
	void RecordPoses(World &ioWorld, StrideNative::TransformStore *ioStore)
	{
		const BodyInterface &bodies = ioWorld.mSystem.GetBodyInterfaceNoLock();
		for (BodyRecord &record : ioWorld.mBodies)
		{
			if (!Integrates(record.mMotion) || record.mRemoved)
				continue;
			record.mPreviousPosition = record.mPosition;
			record.mPreviousRotation = record.mRotation;
			bodies.GetPositionAndRotation(record.mID, record.mPosition, record.mRotation);
			if (ioStore != nullptr && record.mSlot != 0)
				ioStore->SetWorldPose(record.mSlot, Vec3(record.mPosition), record.mRotation);
		}
	}

	/// sn_physics_settings 的默认值（Stride 的 BepuSimulation）
	sn_physics_settings DefaultSettings()
	{
		sn_physics_settings settings {};
		settings.gravity[1] = -9.8f;
	settings.linear_damping = 0.05f;
	settings.angular_damping = 0.05f;
		for (uint32_t &mask : settings.collision_masks)
			mask = 0xffffffffu;
		return settings;
	}

	Vec3 Load3(const float *inValues)
	{
		return Vec3(inValues[0], inValues[1], inValues[2]);
	}

	Quat LoadQuat(const float *inValues)
	{
		Quat q(inValues[0], inValues[1], inValues[2], inValues[3]);
		return q.IsNormalized() ? q : q.Normalized();
	}

	/// 封闭网格按密度 1 的体积积分（Bepu 的 ComputeClosedInertia 同样的做法）：体积、一阶矩 ∫r dV、二阶矩 ∫r rᵀ dV
	/// （xx、yy、zz、xy、xz、yz）。动态的网格刚体用它算质心和转动惯量
	MeshVolume IntegrateMesh(const VertexList &inVertices, const IndexedTriangleList &inTriangles)
	{
		MeshVolume m;
		for (const IndexedTriangle &t : inTriangles)
		{
			// 原点和三角形组成的四面体
			Vec3 a(inVertices[t.mIdx[0]]), b(inVertices[t.mIdx[1]]), c(inVertices[t.mIdx[2]]);
			float v = a.Dot(b.Cross(c)) / 6.0f;
			Vec3 s = a + b + c;
			m.mVolume += v;
			m.mFirst += v * s / 4.0f;
			auto second = [&](int i, int j) { return v / 20.0f * (a[i] * a[j] + b[i] * b[j] + c[i] * c[j] + s[i] * s[j]); };
			m.mSecond[0] += second(0, 0);
			m.mSecond[1] += second(1, 1);
			m.mSecond[2] += second(2, 2);
			m.mSecond[3] += second(0, 1);
			m.mSecond[4] += second(0, 2);
			m.mSecond[5] += second(1, 2);
		}
		// 三角形朝里时体积是负的：整体取反
		if (m.mVolume < 0.0f)
		{
			m.mVolume = -m.mVolume;
			m.mFirst = -m.mFirst;
			for (float &value : m.mSecond)
				value = -value;
		}
		return m;
	}

	/// 动态的网格刚体的质量属性：网格按 inScale 缩放、总质量 inMass；质心写进 outCenterOfMass。网格不封闭（体积接近 0）时
	/// 按包围盒的实心盒子算
	MassProperties MeshMassProperties(const MeshVolume &inVolume, const Shape &inMesh, Vec3Arg inScale, float inMass, Vec3 &outCenterOfMass)
	{
		MassProperties properties;
		float scale_volume = abs(inScale.GetX() * inScale.GetY() * inScale.GetZ());
		float volume = inVolume.mVolume * scale_volume;
		if (volume < 1.0e-6f)
		{
			AABox bounds = inMesh.GetLocalBounds().Scaled(inScale);
			outCenterOfMass = bounds.GetCenter();
			properties.SetMassAndInertiaOfSolidBox(Vec3::sMax(bounds.GetSize(), Vec3::sReplicate(1.0e-3f)), inMass);
			return properties;
		}
		// 缩放 S 之后：一阶矩 det(S)·S·f，二阶矩 det(S)·S·C·S；再按总质量换算密度
		float density = inMass / volume;
		Vec3 first = inVolume.mFirst * inScale * scale_volume;
		outCenterOfMass = first / volume;
		const float *c = inVolume.mSecond;
		float sx = inScale.GetX(), sy = inScale.GetY(), sz = inScale.GetZ();
		float xx = c[0] * sx * sx, yy = c[1] * sy * sy, zz = c[2] * sz * sz;
		float xy = c[3] * sx * sy, xz = c[4] * sx * sz, yz = c[5] * sy * sz;
		// 关于原点的惯量 tr(C)·I - C，乘密度，再移到质心（平行轴定理）
		Mat44 inertia = Mat44::sZero();
		float k = density * scale_volume;
		inertia.SetColumn4(0, Vec4(k * (yy + zz), -k * xy, -k * xz, 0));
		inertia.SetColumn4(1, Vec4(-k * xy, k * (xx + zz), -k * yz, 0));
		inertia.SetColumn4(2, Vec4(-k * xz, -k * yz, k * (xx + yy), 0));
		inertia.SetColumn4(3, Vec4(0, 0, 0, 1));
		Vec3 d = outCenterOfMass;
		Mat44 shift = Mat44::sScale(inMass * d.LengthSq()) - Mat44::sOuterProduct(inMass * d, d);
		properties.mMass = inMass;
		properties.mInertia = inertia - shift;
		properties.mInertia.SetColumn4(3, Vec4(0, 0, 0, 1));
		return properties;
	}

	/// 盒子和圆柱的倒角半径不能超过最短的半边长
	float ConvexRadius(float inSmallestHalfExtent)
	{
		return std::min(cDefaultConvexRadius, inSmallestHalfExtent * 0.5f);
	}

	/// 一个形状，不含本地位姿；网格按 inMeshScale 缩放
	RefConst<Shape> MakeShape(const World &inWorld, const sn_shape &inShape, Vec3Arg inMeshScale, std::string &outError)
	{
		const float *s = inShape.size;
		switch (inShape.type)
		{
		case SN_SHAPE_BOX:
			if (!(s[0] > 0.0f && s[1] > 0.0f && s[2] > 0.0f))
				break;
			return new BoxShape(Vec3(s[0], s[1], s[2]) * 0.5f, ConvexRadius(0.5f * std::min({ s[0], s[1], s[2] })));
		case SN_SHAPE_SPHERE:
			if (!(s[0] > 0.0f))
				break;
			return new SphereShape(s[0]);
		case SN_SHAPE_CAPSULE:
			if (!(s[0] > 0.0f && s[1] >= 0.0f))
				break;
			return new CapsuleShape(0.5f * s[1], s[0]);
		case SN_SHAPE_CYLINDER:
			if (!(s[0] > 0.0f && s[1] > 0.0f))
				break;
			return new CylinderShape(0.5f * s[1], s[0], ConvexRadius(std::min(s[0], 0.5f * s[1])));
		case SN_SHAPE_EMPTY:
			return new EmptyShape();
		case SN_SHAPE_MESH:
		case SN_SHAPE_CONVEX_HULL:
		{
			if (inShape.mesh <= 0 || size_t(inShape.mesh) > inWorld.mMeshes.size())
			{
				outError = "mesh or hull handle " + std::to_string(inShape.mesh) + " does not exist";
				return nullptr;
			}
			const RefConst<Shape> &mesh = inWorld.mMeshes[inShape.mesh - 1];
			return inMeshScale.IsClose(Vec3::sOne()) ? mesh : RefConst<Shape>(new ScaledShape(mesh, inMeshScale));
		}
		default:
			outError = "unknown shape type " + std::to_string(inShape.type);
			return nullptr;
		}
		outError = "shape type " + std::to_string(inShape.type) + " has invalid size";
		return nullptr;
	}

	/// 物体的形状：一个形状且没有本地位姿时就是它本身，否则是复合形状（Stride 的 CompoundCollider）
	RefConst<Shape> MakeBodyShape(const World &inWorld, const sn_shape *inShapes, uint32_t inCount, Vec3Arg inMeshScale, std::string &outError)
	{
		if (inCount == 0)
		{
			outError = "a body needs at least one shape";
			return nullptr;
		}
		auto local = [](const sn_shape &inShape) { return std::pair(Load3(inShape.position), LoadQuat(inShape.rotation)); };
		if (inCount == 1)
		{
			RefConst<Shape> shape = MakeShape(inWorld, inShapes[0], inMeshScale, outError);
			if (shape == nullptr)
				return nullptr;
			auto [position, rotation] = local(inShapes[0]);
			if (position.IsNearZero() && rotation.IsClose(Quat::sIdentity()))
				return shape;
			return new RotatedTranslatedShape(position, rotation, shape);
		}
		StaticCompoundShapeSettings compound;
		for (uint32_t i = 0; i < inCount; ++i)
		{
			if (inShapes[i].type == SN_SHAPE_EMPTY)
			{
				outError = "an empty shape cannot be part of a compound";
				return nullptr;
			}
			RefConst<Shape> shape = MakeShape(inWorld, inShapes[i], inMeshScale, outError);
			if (shape == nullptr)
				return nullptr;
			auto [position, rotation] = local(inShapes[i]);
			compound.AddShape(position, rotation, shape);
		}
		ShapeSettings::ShapeResult result = compound.Create();
		if (result.HasError())
		{
			outError = std::string("compound shape: ") + result.GetError().c_str();
			return nullptr;
		}
		return result.Get();
	}

	/// 角色的一步（Stride 的 CharacterComponent.SimulationUpdate 和 Bepu 的 CharacterControllers）：
	/// 站稳时起跳，或者用有上限的力追目标水平速度、把角色压在地面上；悬空时按比例给一点空中操控。
	/// 目标速度在角色的基里，角色不转，所以就是世界的 X、Z。
	void UpdateCharacter(World &ioWorld, CharacterRecord &ioRecord, float inDeltaTime)
	{
		Character &character = *ioRecord.mCharacter;
		const sn_character &settings = ioRecord.mSettings;
		Vec3 velocity = character.GetLinearVelocity(false);
		bool supported = character.GetGroundState() == CharacterBase::EGroundState::OnGround;
		Vec3 target(ioRecord.mTargetX, 0.0f, ioRecord.mTargetZ);
		if (supported)
		{
			Vec3 ground = character.GetGroundVelocity();
			float relative_up = velocity.GetY() - ground.GetY();
			if (ioRecord.mJump)
			{
				// 跳跃按"相对支撑物的竖直速度补到 JumpVelocity"算，不是固定的冲量
				velocity += Vec3(0.0f, std::max(0.0f, settings.jump_velocity - relative_up), 0.0f);
			}
			else
			{
				Vec3 relative = velocity - ground;
				Vec3 change = target - Vec3(relative.GetX(), 0.0f, relative.GetZ());
				float limit = settings.maximum_horizontal_force * ioRecord.mInverseMass * inDeltaTime;
				float length = change.Length();
				if (length > limit)
					change *= limit / length;
				velocity += change;
				// 竖直方向的约束只往下拉：离开地面的速度用最大竖直力消掉
				if (relative_up > 0.0f)
					velocity -= Vec3(0.0f, std::min(relative_up, settings.maximum_vertical_force * ioRecord.mInverseMass * inDeltaTime), 0.0f);
			}
		}
		else if (target.LengthSq() > 0.0f)
		{
			// 只往移动方向加速、不减速，同 Stride。和 Stride 不同的是方向先归一化：Stride 拿没归一化的目标速度做点积，
			// 单位对不上，离地以后几乎没有空中操控；速度上限按目标速度的大小，摇杆推一半，空中也只到一半
			float target_speed = target.Length();
			Vec3 direction = target / target_speed;
			float current = velocity.Dot(direction);
			float acceleration = ioRecord.mInverseMass * settings.maximum_horizontal_force * settings.air_control_force_scale * inDeltaTime;
			float wanted = std::min(current + acceleration, target_speed * settings.air_control_scale);
			velocity += direction * std::max(0.0f, wanted - current);
		}
		// TryJump 不论成没成都在这一步用掉
		ioRecord.mJump = false;
		character.SetLinearVelocity(velocity, false);
	}

	/// 网格对网格的碰撞（Jolt 没有，Bepu 有：动态的网格刚体撞静态网格）：网格 1 落在网格 2 包围盒里的三角形，逐个当成
	/// 凸的三角形形状和网格 2 碰。三角形在世界空间里，变换是单位的
	void CollideMeshVsMesh(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1,
		Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2,
		const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter)
	{
		AABox bounds = inShape2->GetWorldSpaceBounds(inCenterOfMassTransform2, inScale2);
		bounds.ExpandBy(Vec3::sReplicate(inCollideShapeSettings.mMaxSeparationDistance));
		Shape::GetTrianglesContext context;
		inShape1->GetTrianglesStart(context, bounds, inCenterOfMassTransform1.GetTranslation(), inCenterOfMassTransform1.GetQuaternion(), inScale1);
		Float3 vertices[Shape::cGetTrianglesMinTrianglesRequested * 3];
		for (int count; (count = inShape1->GetTrianglesNext(context, Shape::cGetTrianglesMinTrianglesRequested, vertices)) > 0; )
			for (int t = 0; t < count; ++t)
			{
				Vec3 v1(vertices[t * 3]), v2(vertices[t * 3 + 1]), v3(vertices[t * 3 + 2]);
				TriangleShape triangle(v1, v2, v3);
				triangle.SetEmbedded();
				CollisionDispatch::sCollideShapeVsShape(&triangle, inShape2, Vec3::sOne(), inScale2, Mat44::sIdentity(), inCenterOfMassTransform2,
					inSubShapeIDCreator1, inSubShapeIDCreator2, inCollideShapeSettings, ioCollector, inShapeFilter);
				if (ioCollector.ShouldEarlyOut())
					return;
			}
	}

	/// 第一次创建物理世界时注册 Jolt 的类型，sn_shutdown 时注销。放在物理这边而不是 sn_init 里：物理在 wasm 里是
	/// 单独的侧模块，核心模块不链接 Jolt 的物理部分。JS 只在主线程上调用，不需要加锁。
	void EnsureTypesRegistered()
	{
		// 关闭钩子每次 sn_shutdown 都会执行（同 HandleTable），只注册一次，没注册过类型时什么也不做
		static const bool sHooked = (StrideNative::AddShutdownHook([]
		{
			if (Factory::sInstance == nullptr)
				return;
			UnregisterTypes();
			delete Factory::sInstance;
			Factory::sInstance = nullptr;
		}), true);
		(void)sHooked;
		if (Factory::sInstance != nullptr)
			return;
		Factory::sInstance = new Factory();
		RegisterTypes();
		// Jolt 不支持网格对网格（Bepu 支持）
		CollisionDispatch::sRegisterCollideShape(EShapeSubType::Mesh, EShapeSubType::Mesh, CollideMeshVsMesh);
	}

	/// 物体编号对应的记录；编号不对时记下错误
	/// 按编号找没被移除的物体；没有时记下错误、返回 nullptr。
	BodyRecord *GetBody(World &ioWorld, int32_t inBody)
	{
		if (inBody < 0 || size_t(inBody) >= ioWorld.mBodies.size() || ioWorld.mBodies[inBody].mRemoved)
		{
			StrideNative::SetLastError(SN_ERROR_INVALID_ARGUMENT, "body " + std::to_string(inBody) + " does not exist");
			return nullptr;
		}
		return &ioWorld.mBodies[inBody];
	}

	CharacterRecord *GetCharacter(World &ioWorld, int32_t inBody)
	{
		BodyRecord *body = GetBody(ioWorld, inBody);
		if (body == nullptr)
			return nullptr;
		if (body->mCharacter < 0)
		{
			StrideNative::SetLastError(SN_ERROR_INVALID_ARGUMENT, "body " + std::to_string(inBody) + " is not a character");
			return nullptr;
		}
		return &ioWorld.mCharacters[body->mCharacter];
	}

	int32_t BodyIndexOf(const World &inWorld, BodyID inID)
	{
		return int32_t(inWorld.mSystem.GetBodyInterfaceNoLock().GetUserData(inID));
	}

	/// 扫掠的最近命中；可以跳过起点就重叠的物体（它们的命中位置是 0）
	class ClosestCastHit final : public CastShapeCollector
	{
	public:
		explicit ClosestCastHit(bool inIgnoreInitialOverlap) : mIgnoreInitialOverlap(inIgnoreInitialOverlap) { }

		void AddHit(const ShapeCastResult &inResult) override
		{
			if (mIgnoreInitialOverlap && inResult.mFraction <= 0.0f)
				return;
			if (inResult.mFraction >= GetEarlyOutFraction())
				return;
			mHit = inResult;
			mHadHit = true;
			UpdateEarlyOutFraction(inResult.mFraction);
		}

		ShapeCastResult mHit;
		bool mHadHit = false;

	private:
		bool mIgnoreInitialOverlap;
	};

	/// 查询的方向：归一化，长度为 0 时报错
	bool QueryDirection(const float *inDirection, float inMaxDistance, Vec3 &outDirection)
	{
		Vec3 direction = Load3(inDirection);
		if (direction.IsNearZero() || !(inMaxDistance > 0.0f))
		{
			StrideNative::SetLastError(SN_ERROR_INVALID_ARGUMENT, "query direction and max_distance must be non-zero");
			return false;
		}
		outDirection = direction.Normalized();
		return true;
	}
}

using namespace StrideNative;

SN_API int32_t sn_physics_create(uint32_t max_bodies, int32_t transforms, const sn_physics_settings *settings)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	EnsureTypesRegistered();
	if (max_bodies == 0)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "max_bodies must be positive");
	if (transforms != 0 && LookupTransformStore(transforms) == nullptr)
		return SN_ERROR_INVALID_HANDLE;

	auto world = std::make_unique<World>();
	world->mSettings = settings != nullptr ? *settings : DefaultSettings();
	for (uint32_t a = 0; a < cLayerCount; ++a)
		for (uint32_t b = 0; b < cLayerCount; ++b)
			if ((world->mSettings.collision_masks[a] >> b & 1) != (world->mSettings.collision_masks[b] >> a & 1))
				return SetLastError(SN_ERROR_INVALID_ARGUMENT, "collision_masks must be symmetric (layer " + std::to_string(a) + " and layer " + std::to_string(b) + ")");
	std::copy(std::begin(world->mSettings.collision_masks), std::end(world->mSettings.collision_masks), world->mObjectPairs.mMasks);

	// Resting bodies touch several others (a box in a stack has up to four neighbours and the
	// ground), so pairs and contacts need headroom over the body count.
	constexpr uint32_t cContactsPerBody = 8;
	world->mSystem.Init(max_bodies, 0, max_bodies * cContactsPerBody, max_bodies * cContactsPerBody,
		world->mBroadPhaseLayers, world->mObjectVsBroadPhase, world->mObjectPairs);
	world->mSystem.SetGravity(Load3(world->mSettings.gravity));
	world->mBodies.reserve(max_bodies);
	world->mTransforms = transforms;
	world->mPositions.resize(size_t(max_bodies) * 3);
	world->mContacts.mIndexOfBodyID.assign(max_bodies, -1);
	world->mContacts.mWanted.assign(max_bodies, 0);
	world->mContacts.mSystem = &world->mSystem;
	world->mSystem.SetContactListener(&world->mContacts);
	return GetTable<World>().Add(std::move(world));
}

SN_API void sn_physics_destroy(int32_t world)
{
	std::unique_ptr<World> removed = GetTable<World>().Remove(world);
	if (removed == nullptr)
		return;
	for (CharacterRecord &record : removed->mCharacters)
		if (record.mCharacter != nullptr)
			record.mCharacter->RemoveFromPhysicsSystem(false);
	removed->mCharacters.clear();	// 角色自己销毁它的物体
	std::vector<BodyID> ids;
	for (const BodyRecord &record : removed->mBodies)
		if (record.mCharacter < 0 && !record.mRemoved)
			ids.push_back(record.mID);
	if (ids.empty())
		return;
	BodyInterface &bodies = removed->mSystem.GetBodyInterface();
	bodies.RemoveBodies(ids.data(), int(ids.size()));
	bodies.DestroyBodies(ids.data(), int(ids.size()));
}

SN_API int32_t sn_physics_set_gravity(int32_t world, const float *gravity)
{
	World *w = GetWorld(world);
	if (w == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (gravity == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "gravity is required");
	w->mSystem.SetGravity(Load3(gravity));
	std::copy(gravity, gravity + 3, w->mSettings.gravity);
	return SN_OK;
}

SN_API int32_t sn_physics_create_mesh(int32_t world, const float *positions, uint32_t vertex_count, const uint32_t *indices, uint32_t index_count)
{
	World *w = GetWorld(world);
	if (w == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (positions == nullptr || indices == nullptr || vertex_count == 0 || index_count == 0 || index_count % 3 != 0)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "a mesh needs positions and a multiple of 3 indices");
	VertexList vertices(vertex_count);
	for (uint32_t i = 0; i < vertex_count; ++i)
		vertices[i] = Float3(positions[i * 3], positions[i * 3 + 1], positions[i * 3 + 2]);
	IndexedTriangleList triangles(index_count / 3);
	for (uint32_t i = 0; i < index_count; i += 3)
	{
		if (indices[i] >= vertex_count || indices[i + 1] >= vertex_count || indices[i + 2] >= vertex_count)
			return SetLastError(SN_ERROR_INVALID_ARGUMENT, "mesh index " + std::to_string(i) + " is out of range");
		triangles[i / 3] = IndexedTriangle(indices[i], indices[i + 1], indices[i + 2]);
	}
	MeshVolume volume = IntegrateMesh(vertices, triangles);
	MeshShapeSettings settings(std::move(vertices), std::move(triangles));
	ShapeSettings::ShapeResult result = settings.Create();
	if (result.HasError())
		return SetLastError(SN_ERROR_FAILED, std::string("mesh shape: ") + result.GetError().c_str());
	w->mMeshes.push_back(result.Get());
	w->mMeshVolumes.push_back(volume);
	return int32_t(w->mMeshes.size());
}

SN_API int32_t sn_physics_create_hull(int32_t world, const float *positions, uint32_t point_count)
{
	World *w = GetWorld(world);
	if (w == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (positions == nullptr || point_count < 4)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "a convex hull needs at least 4 points");
	Array<Vec3> points(point_count);
	for (uint32_t i = 0; i < point_count; ++i)
		points[i] = Load3(positions + i * 3);
	ConvexHullShapeSettings settings(points);
	ShapeSettings::ShapeResult result = settings.Create();
	if (result.HasError())
		return SetLastError(SN_ERROR_FAILED, std::string("convex hull shape: ") + result.GetError().c_str());
	w->mMeshes.push_back(result.Get());
	w->mMeshVolumes.emplace_back();
	return int32_t(w->mMeshes.size());
}

SN_API int32_t sn_physics_add_bodies(int32_t world, const sn_body *bodies, uint32_t count, const sn_shape *shapes, uint32_t shape_count,
	const sn_character *characters, uint32_t character_count, int32_t *out_bodies)
{
	World *w = GetWorld(world);
	if (w == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (count == 0)
		return int32_t(w->mBodies.size());
	if (bodies == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "bodies are required");
	if (w->mBodies.size() + count > Capacity(*w) + w->mFreeBodies.size())
		return SetLastError(SN_ERROR_FAILED, "body limit reached");

	// 先全部检查、建好形状，出错时什么都不加
	TransformStore *store = nullptr;
	std::vector<RefConst<Shape>> body_shapes(count);
	std::vector<RMat44> poses(count);
	std::vector<Vec3> mesh_scales(count, Vec3::sOne());
	for (uint32_t i = 0; i < count; ++i)
	{
		const sn_body &body = bodies[i];
		std::string where = "body " + std::to_string(i) + ": ";
		if (body.motion > SN_MOTION_CHARACTER)
			return SetLastError(SN_ERROR_INVALID_ARGUMENT, where + "unknown motion " + std::to_string(body.motion));
		if (body.layer >= cLayerCount)
			return SetLastError(SN_ERROR_INVALID_ARGUMENT, where + "layer must be 0..31");
		if (body.shape_count == 0 || shapes == nullptr || body.first_shape + uint64_t(body.shape_count) > shape_count)
			return SetLastError(SN_ERROR_INVALID_ARGUMENT, where + "shapes are out of range");
		if (body.motion == SN_MOTION_CHARACTER && (characters == nullptr || body.character >= character_count))
			return SetLastError(SN_ERROR_INVALID_ARGUMENT, where + "character settings are out of range");
		Vec3 scale = Vec3::sOne();
		RMat44 pose = RMat44::sRotationTranslation(LoadQuat(body.rotation), RVec3(Load3(body.position)));
		if (body.transform != 0)
		{
			if (w->mTransforms == 0)
				return SetLastError(SN_ERROR_INVALID_ARGUMENT, where + "has a transform but the world was created without a transform store");
			if (store == nullptr && (store = BoundStore(*w)) == nullptr)
				return SN_ERROR_INVALID_HANDLE;
			if (!store->IsAlive(body.transform))
				return SetLastError(SN_ERROR_INVALID_ARGUMENT, where + "transform slot " + std::to_string(body.transform) + " is not allocated");
			// 槽位现在的世界位姿；缩放只作用到网格形状（同 Stride：复合碰撞体按绝对尺寸）
			Mat44 matrix = store->ComputeWorldMatrix(body.transform);
			pose = RMat44::sRotationTranslation(matrix.Decompose(scale).GetQuaternion().Normalized(), RVec3(matrix.GetTranslation()));
			// 世界矩阵不是旋转（比如某一级的四元数接近 0，或是 (0, 0.5, 0.5, 0) 这种按公式得到奇异矩阵的）时分解出 NaN；
			// NaN 的物体会让 Jolt 的宽相卡死，这里直接报错
			if (!pose.GetQuaternion().IsNormalized() || matrix.GetTranslation().IsNaN())
				return SetLastError(SN_ERROR_INVALID_ARGUMENT, where + "transform slot " + std::to_string(body.transform) + " has a world matrix that is not a rotation (check the quaternions)");
		}
		const sn_shape *own = shapes + body.first_shape;
		for (uint32_t s = 0; s < body.shape_count; ++s)
			if (own[s].type == SN_SHAPE_MESH && Moves(body.motion) && (body.shape_count != 1 || body.motion == SN_MOTION_CHARACTER))
				return SetLastError(SN_ERROR_INVALID_ARGUMENT, where + "a moving body with a mesh shape must have only that shape (and cannot be a character)");
		std::string error;
		body_shapes[i] = MakeBodyShape(*w, own, body.shape_count, scale, error);
		mesh_scales[i] = scale;
		if (body_shapes[i] == nullptr)
			return SetLastError(SN_ERROR_INVALID_ARGUMENT, where + error);
		poses[i] = pose;
	}

	BodyInterface &body_interface = w->mSystem.GetBodyInterface();
	std::vector<BodyID> added;
	added.reserve(count);
	auto mass_of = [&](const sn_body &inBody)
	{
		float mass = 0.0f;
		for (uint32_t s = 0; s < inBody.shape_count; ++s)
			mass += shapes[inBody.first_shape + s].mass;
		return mass > 0.0f ? mass : 1.0f;
	};
	// 编号：先复用移除过的，再接在后面
	std::vector<int32_t> indices(count);
	for (uint32_t i = 0; i < count; ++i)
	{
		if (!w->mFreeBodies.empty())
		{
			indices[i] = w->mFreeBodies.back();
			w->mFreeBodies.pop_back();
		}
		else
		{
			// 先占位、标成已移除：后面建物体失败中途返回时，占位的记录不会被当成物体
			indices[i] = int32_t(w->mBodies.size());
			w->mBodies.emplace_back().mRemoved = true;
		}
	}
	for (uint32_t i = 0; i < count; ++i)
	{
		const sn_body &body = bodies[i];
		int32_t index = indices[i];
		BodyRecord record;
		record.mMotion = body.motion;
		record.mFlags = body.flags;
		record.mSlot = body.transform;
		record.mPosition = record.mPreviousPosition = poses[i].GetTranslation();
		record.mRotation = record.mPreviousRotation = poses[i].GetQuaternion();
		ObjectLayer layer = ToObjectLayer(body.layer, Moves(body.motion) || body.motion == SN_MOTION_KINEMATIC);
		if (body.motion == SN_MOTION_CHARACTER)
		{
			// 角色的物体由 JPH::Character 建：只能平移；摩擦为 0，水平速度完全由角色逻辑给（相当于 Bepu 的运动约束）
			CharacterSettings settings;
			settings.mShape = body_shapes[i];
			settings.mLayer = layer;
			settings.mMass = mass_of(body);
			settings.mFriction = 0.0f;
			settings.mGravityFactor = (body.flags & SN_BODY_NO_GRAVITY) != 0 ? 0.0f : 1.0f;
			settings.mMaxSlopeAngle = DegreesToRadians(characters[body.character].slope_angle);
			CharacterRecord character;
			character.mSettings = characters[body.character];
			character.mInverseMass = 1.0f / settings.mMass;
			character.mCharacter = new Character(&settings, record.mPosition, record.mRotation, uint64(index), &w->mSystem);
			record.mID = character.mCharacter->GetBodyID();
			{
				BodyLockWrite lock(w->mSystem.GetBodyLockInterface(), record.mID);
				lock.GetBody().GetMotionProperties()->SetLinearDamping(w->mSettings.linear_damping);
				lock.GetBody().GetMotionProperties()->SetAngularDamping(w->mSettings.angular_damping);
			}
			record.mCharacter = int32_t(w->mCharacters.size());
			w->mCharacters.push_back(std::move(character));
		}
		else
		{
			EMotionType motion = body.motion == SN_MOTION_DYNAMIC ? EMotionType::Dynamic
				: body.motion == SN_MOTION_KINEMATIC ? EMotionType::Kinematic : EMotionType::Static;
			BodyCreationSettings settings(body_shapes[i], record.mPosition, record.mRotation, motion, layer);
			settings.mUserData = uint64(index);
			settings.mFriction = body.friction;
			settings.mLinearDamping = w->mSettings.linear_damping;
			settings.mAngularDamping = w->mSettings.angular_damping;
			settings.mGravityFactor = (body.flags & SN_BODY_NO_GRAVITY) != 0 ? 0.0f : 1.0f;
			settings.mAllowSleeping = (body.flags & SN_BODY_NEVER_SLEEP) == 0;
			settings.mIsSensor = (body.flags & SN_BODY_SENSOR) != 0;
			// BodyComponent 能在动态和运动学之间切换；质量照动态的算，切过去时用得上。只有一个网格形状的按封闭网格算质量
			// （Bepu 的动态网格，Stride 的 MeshCollider），质心挪到网格的质心；复合形状里有网格的只能是静态
			const sn_shape &first_shape = shapes[body.first_shape];
			bool single_mesh = body.shape_count == 1 && first_shape.type == SN_SHAPE_MESH;
			bool has_mesh = false;
			for (uint32_t s = 0; s < body.shape_count; ++s)
				has_mesh |= shapes[body.first_shape + s].type == SN_SHAPE_MESH;
			settings.mAllowDynamicOrKinematic = motion != EMotionType::Static && (!has_mesh || single_mesh);
			if (settings.mAllowDynamicOrKinematic && single_mesh)
			{
				Vec3 center_of_mass;
				const RefConst<Shape> &mesh = w->mMeshes[first_shape.mesh - 1];
				settings.mOverrideMassProperties = EOverrideMassProperties::MassAndInertiaProvided;
				settings.mMassPropertiesOverride = MeshMassProperties(w->mMeshVolumes[first_shape.mesh - 1], *mesh, mesh_scales[i], mass_of(body), center_of_mass);
				settings.SetShape(new OffsetCenterOfMassShape(body_shapes[i], center_of_mass));
			}
			else if (settings.mAllowDynamicOrKinematic && body.shape_count == 1 && shapes[body.first_shape].type == SN_SHAPE_EMPTY)
			{
				// Bepu 的 EmptyCollider：转动惯量是质量为 m、半径 1 的实心球
				float mass = mass_of(body);
				settings.mOverrideMassProperties = EOverrideMassProperties::MassAndInertiaProvided;
				settings.mMassPropertiesOverride.mMass = mass;
				settings.mMassPropertiesOverride.mInertia = Mat44::sScale(0.4f * mass);
			}
			else if (settings.mAllowDynamicOrKinematic)
			{
				settings.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
				settings.mMassPropertiesOverride.mMass = mass_of(body);
			}
			Body *created = body_interface.CreateBody(settings);
			if (created == nullptr)
				return SetLastError(SN_ERROR_FAILED, "body limit reached");
			record.mID = created->GetID();
			added.push_back(record.mID);
		}
		w->mContacts.mIndexOfBodyID[record.mID.GetIndex()] = index;
		w->mContacts.mWanted[index] = (body.flags & SN_BODY_CONTACT_EVENTS) != 0;
		w->mBodies[index] = record;
	}

	// 普通物体成批插进宽相（AddBodiesPrepare 会重排传入的数组，所以用副本）；角色各自加入
	if (!added.empty())
	{
		BodyInterface::AddState state = body_interface.AddBodiesPrepare(added.data(), int(added.size()));
		body_interface.AddBodiesFinalize(added.data(), int(added.size()), state, EActivation::Activate);
	}
	for (int32_t index : indices)
		if (w->mBodies[index].mCharacter >= 0)
			w->mCharacters[w->mBodies[index].mCharacter].mCharacter->AddToPhysicsSystem(EActivation::Activate);
	WritePositions(*w, size_t(*std::min_element(indices.begin(), indices.end())));
	if (out_bodies != nullptr)
		std::copy(indices.begin(), indices.end(), out_bodies);
	return indices[0];
}

SN_API int32_t sn_physics_step(int32_t world, float delta_time, int32_t collision_steps)
{
	World *w = GetWorld(world);
	if (w == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	TransformStore *store = nullptr;
	if (w->mTransforms != 0 && (store = BoundStore(*w)) == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	SN_DEBUG_SCOPE("physics.step");
	for (CharacterRecord &character : w->mCharacters)
		if (character.mCharacter != nullptr)
			UpdateCharacter(*w, character, delta_time);
	w->mContacts.mEvents.clear();
	// 单体角马达：世界角速度换到物体当前的本地空间（Jolt 的转动马达在物体 2 的约束空间里）
	for (const World::ConstraintRecord &record : w->mConstraints)
		if (record.mWorldAngularVelocity && record.mConstraint != nullptr)
		{
			Quat rotation = w->mSystem.GetBodyInterface().GetRotation(w->mBodies[record.mBodyA].mID);
			static_cast<SixDOFConstraint &>(*record.mConstraint).SetTargetAngularVelocityCS(rotation.Conjugated() * record.mTarget);
		}
	EPhysicsUpdateError errors = w->mSystem.Update(delta_time, collision_steps > 0 ? collision_steps : 1, &GetTempAllocator(), &GetJobSystem());
	if (DebugEnabled())
	{
		sn_debug_set_counter("physics.bodies", w->mSystem.GetNumBodies());
		sn_debug_set_counter("physics.activeBodies", w->mSystem.GetNumActiveBodies(EBodyType::RigidBody));
		sn_debug_set_counter("physics.characters", double(w->mCharacters.size()));
	}
	// 地面状态按这一步结束时的接触算（Bepu 也是用这一步的接触判断支撑）
	constexpr float cMaxSeparationDistance = 0.05f;
	for (CharacterRecord &character : w->mCharacters)
		if (character.mCharacter != nullptr)
			character.mCharacter->PostSimulation(cMaxSeparationDistance, false);
	WritePositions(*w, 0);
	RecordPoses(*w, store);
	if (errors == EPhysicsUpdateError::None)
		return SN_OK;
	DebugLog(SN_LOG_ERROR, "physics step failed: error flags %u", uint32_t(errors));
	if ((errors & EPhysicsUpdateError::BodyPairCacheFull) != EPhysicsUpdateError::None)
		return SetLastError(SN_ERROR_FAILED, "physics step: body pair limit reached");
	if ((errors & EPhysicsUpdateError::ContactConstraintsFull) != EPhysicsUpdateError::None)
		return SetLastError(SN_ERROR_FAILED, "physics step: contact constraint limit reached");
	return SetLastError(SN_ERROR_FAILED, "physics step: contact cache full (temp allocator too small)");
}

SN_API int32_t sn_physics_interpolate(int32_t world, float alpha)
{
	World *w = GetWorld(world);
	if (w == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (w->mTransforms == 0)
		return SN_OK;
	TransformStore *store = BoundStore(*w);
	if (store == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	float t = std::clamp(alpha, 0.0f, 1.0f);
	for (const BodyRecord &record : w->mBodies)
	{
		if ((record.mFlags & SN_BODY_INTERPOLATED) == 0 || record.mSlot == 0 || !Integrates(record.mMotion) || record.mRemoved)
			continue;
		RVec3 position = record.mPreviousPosition + (record.mPosition - record.mPreviousPosition) * t;
		Quat rotation = record.mPreviousRotation.SLERP(record.mRotation, t);
		store->SetWorldPose(record.mSlot, Vec3(position), rotation);
	}
	return SN_OK;
}

SN_API const float *sn_physics_positions(int32_t world)
{
	World *w = GetWorld(world);
	return w != nullptr ? w->mPositions.data() : nullptr;
}

namespace
{
	/// 按编号找动态或运动学物体（BodyComponent）；不是时记下错误、返回 nullptr。
	/// 动态或运动学的物体；inAllowCharacter 时角色也行（角色的速度由它自己每步设，只开放读速度和瞬移）
	BodyRecord *GetMovableBody(World *inWorld, int32_t inBody, const char *inWhat, bool inAllowCharacter = false)
	{
		if (inWorld == nullptr)
			return nullptr;
		if (inBody < 0 || size_t(inBody) >= inWorld->mBodies.size())
		{
			SetLastError(SN_ERROR_INVALID_ARGUMENT, std::string(inWhat) + ": body " + std::to_string(inBody) + " does not exist");
			return nullptr;
		}
		BodyRecord &record = inWorld->mBodies[inBody];
		if (record.mRemoved)
		{
			SetLastError(SN_ERROR_INVALID_ARGUMENT, std::string(inWhat) + ": body " + std::to_string(inBody) + " was removed");
			return nullptr;
		}
		if (record.mMotion != SN_MOTION_DYNAMIC && record.mMotion != SN_MOTION_KINEMATIC && !(inAllowCharacter && record.mMotion == SN_MOTION_CHARACTER))
		{
			SetLastError(SN_ERROR_INVALID_ARGUMENT, std::string(inWhat) + ": body " + std::to_string(inBody) + (inAllowCharacter ? " is not a dynamic, kinematic or character body" : " is not a dynamic or kinematic body"));
			return nullptr;
		}
		return &record;
	}
}

SN_API int32_t sn_physics_remove_bodies(int32_t world, const int32_t *bodies, uint32_t count)
{
	World *w = GetWorld(world);
	if (w == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (bodies == nullptr && count > 0)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "bodies are required");
	BodyInterface &body_interface = w->mSystem.GetBodyInterface();
	std::vector<BodyID> ids;
	for (uint32_t i = 0; i < count; ++i)
	{
		int32_t index = bodies[i];
		if (index < 0 || size_t(index) >= w->mBodies.size() || w->mBodies[index].mRemoved)
			continue;
		// 先拿走连着它的约束（Jolt 要求约束比物体先移除）
		for (World::ConstraintRecord &constraint : w->mConstraints)
			if (constraint.mConstraint != nullptr && (constraint.mBodyA == index || constraint.mBodyB == index))
			{
				w->mSystem.RemoveConstraint(constraint.mConstraint);
				constraint.mConstraint = nullptr;
			}
		BodyRecord &record = w->mBodies[index];
		record.mRemoved = true;
		w->mContacts.mWanted[index] = 0;
		w->mContacts.mIndexOfBodyID[record.mID.GetIndex()] = -1;
		w->mContacts.Forget(index);
		if (record.mCharacter >= 0)
		{
			// 角色自己销毁它的物体
			CharacterRecord &character = w->mCharacters[record.mCharacter];
			character.mCharacter->RemoveFromPhysicsSystem(false);
			character.mCharacter = nullptr;
		}
		else
			ids.push_back(record.mID);
		w->mFreeBodies.push_back(index);
	}
	if (!ids.empty())
	{
		body_interface.RemoveBodies(ids.data(), int(ids.size()));
		body_interface.DestroyBodies(ids.data(), int(ids.size()));
	}
	return SN_OK;
}

namespace
{
	/// 和 inAxis 垂直的单位向量（铰链、SixDOF 的第二条轴，取哪一条不影响没有限制的那几个自由度）
	Vec3 Perpendicular(Vec3Arg inAxis)
	{
		return inAxis.GetNormalizedPerpendicular();
	}

	bool IsOneBody(uint32_t inType)
	{
		return inType == SN_CONSTRAINT_ONE_BODY_LINEAR_SERVO || inType == SN_CONSTRAINT_ONE_BODY_ANGULAR_SERVO || inType == SN_CONSTRAINT_ONE_BODY_ANGULAR_MOTOR;
	}

	/// 马达设置：频率 > 0 时位置马达带弹簧；最大力 <= 0 时不限
	MotorSettings MakeMotor(const sn_constraint &inConstraint)
	{
		float limit = inConstraint.max_force > 0.0f ? inConstraint.max_force : FLT_MAX;
		float frequency = inConstraint.spring[0] > 0.0f ? inConstraint.spring[0] : 0.0f;
		return MotorSettings(frequency, inConstraint.spring[1], limit, limit);
	}

	/// 约束的 Jolt 设置；a、b 是 Jolt 的两个物体（齿轮里和 Bepu 的 A、B 对调；单体约束里 a 是世界）
	Ref<TwoBodyConstraintSettings> MakeConstraintSettings(const sn_constraint &inConstraint, const Body &inA, const Body &inB, std::string &outError)
	{
		const sn_constraint &c = inConstraint;
		// A 的本地方向换到 B 的本地空间（建约束时两边的约束坐标系对齐）
		const Quat a_to_b = inB.GetRotation().Conjugated() * inA.GetRotation();
		switch (c.type)
		{
		case SN_CONSTRAINT_BALL_SOCKET:
		{
			Ref<PointConstraintSettings> settings = new PointConstraintSettings;
			settings->mSpace = EConstraintSpace::LocalToBodyCOM;
			settings->mPoint1 = RVec3(Load3(c.offset_a));
			settings->mPoint2 = RVec3(Load3(c.offset_b));
			return settings.GetPtr();
		}
		case SN_CONSTRAINT_HINGE:
		{
			Ref<HingeConstraintSettings> settings = new HingeConstraintSettings;
			settings->mSpace = EConstraintSpace::LocalToBodyCOM;
			settings->mPoint1 = RVec3(Load3(c.offset_a));
			settings->mPoint2 = RVec3(Load3(c.offset_b));
			settings->mHingeAxis1 = Load3(c.axis_a).Normalized();
			settings->mHingeAxis2 = Load3(c.axis_b).Normalized();
			settings->mNormalAxis1 = Perpendicular(settings->mHingeAxis1);
			settings->mNormalAxis2 = Perpendicular(settings->mHingeAxis2);
			return settings.GetPtr();
		}
		case SN_CONSTRAINT_ANGULAR_AXIS_GEAR_MOTOR:
		{
			// Jolt：Gear1Rotation = -ratio · Gear2Rotation。物体 1 是 Bepu 的 B（inA 这里已经是它），物体 2 是 Bepu 的 A，
			// 轴 2 取反，ratio 就是 VelocityScale
			Ref<GearConstraintSettings> settings = new GearConstraintSettings;
			settings->mSpace = EConstraintSpace::LocalToBodyCOM;
			Vec3 axis = Load3(c.axis_a).Normalized();
			settings->mHingeAxis1 = (inA.GetRotation().Conjugated() * inB.GetRotation()) * axis;
			settings->mHingeAxis2 = -axis;
			settings->mRatio = c.scale;
			return settings.GetPtr();
		}
		case SN_CONSTRAINT_POINT_ON_LINE:
		case SN_CONSTRAINT_LINEAR_AXIS_LIMIT:
		case SN_CONSTRAINT_SWING_LIMIT:
		case SN_CONSTRAINT_TWIST_LIMIT:
		{
			// 都用 SixDOF：只约束需要的自由度，其余放开（Bepu 的这几种约束彼此独立，可以叠加）
			Ref<SixDOFConstraintSettings> settings = new SixDOFConstraintSettings;
			settings->mSpace = EConstraintSpace::LocalToBodyCOM;
			settings->mPosition1 = RVec3(Load3(c.offset_a));
			settings->mPosition2 = RVec3(Load3(c.offset_b));
			for (int axis = 0; axis < SixDOFConstraintSettings::EAxis::Num; ++axis)
				settings->MakeFreeAxis(SixDOFConstraintSettings::EAxis(axis));
			using EAxis = SixDOFConstraintSettings::EAxis;
			if (c.type == SN_CONSTRAINT_TWIST_LIMIT)
			{
				// Bepu 的扭转轴是基的 Z、0° 是 X；Jolt 的扭转轴是约束坐标系的 X、从 Y 量起
				Quat basis_a = LoadQuat(c.basis_a), basis_b = LoadQuat(c.basis_b);
				settings->mAxisX1 = basis_a * Vec3::sAxisZ();
				settings->mAxisY1 = basis_a * Vec3::sAxisX();
				settings->mAxisX2 = basis_b * Vec3::sAxisZ();
				settings->mAxisY2 = basis_b * Vec3::sAxisX();
				settings->SetLimitedAxis(EAxis::RotationX, c.minimum, c.maximum);
				return settings.GetPtr();
			}
			Vec3 x1 = Load3(c.axis_a).Normalized();
			Vec3 x2 = c.type == SN_CONSTRAINT_SWING_LIMIT ? Load3(c.axis_b).Normalized() : a_to_b * x1;
			settings->mAxisX1 = x1;
			settings->mAxisY1 = Perpendicular(x1);
			settings->mAxisX2 = x2;
			settings->mAxisY2 = c.type == SN_CONSTRAINT_SWING_LIMIT ? Perpendicular(x2) : a_to_b * settings->mAxisY1;
			if (c.type == SN_CONSTRAINT_POINT_ON_LINE)
			{
				settings->MakeFixedAxis(EAxis::TranslationY);
				settings->MakeFixedAxis(EAxis::TranslationZ);
			}
			else if (c.type == SN_CONSTRAINT_LINEAR_AXIS_LIMIT)
				settings->SetLimitedAxis(EAxis::TranslationX, c.minimum, c.maximum);
			else
			{
				// 摆动：绕约束坐标系的 Y、Z 各不超过 maximum，圆锥形（Jolt 默认的 ESwingType::Cone）
				settings->SetLimitedAxis(EAxis::RotationY, -c.maximum, c.maximum);
				settings->SetLimitedAxis(EAxis::RotationZ, -c.maximum, c.maximum);
			}
			return settings.GetPtr();
		}
		case SN_CONSTRAINT_ONE_BODY_LINEAR_SERVO:
		case SN_CONSTRAINT_ONE_BODY_ANGULAR_SERVO:
		case SN_CONSTRAINT_ONE_BODY_ANGULAR_MOTOR:
		{
			// 世界（物体 1）和物体（物体 2）之间的 SixDOF，全部自由度放开，用马达驱动。约束坐标系：物体 1 是世界的轴，
			// 物体 2 是物体本地的轴，所以平移的目标就是世界坐标，朝向的目标就是世界朝向
			using EAxis = SixDOFConstraintSettings::EAxis;
			Ref<SixDOFConstraintSettings> settings = new SixDOFConstraintSettings;
			settings->mSpace = EConstraintSpace::LocalToBodyCOM;
			settings->mPosition1 = RVec3::sZero();
			settings->mPosition2 = RVec3(c.type == SN_CONSTRAINT_ONE_BODY_LINEAR_SERVO ? Load3(c.offset_a) : Vec3::sZero());
			settings->mAxisX1 = settings->mAxisX2 = Vec3::sAxisX();
			settings->mAxisY1 = settings->mAxisY2 = Vec3::sAxisY();
			for (int axis = 0; axis < EAxis::Num; ++axis)
			{
				settings->MakeFreeAxis(EAxis(axis));
				settings->mMotorSettings[axis] = MakeMotor(c);
			}
			return settings.GetPtr();
		}
		case SN_CONSTRAINT_DISTANCE_LIMIT:
		{
			Ref<DistanceConstraintSettings> settings = new DistanceConstraintSettings;
			settings->mSpace = EConstraintSpace::LocalToBodyCOM;
			settings->mPoint1 = RVec3(Load3(c.offset_a));
			settings->mPoint2 = RVec3(Load3(c.offset_b));
			settings->mMinDistance = c.minimum;
			settings->mMaxDistance = c.maximum;
			if (c.spring[0] > 0.0f)
				settings->mLimitsSpringSettings = SpringSettings(ESpringMode::FrequencyAndDamping, c.spring[0], c.spring[1]);
			return settings.GetPtr();
		}
		case SN_CONSTRAINT_BALL_SOCKET_MOTOR:
		{
			// A、B 之间的 SixDOF：平移的速度马达（物体 1 的约束空间就是 A 的本地轴），A 上的点取建约束时和 B 的点重合的位置
			using EAxis = SixDOFConstraintSettings::EAxis;
			Ref<SixDOFConstraintSettings> settings = new SixDOFConstraintSettings;
			settings->mSpace = EConstraintSpace::LocalToBodyCOM;
			RVec3 anchor = inB.GetCenterOfMassTransform() * Load3(c.offset_b);
			settings->mPosition1 = inA.GetInverseCenterOfMassTransform() * anchor;
			settings->mPosition2 = RVec3(Load3(c.offset_b));
			settings->mAxisX1 = Vec3::sAxisX();
			settings->mAxisY1 = Vec3::sAxisY();
			settings->mAxisX2 = a_to_b * Vec3::sAxisX();
			settings->mAxisY2 = a_to_b * Vec3::sAxisY();
			for (int axis = 0; axis < EAxis::Num; ++axis)
			{
				settings->MakeFreeAxis(EAxis(axis));
				settings->mMotorSettings[axis] = MakeMotor(c);
			}
			return settings.GetPtr();
		}
		case SN_CONSTRAINT_WELD:
		{
			Ref<FixedConstraintSettings> settings = new FixedConstraintSettings;
			settings->mSpace = EConstraintSpace::LocalToBodyCOM;
			Quat orientation = LoadQuat(c.basis_a);
			settings->mPoint1 = RVec3(Load3(c.offset_a));
			settings->mAxisX1 = orientation * Vec3::sAxisX();
			settings->mAxisY1 = orientation * Vec3::sAxisY();
			settings->mPoint2 = RVec3::sZero();
			settings->mAxisX2 = Vec3::sAxisX();
			settings->mAxisY2 = Vec3::sAxisY();
			return settings.GetPtr();
		}
		default:
			outError = "unknown constraint type " + std::to_string(c.type);
			return nullptr;
		}
	}
}

SN_API int32_t sn_physics_add_constraint(int32_t world, const sn_constraint *constraint)
{
	World *w = GetWorld(world);
	if (w == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (constraint == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "constraint is required");
	const sn_constraint &c = *constraint;
	bool one_body = IsOneBody(c.type);
	BodyRecord *a = GetBody(*w, c.body_a);
	BodyRecord *b = a == nullptr || one_body ? nullptr : GetBody(*w, c.body_b);
	if (a == nullptr || (!one_body && b == nullptr))
		return SN_ERROR_INVALID_ARGUMENT;
	if (!one_body && c.body_a == c.body_b)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "a constraint needs two different bodies");
	Ref<Constraint> created;
	std::string error;
	if (one_body)
	{
		// 物体 1 是世界
		BodyLockWrite lock(w->mSystem.GetBodyLockInterface(), a->mID);
		if (!lock.Succeeded())
			return SetLastError(SN_ERROR_INVALID_ARGUMENT, "constraint body is not in the physics system");
		Ref<TwoBodyConstraintSettings> settings = MakeConstraintSettings(c, Body::sFixedToWorld, lock.GetBody(), error);
		if (settings == nullptr)
			return SetLastError(SN_ERROR_INVALID_ARGUMENT, error);
		created = settings->Create(Body::sFixedToWorld, lock.GetBody());
	}
	else
	{
		// 齿轮马达里 Jolt 的物体 1 是 Bepu 的 B
		bool swapped = c.type == SN_CONSTRAINT_ANGULAR_AXIS_GEAR_MOTOR;
		BodyID ids[2] = { swapped ? b->mID : a->mID, swapped ? a->mID : b->mID };
		BodyLockMultiWrite lock(w->mSystem.GetBodyLockInterface(), ids, 2);
		Body *body1 = lock.GetBody(0), *body2 = lock.GetBody(1);
		if (body1 == nullptr || body2 == nullptr)
			return SetLastError(SN_ERROR_INVALID_ARGUMENT, "constraint bodies are not in the physics system");
		Ref<TwoBodyConstraintSettings> settings = MakeConstraintSettings(c, *body1, *body2, error);
		if (settings == nullptr)
			return SetLastError(SN_ERROR_INVALID_ARGUMENT, error);
		created = settings->Create(*body1, *body2);
	}
	World::ConstraintRecord record { created, c.body_a, one_body ? -1 : c.body_b };
	// 伺服、马达的目标
	if (c.type == SN_CONSTRAINT_ONE_BODY_LINEAR_SERVO || c.type == SN_CONSTRAINT_ONE_BODY_ANGULAR_SERVO || c.type == SN_CONSTRAINT_ONE_BODY_ANGULAR_MOTOR || c.type == SN_CONSTRAINT_BALL_SOCKET_MOTOR)
	{
		using EAxis = SixDOFConstraintSettings::EAxis;
		SixDOFConstraint &six = static_cast<SixDOFConstraint &>(*created);
		bool linear = c.type == SN_CONSTRAINT_ONE_BODY_LINEAR_SERVO || c.type == SN_CONSTRAINT_BALL_SOCKET_MOTOR;
		EMotorState state = c.type == SN_CONSTRAINT_ONE_BODY_ANGULAR_MOTOR || c.type == SN_CONSTRAINT_BALL_SOCKET_MOTOR ? EMotorState::Velocity : EMotorState::Position;
		for (int axis = 0; axis < 3; ++axis)
			six.SetMotorState(EAxis((linear ? EAxis::TranslationX : EAxis::RotationX) + axis), state);
		if (c.type == SN_CONSTRAINT_ONE_BODY_LINEAR_SERVO)
			six.SetTargetPositionCS(Load3(c.target));
		else if (c.type == SN_CONSTRAINT_ONE_BODY_ANGULAR_SERVO)
			six.SetTargetOrientationCS(LoadQuat(c.target));
		else if (c.type == SN_CONSTRAINT_BALL_SOCKET_MOTOR)
			six.SetTargetVelocityCS(Load3(c.target));
		else
		{
			record.mWorldAngularVelocity = true;
			record.mTarget = Load3(c.target);
		}
	}
	w->mSystem.AddConstraint(created);
	BodyInterface &bodies = w->mSystem.GetBodyInterface();
	bodies.ActivateBody(a->mID);
	if (b != nullptr)
		bodies.ActivateBody(b->mID);
	w->mConstraints.push_back(record);
	return int32_t(w->mConstraints.size());
}

SN_API int32_t sn_physics_remove_constraint(int32_t world, int32_t constraint)
{
	World *w = GetWorld(world);
	if (w == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (constraint <= 0 || size_t(constraint) > w->mConstraints.size())
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "constraint " + std::to_string(constraint) + " does not exist");
	World::ConstraintRecord &record = w->mConstraints[constraint - 1];
	if (record.mConstraint == nullptr)
		return SN_OK;
	w->mSystem.RemoveConstraint(record.mConstraint);
	record.mConstraint = nullptr;
	// 两边醒过来：之前被约束住的状态变了
	BodyInterface &bodies = w->mSystem.GetBodyInterface();
	for (int32_t body : { record.mBodyA, record.mBodyB })
		if (body >= 0 && !w->mBodies[body].mRemoved)
			bodies.ActivateBody(w->mBodies[body].mID);
	return SN_OK;
}

SN_API int32_t sn_physics_body_set_contact_events(int32_t world, int32_t body, int32_t enabled)
{
	World *w = GetWorld(world);
	if (w == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	BodyRecord *record = GetBody(*w, body);
	if (record == nullptr)
		return SN_ERROR_INVALID_ARGUMENT;
	w->mContacts.mWanted[body] = enabled != 0;
	return SN_OK;
}

SN_API int32_t sn_physics_body_set_sensor(int32_t world, int32_t body, int32_t sensor)
{
	World *w = GetWorld(world);
	if (w == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	BodyRecord *record = GetBody(*w, body);
	if (record == nullptr)
		return SN_ERROR_INVALID_ARGUMENT;
	w->mSystem.GetBodyInterface().SetIsSensor(record->mID, sensor != 0);
	return SN_OK;
}

SN_API const sn_contact_event *sn_physics_contact_events(int32_t world, uint32_t *count)
{
	World *w = GetWorld(world);
	if (w == nullptr || count == nullptr)
	{
		if (count != nullptr)
			*count = 0;
		return nullptr;
	}
	*count = uint32_t(w->mContacts.mEvents.size());
	return w->mContacts.mEvents.data();
}

SN_API int32_t sn_physics_body_set_kinematic(int32_t world, int32_t body, int32_t kinematic)
{
	World *w = GetWorld(world);
	BodyRecord *record = GetMovableBody(w, body, "sn_physics_body_set_kinematic");
	if (record == nullptr)
		return w == nullptr ? SN_ERROR_INVALID_HANDLE : SN_ERROR_INVALID_ARGUMENT;
	record->mMotion = kinematic != 0 ? SN_MOTION_KINEMATIC : SN_MOTION_DYNAMIC;
	w->mSystem.GetBodyInterface().SetMotionType(record->mID, kinematic != 0 ? EMotionType::Kinematic : EMotionType::Dynamic, EActivation::DontActivate);
	return SN_OK;
}

SN_API int32_t sn_physics_body_set_awake(int32_t world, int32_t body, int32_t awake)
{
	World *w = GetWorld(world);
	BodyRecord *record = GetMovableBody(w, body, "sn_physics_body_set_awake");
	if (record == nullptr)
		return w == nullptr ? SN_ERROR_INVALID_HANDLE : SN_ERROR_INVALID_ARGUMENT;
	BodyInterface &bodies = w->mSystem.GetBodyInterface();
	if (awake != 0)
		bodies.ActivateBody(record->mID);
	else
		bodies.DeactivateBody(record->mID);
	return SN_OK;
}

SN_API int32_t sn_physics_body_awake(int32_t world, int32_t body)
{
	World *w = GetWorld(world);
	BodyRecord *record = GetMovableBody(w, body, "sn_physics_body_awake");
	if (record == nullptr)
		return w == nullptr ? SN_ERROR_INVALID_HANDLE : SN_ERROR_INVALID_ARGUMENT;
	return w->mSystem.GetBodyInterface().IsActive(record->mID) ? 1 : 0;
}

SN_API int32_t sn_physics_body_teleport(int32_t world, int32_t body, const float *position, const float *rotation)
{
	World *w = GetWorld(world);
	BodyRecord *record = GetMovableBody(w, body, "sn_physics_body_teleport", true);
	if (record == nullptr)
		return w == nullptr ? SN_ERROR_INVALID_HANDLE : SN_ERROR_INVALID_ARGUMENT;
	if (position == nullptr || rotation == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "sn_physics_body_teleport: position and rotation are required");
	record->mPosition = record->mPreviousPosition = RVec3(Load3(position));
	record->mRotation = record->mPreviousRotation = LoadQuat(rotation);
	w->mSystem.GetBodyInterface().SetPositionAndRotation(record->mID, record->mPosition, record->mRotation, EActivation::Activate);
	float *out = &w->mPositions[size_t(body) * 3];
	out[0] = position[0];
	out[1] = position[1];
	out[2] = position[2];
	if (record->mSlot != 0)
	{
		TransformStore *store = BoundStore(*w);
		if (store == nullptr)
			return SN_ERROR_INVALID_HANDLE;
		store->SetWorldPose(record->mSlot, Vec3(record->mPosition), record->mRotation);
	}
	return SN_OK;
}

SN_API int32_t sn_physics_body_velocity(int32_t world, int32_t body, float *linear, float *angular)
{
	World *w = GetWorld(world);
	BodyRecord *record = GetMovableBody(w, body, "sn_physics_body_velocity", true);
	if (record == nullptr)
		return w == nullptr ? SN_ERROR_INVALID_HANDLE : SN_ERROR_INVALID_ARGUMENT;
	const BodyInterface &bodies = w->mSystem.GetBodyInterface();
	if (linear != nullptr)
		bodies.GetLinearVelocity(record->mID).StoreFloat3(reinterpret_cast<Float3 *>(linear));
	if (angular != nullptr)
		bodies.GetAngularVelocity(record->mID).StoreFloat3(reinterpret_cast<Float3 *>(angular));
	return SN_OK;
}

SN_API int32_t sn_physics_body_set_velocity(int32_t world, int32_t body, const float *linear, const float *angular)
{
	World *w = GetWorld(world);
	BodyRecord *record = GetMovableBody(w, body, "sn_physics_body_set_velocity");
	if (record == nullptr)
		return w == nullptr ? SN_ERROR_INVALID_HANDLE : SN_ERROR_INVALID_ARGUMENT;
	BodyInterface &bodies = w->mSystem.GetBodyInterface();
	if (linear != nullptr)
		bodies.SetLinearVelocity(record->mID, Load3(linear));
	if (angular != nullptr)
		bodies.SetAngularVelocity(record->mID, Load3(angular));
	return SN_OK;
}

SN_API int32_t sn_physics_body_apply_impulse(int32_t world, int32_t body, const float *linear, const float *angular)
{
	World *w = GetWorld(world);
	BodyRecord *record = GetMovableBody(w, body, "sn_physics_body_apply_impulse");
	if (record == nullptr)
		return w == nullptr ? SN_ERROR_INVALID_HANDLE : SN_ERROR_INVALID_ARGUMENT;
	if (record->mMotion != SN_MOTION_DYNAMIC)
		return SN_OK;
	BodyInterface &bodies = w->mSystem.GetBodyInterface();
	if (linear != nullptr)
		bodies.AddImpulse(record->mID, Load3(linear));
	if (angular != nullptr)
		bodies.AddAngularImpulse(record->mID, Load3(angular));
	return SN_OK;
}

SN_API int32_t sn_physics_body_set_gravity(int32_t world, int32_t body, int32_t enabled)
{
	World *w = GetWorld(world);
	BodyRecord *record = GetMovableBody(w, body, "sn_physics_body_set_gravity");
	if (record == nullptr)
		return w == nullptr ? SN_ERROR_INVALID_HANDLE : SN_ERROR_INVALID_ARGUMENT;
	w->mSystem.GetBodyInterface().SetGravityFactor(record->mID, enabled != 0 ? 1.0f : 0.0f);
	if (enabled != 0)
		record->mFlags &= ~uint32_t(SN_BODY_NO_GRAVITY);
	else
		record->mFlags |= SN_BODY_NO_GRAVITY;
	return SN_OK;
}

SN_API int32_t sn_physics_body_set_never_sleep(int32_t world, int32_t body, int32_t never_sleep)
{
	World *w = GetWorld(world);
	BodyRecord *record = GetMovableBody(w, body, "sn_physics_body_set_never_sleep");
	if (record == nullptr)
		return w == nullptr ? SN_ERROR_INVALID_HANDLE : SN_ERROR_INVALID_ARGUMENT;
	{
		BodyLockWrite lock(w->mSystem.GetBodyLockInterface(), record->mID);
		if (lock.Succeeded())
			lock.GetBody().SetAllowSleeping(never_sleep == 0);
	}
	if (never_sleep != 0)
		w->mSystem.GetBodyInterface().ActivateBody(record->mID);
	return SN_OK;
}

SN_API int32_t sn_physics_character_move(int32_t world, int32_t body, float target_x, float target_z, int32_t jump)
{
	World *w = GetWorld(world);
	if (w == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	CharacterRecord *character = GetCharacter(*w, body);
	if (character == nullptr)
		return SN_ERROR_INVALID_ARGUMENT;
	character->mTargetX = target_x;
	character->mTargetZ = target_z;
	character->mJump = character->mJump || jump != 0;
	return SN_OK;
}

SN_API int32_t sn_physics_character_grounded(int32_t world, int32_t body)
{
	World *w = GetWorld(world);
	if (w == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	CharacterRecord *character = GetCharacter(*w, body);
	if (character == nullptr)
		return SN_ERROR_INVALID_ARGUMENT;
	return character->mCharacter->GetGroundState() == CharacterBase::EGroundState::OnGround ? 1 : 0;
}

SN_API int32_t sn_physics_ray_cast(int32_t world, const float *origin, const float *direction, float max_distance, uint32_t mask, sn_hit *hit)
{
	World *w = GetWorld(world);
	if (w == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (origin == nullptr || direction == nullptr || hit == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "origin, direction and hit are required");
	Vec3 unit;
	if (!QueryDirection(direction, max_distance, unit))
		return SN_ERROR_INVALID_ARGUMENT;
	RRayCast ray(RVec3(Load3(origin)), unit * max_distance);
	RayCastResult result;
	MaskFilter filter(mask);
	if (!w->mSystem.GetNarrowPhaseQuery().CastRay(ray, result, {}, filter))
		return 0;
	RVec3 point = ray.GetPointOnRay(result.mFraction);
	Vec3 normal;
	{
		BodyLockRead lock(w->mSystem.GetBodyLockInterface(), result.mBodyID);
		normal = lock.GetBody().GetWorldSpaceSurfaceNormal(result.mSubShapeID2, point);
	}
	hit->distance = result.mFraction * max_distance;
	Vec3(point).StoreFloat3(reinterpret_cast<Float3 *>(hit->point));
	normal.StoreFloat3(reinterpret_cast<Float3 *>(hit->normal));
	hit->body = BodyIndexOf(*w, result.mBodyID);
	return 1;
}

SN_API int32_t sn_physics_static_triangles(int32_t world, uint32_t mask, float **out_vertices)
{
	World *w = GetWorld(world);
	if (w == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (out_vertices == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "out_vertices is required");
	*out_vertices = nullptr;
	std::vector<Float3> vertices;
	constexpr int cBatch = Shape::cGetTrianglesMinTrianglesRequested;
	Float3 batch[3 * cBatch];
	for (const BodyRecord &record : w->mBodies)
	{
		if (record.mRemoved || record.mMotion != SN_MOTION_STATIC || (record.mFlags & SN_BODY_SENSOR) != 0)
			continue;
		BodyLockRead lock(w->mSystem.GetBodyLockInterface(), record.mID);
		if (!lock.Succeeded())
			continue;
		const Body &body = lock.GetBody();
		if ((mask >> CollisionLayerOf(body.GetObjectLayer()) & 1) == 0)
			continue;
		const Shape *shape = body.GetShape();
		Shape::GetTrianglesContext context;
		shape->GetTrianglesStart(context, AABox::sBiggest(), body.GetCenterOfMassPosition(), body.GetRotation(), Vec3::sOne());
		for (int count; (count = shape->GetTrianglesNext(context, cBatch, batch)) > 0;)
			vertices.insert(vertices.end(), batch, batch + 3 * count);
	}
	if (vertices.empty())
		return 0;
	void *data = sn_alloc(uint32_t(vertices.size() * sizeof(Float3)));
	if (data == nullptr)
		return SetLastError(SN_ERROR_FAILED, "sn_physics_static_triangles: out of memory");
	std::memcpy(data, vertices.data(), vertices.size() * sizeof(Float3));
	*out_vertices = static_cast<float *>(data);
	return int32_t(vertices.size() / 3);
}

SN_API int32_t sn_physics_sphere_cast(int32_t world, float radius, const float *origin, const float *direction, float max_distance, uint32_t mask, uint32_t flags, sn_hit *hit)
{
	World *w = GetWorld(world);
	if (w == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (origin == nullptr || direction == nullptr || hit == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "origin, direction and hit are required");
	Vec3 unit;
	if (!QueryDirection(direction, max_distance, unit))
		return SN_ERROR_INVALID_ARGUMENT;
	if (!(radius > 0.0f))
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "radius must be positive");
	RefConst<Shape> sphere = new SphereShape(radius);
	RShapeCast cast(sphere, Vec3::sOne(), RMat44::sTranslation(RVec3(Load3(origin))), unit * max_distance);
	ShapeCastSettings settings;
	ClosestCastHit collector((flags & SN_CAST_IGNORE_INITIAL_OVERLAP) != 0);
	MaskFilter filter(mask);
	w->mSystem.GetNarrowPhaseQuery().CastShape(cast, settings, RVec3::sZero(), collector, {}, filter);
	if (!collector.mHadHit)
		return 0;
	const ShapeCastResult &result = collector.mHit;
	hit->distance = result.mFraction * max_distance;
	Vec3 point(result.mContactPointOn2);
	point.StoreFloat3(reinterpret_cast<Float3 *>(hit->point));
	Vec3 normal = result.mPenetrationAxis.IsNearZero() ? -unit : -result.mPenetrationAxis.Normalized();
	normal.StoreFloat3(reinterpret_cast<Float3 *>(hit->normal));
	hit->body = BodyIndexOf(*w, result.mBodyID2);
	return 1;
}
