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
#include <Jolt/Physics/Character/Character.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include <algorithm>
#include <cmath>
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
		BodyID mID;
		uint32_t mMotion = SN_MOTION_STATIC;
		uint32_t mFlags = 0;
		int32_t mSlot = 0;					///< 绑定的变换槽位，0 表示没有
		int32_t mCharacter = -1;			///< mCharacters 里的下标
		RVec3 mPreviousPosition, mPosition;	///< 插值用：上一步和这一步的位姿
		Quat mPreviousRotation = Quat::sIdentity(), mRotation = Quat::sIdentity();
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
		std::vector<RefConst<Shape>>	mMeshes;			///< 网格句柄是下标加 1
		int32_t							mTransforms = 0;	///< 绑定用的变换存储句柄，0 表示没有
		std::vector<float>				mPositions;			///< 每个物体的 xyz；只分配一次，调用方可以一直拿着视图
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

	size_t Capacity(const World &inWorld)
	{
		return inWorld.mPositions.size() / 3;
	}

	void WritePositions(World &ioWorld, size_t inFirst)
	{
		const BodyInterface &bodies = ioWorld.mSystem.GetBodyInterfaceNoLock();
		for (size_t i = inFirst; i < ioWorld.mBodies.size(); ++i)
		{
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
			if (!Moves(record.mMotion))
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
		case SN_SHAPE_MESH:
		{
			if (inShape.mesh <= 0 || size_t(inShape.mesh) > inWorld.mMeshes.size())
			{
				outError = "mesh handle " + std::to_string(inShape.mesh) + " does not exist";
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
	}

	/// 物体编号对应的记录；编号不对时记下错误
	BodyRecord *GetBody(World &ioWorld, int32_t inBody)
	{
		if (inBody < 0 || size_t(inBody) >= ioWorld.mBodies.size())
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
	return GetTable<World>().Add(std::move(world));
}

SN_API void sn_physics_destroy(int32_t world)
{
	std::unique_ptr<World> removed = GetTable<World>().Remove(world);
	if (removed == nullptr)
		return;
	for (CharacterRecord &record : removed->mCharacters)
		record.mCharacter->RemoveFromPhysicsSystem(false);
	removed->mCharacters.clear();	// 角色自己销毁它的物体
	std::vector<BodyID> ids;
	for (const BodyRecord &record : removed->mBodies)
		if (record.mCharacter < 0)
			ids.push_back(record.mID);
	if (ids.empty())
		return;
	BodyInterface &bodies = removed->mSystem.GetBodyInterface();
	bodies.RemoveBodies(ids.data(), int(ids.size()));
	bodies.DestroyBodies(ids.data(), int(ids.size()));
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
	MeshShapeSettings settings(std::move(vertices), std::move(triangles));
	ShapeSettings::ShapeResult result = settings.Create();
	if (result.HasError())
		return SetLastError(SN_ERROR_FAILED, std::string("mesh shape: ") + result.GetError().c_str());
	w->mMeshes.push_back(result.Get());
	return int32_t(w->mMeshes.size());
}

SN_API int32_t sn_physics_add_bodies(int32_t world, const sn_body *bodies, uint32_t count, const sn_shape *shapes, uint32_t shape_count,
	const sn_character *characters, uint32_t character_count)
{
	World *w = GetWorld(world);
	if (w == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	size_t first = w->mBodies.size();
	if (count == 0)
		return int32_t(first);
	if (bodies == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "bodies are required");
	if (first + count > Capacity(*w))
		return SetLastError(SN_ERROR_FAILED, "body limit reached");

	// 先全部检查、建好形状，出错时什么都不加
	TransformStore *store = nullptr;
	std::vector<RefConst<Shape>> body_shapes(count);
	std::vector<RMat44> poses(count);
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
		}
		const sn_shape *own = shapes + body.first_shape;
		for (uint32_t s = 0; s < body.shape_count; ++s)
			if (own[s].type == SN_SHAPE_MESH && Moves(body.motion))
				return SetLastError(SN_ERROR_INVALID_ARGUMENT, where + "mesh shapes are only allowed on static and kinematic bodies");
		std::string error;
		body_shapes[i] = MakeBodyShape(*w, own, body.shape_count, scale, error);
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
	for (uint32_t i = 0; i < count; ++i)
	{
		const sn_body &body = bodies[i];
		int32_t index = int32_t(first + i);
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
			if (motion == EMotionType::Dynamic)
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
		w->mBodies.push_back(record);
	}

	// 普通物体成批插进宽相（AddBodiesPrepare 会重排传入的数组，所以用副本）；角色各自加入
	if (!added.empty())
	{
		BodyInterface::AddState state = body_interface.AddBodiesPrepare(added.data(), int(added.size()));
		body_interface.AddBodiesFinalize(added.data(), int(added.size()), state, EActivation::Activate);
	}
	for (size_t i = first; i < w->mBodies.size(); ++i)
		if (w->mBodies[i].mCharacter >= 0)
			w->mCharacters[w->mBodies[i].mCharacter].mCharacter->AddToPhysicsSystem(EActivation::Activate);
	WritePositions(*w, first);
	return int32_t(first);
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
		UpdateCharacter(*w, character, delta_time);
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
		if ((record.mFlags & SN_BODY_INTERPOLATED) == 0 || record.mSlot == 0 || !Moves(record.mMotion))
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
