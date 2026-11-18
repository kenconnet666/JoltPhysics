// Minimal physics: enough to prove Jolt steps on the module's shared job system.
#include <Core/Core.h>
#include <Core/HandleTable.h>
#include <Runtime/Transforms.h>
#include <StrideNative/Physics.h>
#include <StrideNative/Plugin.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include <vector>

SN_PLUGIN(physics)

namespace
{
	using namespace JPH;

	namespace Layers
	{
		constexpr ObjectLayer cStatic = 0;
		constexpr ObjectLayer cMoving = 1;
		constexpr ObjectLayer cCount = 2;
	}

	class BroadPhaseLayers final : public BroadPhaseLayerInterface
	{
	public:
		uint GetNumBroadPhaseLayers() const override { return Layers::cCount; }
		BroadPhaseLayer GetBroadPhaseLayer(ObjectLayer inLayer) const override { return BroadPhaseLayer(BroadPhaseLayer::Type(inLayer)); }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
		const char *GetBroadPhaseLayerName(BroadPhaseLayer inLayer) const override { return inLayer.GetValue() == Layers::cStatic ? "Static" : "Moving"; }
#endif
	};

	class ObjectVsBroadPhaseFilter final : public ObjectVsBroadPhaseLayerFilter
	{
	public:
		bool ShouldCollide(ObjectLayer inLayer, BroadPhaseLayer inBroadPhaseLayer) const override
		{
			return inLayer == Layers::cMoving || inBroadPhaseLayer.GetValue() == Layers::cMoving;
		}
	};

	class ObjectPairFilter final : public ObjectLayerPairFilter
	{
	public:
		bool ShouldCollide(ObjectLayer inA, ObjectLayer inB) const override
		{
			return inA == Layers::cMoving || inB == Layers::cMoving;
		}
	};

	struct World
	{
		BroadPhaseLayers				mBroadPhaseLayers;
		ObjectVsBroadPhaseFilter		mObjectVsBroadPhase;
		ObjectPairFilter				mObjectPairs;
		PhysicsSystem					mSystem;
		std::vector<BodyID>				mBodies;			///< Creation order
		std::vector<int32_t>			mSlots;				///< 每个物体绑定的变换槽位，0 表示没有；和 mBodies 一一对应
		int32_t							mTransforms = 0;	///< 绑定用的变换存储句柄，0 表示没有
		std::vector<float>				mPositions;			///< xyz per body; sized once so callers can keep a view
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

	/// 动态物体的位姿写回绑定的槽位。静态物体不动，不用写。
	void WriteTransforms(const World &inWorld, StrideNative::TransformStore &ioStore)
	{
		const BodyInterface &bodies = inWorld.mSystem.GetBodyInterfaceNoLock();
		for (size_t i = 0; i < inWorld.mBodies.size(); ++i)
		{
			int32_t slot = inWorld.mSlots[i];
			if (slot == 0 || bodies.GetMotionType(inWorld.mBodies[i]) != EMotionType::Dynamic)
				continue;
			RVec3 position;
			Quat rotation;
			bodies.GetPositionAndRotation(inWorld.mBodies[i], position, rotation);
			ioStore.SetWorldPose(slot, Vec3(position), rotation);
		}
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
			RVec3 position = bodies.GetCenterOfMassPosition(ioWorld.mBodies[i]);
			float *out = &ioWorld.mPositions[i * 3];
			out[0] = float(position.GetX());
			out[1] = float(position.GetY());
			out[2] = float(position.GetZ());
		}
	}
}

using namespace StrideNative;

namespace
{
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
}

SN_API int32_t sn_physics_create(uint32_t max_bodies, int32_t transforms)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	EnsureTypesRegistered();
	if (max_bodies == 0)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "max_bodies must be positive");
	if (transforms != 0 && LookupTransformStore(transforms) == nullptr)
		return SN_ERROR_INVALID_HANDLE;

	// Resting bodies touch several others (a box in a stack has up to four neighbours and the
	// ground), so pairs and contacts need headroom over the body count.
	constexpr uint32_t cContactsPerBody = 8;
	auto world = std::make_unique<World>();
	world->mSystem.Init(max_bodies, 0, max_bodies * cContactsPerBody, max_bodies * cContactsPerBody,
		world->mBroadPhaseLayers, world->mObjectVsBroadPhase, world->mObjectPairs);
	world->mBodies.reserve(max_bodies);
	world->mSlots.reserve(max_bodies);
	world->mTransforms = transforms;
	world->mPositions.resize(size_t(max_bodies) * 3);
	return GetTable<World>().Add(std::move(world));
}

SN_API void sn_physics_destroy(int32_t world)
{
	std::unique_ptr<World> removed = GetTable<World>().Remove(world);
	if (removed == nullptr || removed->mBodies.empty())
		return;
	BodyInterface &bodies = removed->mSystem.GetBodyInterface();
	int count = int(removed->mBodies.size());
	bodies.RemoveBodies(removed->mBodies.data(), count);
	bodies.DestroyBodies(removed->mBodies.data(), count);
}

SN_API int32_t sn_physics_add_boxes(int32_t world, const sn_box *boxes, uint32_t count)
{
	World *w = GetWorld(world);
	if (w == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	size_t first = w->mBodies.size();
	if (count == 0)
		return int32_t(first);
	if (boxes == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "boxes are required");
	if (first + count > Capacity(*w))
		return SetLastError(SN_ERROR_FAILED, "body limit reached");
	TransformStore *store = nullptr;
	for (uint32_t i = 0; i < count; ++i)
	{
		if (boxes[i].transform == 0)
			continue;
		if (w->mTransforms == 0)
			return SetLastError(SN_ERROR_INVALID_ARGUMENT, "box " + std::to_string(i) + " has a transform but the world was created without a transform store");
		if (store == nullptr && (store = BoundStore(*w)) == nullptr)
			return SN_ERROR_INVALID_HANDLE;
		if (!store->IsAlive(boxes[i].transform))
			return SetLastError(SN_ERROR_INVALID_ARGUMENT, "box " + std::to_string(i) + ": transform slot " + std::to_string(boxes[i].transform) + " is not allocated");
	}

	BodyInterface &bodies = w->mSystem.GetBodyInterface();
	std::vector<BodyID> added;
	added.reserve(count);
	// Consecutive boxes of the same size share one shape, as Jolt recommends for identical bodies
	RefConst<Shape> shape;
	Vec3 shape_half_extent = Vec3::sZero();
	for (uint32_t i = 0; i < count; ++i)
	{
		const sn_box &box = boxes[i];
		Vec3 half_extent(box.half_extent[0], box.half_extent[1], box.half_extent[2]);
		if (shape == nullptr || half_extent != shape_half_extent)
		{
			shape = new BoxShape(half_extent);
			shape_half_extent = half_extent;
		}
		// 绑定了变换的物体从槽位现在的世界位姿出发（缩放不作用到形状上）
		RVec3 position(box.position[0], box.position[1], box.position[2]);
		Quat rotation = Quat::sIdentity();
		if (box.transform != 0)
		{
			Mat44 world = store->ComputeWorldMatrix(box.transform);
			Vec3 scale;
			position = RVec3(world.GetTranslation());
			rotation = world.Decompose(scale).GetQuaternion().Normalized();
		}
		bool dynamic = (box.flags & SN_BODY_DYNAMIC) != 0;
		BodyCreationSettings settings(shape, position, rotation,
			dynamic ? EMotionType::Dynamic : EMotionType::Static, dynamic ? Layers::cMoving : Layers::cStatic);
		Body *body = bodies.CreateBody(settings);
		if (body == nullptr)
		{
			bodies.DestroyBodies(added.data(), int(added.size()));
			return SetLastError(SN_ERROR_FAILED, "body limit reached");
		}
		added.push_back(body->GetID());
	}

	// mBodies keeps creation order; AddBodiesPrepare reorders the array it is given
	w->mBodies.insert(w->mBodies.end(), added.begin(), added.end());
	for (uint32_t i = 0; i < count; ++i)
		w->mSlots.push_back(boxes[i].transform);
	BodyInterface::AddState state = bodies.AddBodiesPrepare(added.data(), int(count));
	bodies.AddBodiesFinalize(added.data(), int(count), state, EActivation::Activate);
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
	EPhysicsUpdateError errors = w->mSystem.Update(delta_time, collision_steps > 0 ? collision_steps : 1, &GetTempAllocator(), &GetJobSystem());
	WritePositions(*w, 0);
	if (store != nullptr)
		WriteTransforms(*w, *store);
	if (errors == EPhysicsUpdateError::None)
		return SN_OK;
	if ((errors & EPhysicsUpdateError::BodyPairCacheFull) != EPhysicsUpdateError::None)
		return SetLastError(SN_ERROR_FAILED, "physics step: body pair limit reached");
	if ((errors & EPhysicsUpdateError::ContactConstraintsFull) != EPhysicsUpdateError::None)
		return SetLastError(SN_ERROR_FAILED, "physics step: contact constraint limit reached");
	return SetLastError(SN_ERROR_FAILED, "physics step: contact cache full (temp allocator too small)");
}

SN_API const float *sn_physics_positions(int32_t world)
{
	World *w = GetWorld(world);
	return w != nullptr ? w->mPositions.data() : nullptr;
}
