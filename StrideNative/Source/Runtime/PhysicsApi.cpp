// Minimal physics: enough to prove Jolt steps on the module's shared job system.
#include <Core/Core.h>
#include <Core/HandleTable.h>
#include <StrideNative/Runtime.h>

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <algorithm>
#include <vector>

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
		std::vector<BodyID>				mBodies;
	};

	World *GetWorld(int32_t inHandle)
	{
		return StrideNative::Lookup<World>(inHandle, "physics world");
	}
}

using namespace StrideNative;

SN_API int32_t sn_physics_create(uint32_t max_bodies)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	if (max_bodies == 0)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "max_bodies must be positive");

	auto world = std::make_unique<World>();
	world->mSystem.Init(max_bodies, 0, max_bodies, max_bodies, world->mBroadPhaseLayers, world->mObjectVsBroadPhase, world->mObjectPairs);
	return GetTable<World>().Add(std::move(world));
}

SN_API void sn_physics_destroy(int32_t world)
{
	std::unique_ptr<World> removed = GetTable<World>().Remove(world);
	if (removed == nullptr)
		return;
	BodyInterface &bodies = removed->mSystem.GetBodyInterface();
	for (BodyID id : removed->mBodies)
	{
		bodies.RemoveBody(id);
		bodies.DestroyBody(id);
	}
}

SN_API int32_t sn_physics_add_box(int32_t world, const float *position3, const float *half_extent3, int32_t dynamic)
{
	World *w = GetWorld(world);
	if (w == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (position3 == nullptr || half_extent3 == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "position and half extent are required");

	EMotionType motion = dynamic != 0 ? EMotionType::Dynamic : EMotionType::Static;
	BodyCreationSettings settings(new BoxShape(Vec3(half_extent3[0], half_extent3[1], half_extent3[2])),
		RVec3(position3[0], position3[1], position3[2]), Quat::sIdentity(), motion, dynamic != 0 ? Layers::cMoving : Layers::cStatic);
	BodyID id = w->mSystem.GetBodyInterface().CreateAndAddBody(settings, dynamic != 0 ? EActivation::Activate : EActivation::DontActivate);
	if (id.IsInvalid())
		return SetLastError(SN_ERROR_FAILED, "body limit reached");
	w->mBodies.push_back(id);
	return int32_t(w->mBodies.size() - 1);
}

SN_API int32_t sn_physics_step(int32_t world, float delta_time, int32_t collision_steps)
{
	World *w = GetWorld(world);
	if (w == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	EPhysicsUpdateError errors = w->mSystem.Update(delta_time, collision_steps > 0 ? collision_steps : 1, &GetTempAllocator(), &GetJobSystem());
	return errors == EPhysicsUpdateError::None ? SN_OK : SetLastError(SN_ERROR_FAILED, "physics update ran out of capacity");
}

SN_API int32_t sn_physics_get_positions(int32_t world, float *out_xyz, uint32_t max_bodies)
{
	World *w = GetWorld(world);
	if (w == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	const BodyInterface &bodies = w->mSystem.GetBodyInterfaceNoLock();
	uint32_t count = std::min(max_bodies, uint32_t(w->mBodies.size()));
	for (uint32_t i = 0; i < count; ++i)
	{
		RVec3 position = bodies.GetCenterOfMassPosition(w->mBodies[i]);
		out_xyz[i * 3 + 0] = float(position.GetX());
		out_xyz[i * 3 + 1] = float(position.GetY());
		out_xyz[i * 3 + 2] = float(position.GetZ());
	}
	return int32_t(count);
}
