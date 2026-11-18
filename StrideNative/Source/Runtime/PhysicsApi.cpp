// Minimal physics: enough to prove Jolt steps on the module's shared job system.
#include <Core/Core.h>
#include <Core/HandleTable.h>
#include <StrideNative/Runtime.h>

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/PhysicsSystem.h>

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
		std::vector<BodyID>				mBodies;			///< Creation order
		std::vector<float>				mPositions;			///< xyz per body; sized once so callers can keep a view
	};

	World *GetWorld(int32_t inHandle)
	{
		return StrideNative::Lookup<World>(inHandle, "physics world");
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

SN_API int32_t sn_physics_create(uint32_t max_bodies)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	if (max_bodies == 0)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "max_bodies must be positive");

	// Resting bodies touch several others (a box in a stack has up to four neighbours and the
	// ground), so pairs and contacts need headroom over the body count.
	constexpr uint32_t cContactsPerBody = 8;
	auto world = std::make_unique<World>();
	world->mSystem.Init(max_bodies, 0, max_bodies * cContactsPerBody, max_bodies * cContactsPerBody,
		world->mBroadPhaseLayers, world->mObjectVsBroadPhase, world->mObjectPairs);
	world->mBodies.reserve(max_bodies);
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
		bool dynamic = (box.flags & SN_BODY_DYNAMIC) != 0;
		BodyCreationSettings settings(shape, RVec3(box.position[0], box.position[1], box.position[2]), Quat::sIdentity(),
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
	EPhysicsUpdateError errors = w->mSystem.Update(delta_time, collision_steps > 0 ? collision_steps : 1, &GetTempAllocator(), &GetJobSystem());
	WritePositions(*w, 0);
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
