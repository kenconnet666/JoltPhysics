// JS API of the stride-native runtime module (browser and Node). See stride-native.d.ts.
import createStrideNativeCore from './stride-native-core.mjs';
import { NativeModule, NAV_CONFIG_SIZE, instantiate, writeNavConfig } from './native-module.mjs';

export { NativeError, defaultWorkerThreads } from './native-module.mjs';

export const MeshoptMode = Object.freeze({ attributes: 0, triangles: 1, indices: 2 });
export const MeshoptFilter = Object.freeze({ none: 0, octahedral: 1, quaternion: 2, exponential: 3, color: 4 });
/** Transcode targets, named after the WebGPU format family they upload to. */
export const TextureTarget = Object.freeze({ rgba8: 0, bc7: 1, astc4x4: 2, etc2rgba8: 3 });

/** Loads the module and starts its job system. */
export async function createStrideNative(options = {}) {
	return new StrideNative(await instantiate(createStrideNativeCore, options));
}

export class StrideNative extends NativeModule {
	// ---------------------------------------------------------------------------------------------
	// Physics (smoke test)

	createPhysicsWorld(maxBodies = 1024) {
		return new PhysicsWorld(this, this.check(this.module._sn_physics_create(maxBodies), 'sn_physics_create'));
	}

	// ---------------------------------------------------------------------------------------------
	// Meshopt decoding

	/** Decodes one glTF meshopt buffer view; returns count * stride bytes. */
	decodeMeshopt(source, count, stride, mode = MeshoptMode.attributes, filter = MeshoptFilter.none) {
		return this.withArrays([source], (src) => this.withScratch(count * stride, (dst) => {
			this.check(this.module._sn_meshopt_decode(dst, count, stride, src, source.byteLength, mode, filter), 'sn_meshopt_decode');
			return this.copyOut(Uint8Array, dst, count * stride);
		}));
	}

	/** Decodes many buffer views in parallel on the job system. Requests: { source, count, stride, mode, filter }. */
	async decodeMeshoptBatch(requests) {
		const REQUEST_SIZE = 32;
		const table = this.alloc(Math.max(1, requests.length) * REQUEST_SIZE);
		const buffers = [];
		try {
			requests.forEach((request, i) => {
				const src = this.copyIn(request.source);
				const dst = this.alloc(request.count * request.stride);
				buffers.push(src, dst);
				const view = this.view(table + i * REQUEST_SIZE, REQUEST_SIZE);
				view.setUint32(0, dst, true);
				view.setUint32(4, request.count, true);
				view.setUint32(8, request.stride, true);
				view.setUint32(12, src, true);
				view.setUint32(16, request.source.byteLength, true);
				view.setInt32(20, request.mode ?? MeshoptMode.attributes, true);
				view.setInt32(24, request.filter ?? MeshoptFilter.none, true);
				view.setInt32(28, 1, true);
			});
			await this.awaitTask(this.module._sn_meshopt_decode_batch(table, requests.length), 'sn_meshopt_decode_batch');
			return requests.map((request, i) => this.copyOut(Uint8Array, buffers[i * 2 + 1], request.count * request.stride));
		} finally {
			buffers.forEach((ptr) => this.free(ptr));
			this.free(table);
		}
	}

	// ---------------------------------------------------------------------------------------------
	// KTX2 transcoding

	ktx2Info(bytes) {
		return this.withArrays([bytes], (data) => this.withScratch(32, (info) => {
			this.check(this.module._sn_ktx2_get_info(data, bytes.byteLength, info), 'sn_ktx2_get_info');
			const [width, height, levels, layers, faces, hasAlpha, isSrgb, isHdr] = this.i32(info, 8);
			return { width, height, levels, layers, faces, hasAlpha: !!hasAlpha, isSrgb: !!isSrgb, isHdr: !!isHdr };
		}));
	}

	/** Transcodes all mip levels in parallel. Returns { width, height, unitSize, levels: Uint8Array[] }. */
	transcodeKtx2(bytes, target = TextureTarget.rgba8) {
		return this.withArraysAsync([bytes], (data) =>
			this.awaitTask(this.module._sn_ktx2_transcode(data, bytes.byteLength, target), 'sn_ktx2_transcode', (task) => ({
				width: task.result(0),
				height: task.result(1),
				unitSize: task.result(3),
				levels: Array.from({ length: task.result(2) }, (_, level) => task.output(Uint8Array, level)),
			})));
	}

	// ---------------------------------------------------------------------------------------------
	// Navigation

	/** Builds a tiled navmesh from a triangle soup; tiles are built in parallel. */
	buildNavMesh(vertices, triangles, config = {}) {
		return this.withArraysAsync([vertices, triangles], (v, t) => this.withScratch(NAV_CONFIG_SIZE, (cfg) => {
			writeNavConfig(this, cfg, config);
			return this.awaitTask(this.module._sn_nav_build(v, vertices.length / 3, t, triangles.length / 3, cfg), 'sn_nav_build',
				(task) => new NavMesh(this, task.result(0)));
		}));
	}

	/** Loads a navmesh serialized by NavMesh.serialize or baked by stride-tools. */
	loadNavMesh(bytes) {
		return this.withArrays([bytes], (data) => new NavMesh(this, this.check(this.module._sn_nav_load(data, bytes.byteLength), 'sn_nav_load')));
	}

	// ---------------------------------------------------------------------------------------------
	// Animation

	loadSkeleton(bytes) {
		return this.withArrays([bytes], (data) => new Skeleton(this, this.check(this.module._sn_anim_load_skeleton(data, bytes.byteLength), 'sn_anim_load_skeleton')));
	}

	loadAnimation(bytes) {
		return this.withArrays([bytes], (data) => new Animation(this, this.check(this.module._sn_anim_load_animation(data, bytes.byteLength), 'sn_anim_load_animation')));
	}

	/** Samples every { instance, animation, ratio } in parallel and updates the instances' matrices. */
	sampleAnimations(requests) {
		const REQUEST_SIZE = 16;
		return this.withScratch(Math.max(1, requests.length) * REQUEST_SIZE, (table) => {
			requests.forEach(({ instance, animation, ratio }, i) => {
				const view = this.view(table + i * REQUEST_SIZE, REQUEST_SIZE);
				view.setInt32(0, instance.handle, true);
				view.setInt32(4, animation.handle, true);
				view.setFloat32(8, ratio, true);
			});
			this.check(this.module._sn_anim_sample_batch(table, requests.length), 'sn_anim_sample_batch');
		});
	}
}

export class PhysicsWorld {
	constructor(native, handle) {
		this.native = native;
		this.handle = handle;
		this.bodyCount = 0;
	}

	addBox(position, halfExtent, dynamic) {
		const n = this.native;
		return n.withArrays([new Float32Array(position), new Float32Array(halfExtent)], (p, h) => {
			const index = n.check(n.module._sn_physics_add_box(this.handle, p, h, dynamic ? 1 : 0), 'sn_physics_add_box');
			this.bodyCount = index + 1;
			return index;
		});
	}

	/** Steps on the shared job system; this thread runs physics jobs too while it waits. */
	step(deltaTime, collisionSteps = 1) {
		this.native.check(this.native.module._sn_physics_step(this.handle, deltaTime, collisionSteps), 'sn_physics_step');
	}

	/** xyz positions of all bodies in creation order. */
	positions() {
		const n = this.native;
		return n.withScratch(Math.max(1, this.bodyCount) * 12, (out) => {
			const count = n.check(n.module._sn_physics_get_positions(this.handle, out, this.bodyCount), 'sn_physics_get_positions');
			return n.copyOut(Float32Array, out, count * 3);
		});
	}

	destroy() {
		this.native.module._sn_physics_destroy(this.handle);
	}
}

export class NavMesh {
	constructor(native, handle) {
		this.native = native;
		this.handle = handle;
	}

	/** Straight path corners (xyz) from start to end, or throws if there is none. */
	findPath(start, end, searchExtents = [2, 4, 2], maxPoints = 256) {
		const n = this.native;
		return n.withArrays([new Float32Array(start), new Float32Array(end), new Float32Array(searchExtents)], (s, e, x) =>
			n.withScratch(maxPoints * 12, (out) => {
				const count = n.check(n.module._sn_nav_find_path(this.handle, s, e, x, out, maxPoints), 'sn_nav_find_path');
				return n.copyOut(Float32Array, out, count * 3);
			}));
	}

	serialize() {
		const n = this.native;
		return n.readOwnedBuffer((data, size) => n.module._sn_nav_serialize(this.handle, data, size), 'sn_nav_serialize');
	}

	createCrowd(maxAgents, maxAgentRadius) {
		const n = this.native;
		return new Crowd(n, n.check(n.module._sn_crowd_create(this.handle, maxAgents, maxAgentRadius), 'sn_crowd_create'), maxAgents);
	}

	destroy() {
		this.native.module._sn_nav_destroy(this.handle);
	}
}

export class Crowd {
	constructor(native, handle, maxAgents) {
		this.native = native;
		this.handle = handle;
		this.maxAgents = maxAgents;
	}

	addAgent(position, { radius = 0.5, height = 2, maxSpeed = 3.5, maxAcceleration = 8 } = {}) {
		const n = this.native;
		return n.withArrays([new Float32Array(position)], (p) =>
			n.check(n.module._sn_crowd_add_agent(this.handle, p, radius, height, maxSpeed, maxAcceleration), 'sn_crowd_add_agent'));
	}

	removeAgent(agent) {
		this.native.module._sn_crowd_remove_agent(this.handle, agent);
	}

	requestMove(agent, target) {
		const n = this.native;
		n.withArrays([new Float32Array(target)], (t) => n.check(n.module._sn_crowd_request_move(this.handle, agent, t), 'sn_crowd_request_move'));
	}

	update(deltaTime) {
		this.native.module._sn_crowd_update(this.handle, deltaTime);
	}

	/** xyz per agent slot; NaN for empty slots. */
	positions() {
		const n = this.native;
		return n.withScratch(this.maxAgents * 12, (out) => {
			const count = n.check(n.module._sn_crowd_get_positions(this.handle, out, this.maxAgents), 'sn_crowd_get_positions');
			return n.copyOut(Float32Array, out, count * 3);
		});
	}

	destroy() {
		this.native.module._sn_crowd_destroy(this.handle);
	}
}

export class Skeleton {
	constructor(native, handle) {
		this.native = native;
		this.handle = handle;
		this.jointCount = native.module._sn_anim_skeleton_joint_count(handle);
	}

	createInstance() {
		const n = this.native;
		return new AnimationInstance(n, n.check(n.module._sn_anim_instance_create(this.handle), 'sn_anim_instance_create'), this.jointCount);
	}

	destroy() {
		this.native.module._sn_anim_destroy_skeleton(this.handle);
	}
}

export class Animation {
	constructor(native, handle) {
		this.native = native;
		this.handle = handle;
		this.duration = native.module._sn_anim_animation_duration(handle);
	}

	destroy() {
		this.native.module._sn_anim_destroy_animation(this.handle);
	}
}

export class AnimationInstance {
	constructor(native, handle, jointCount) {
		this.native = native;
		this.handle = handle;
		this.jointCount = jointCount;
	}

	/**
	 * Model-space joint matrices (column-major 4x4 each), as a view of native memory: no copy, so it
	 * can go straight to GPUQueue.writeBuffer. Valid until the instance is destroyed.
	 */
	modelMatrices() {
		return this.native.f32(this.native.module._sn_anim_instance_model_matrices(this.handle), this.jointCount * 16);
	}

	destroy() {
		this.native.module._sn_anim_instance_destroy(this.handle);
	}
}
