// Serves the browser test page with the headers shared memory needs (COOP/COEP).
// Fixtures the runtime consumes are produced here first with stride-tools, as the editor would.
//   node StrideNative/Tests/Browser/serve.mjs [port]     (测的是 npm 包的 JS/dist，先跑 Scripts/build-wasm.ps1)
import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const dist = path.join(here, '..', '..', 'JS', 'dist');
const port = Number(process.argv[2] ?? 8190);

const { createStrideTools } = await import(pathToFileURL(path.join(dist, 'stride-tools.js')));

async function makeFixtures() {
	const tools = await createStrideTools();
	const size = 256;
	const rgba = new Uint8Array(size * size * 4);
	for (let i = 0; i < size * size; ++i)
		rgba.set([i % size, Math.floor(i / size), 160, 255], i * 4);

	const vertexCount = 20000;
	const vertices = new Float32Array(vertexCount * 4).map((_, i) => Math.sin(i * 0.37));

	const cells = 60, extent = 120;
	const ground = [], triangles = [];
	for (let z = 0; z <= cells; ++z)
		for (let x = 0; x <= cells; ++x)
			ground.push(-extent / 2 + (extent * x) / cells, 0, -extent / 2 + (extent * z) / cells);
	for (let z = 0; z < cells; ++z)
		for (let x = 0; x < cells; ++x) {
			const i = z * (cells + 1) + x;
			triangles.push(i, i + cells + 1, i + 1, i + 1, i + cells + 1, i + cells + 2);
		}

	const s = Math.SQRT1_2;
	const [ktx2, navmesh] = await Promise.all([
		tools.encodeBasis(rgba, size, size, { mode: 'uastc', mipmaps: true }),
		tools.bakeNavMesh(new Float32Array(ground), new Int32Array(triangles), { tileSize: 32 }),
	]);
	const fixtures = {
		'texture.ktx2': ktx2,
		'navmesh.bin': navmesh,
		'ground-vertices.bin': new Uint8Array(new Float32Array(ground).buffer),
		'ground-triangles.bin': new Uint8Array(new Int32Array(triangles).buffer),
		'vertices.bin': new Uint8Array(vertices.buffer),
		'vertices.meshopt': tools.encodeVertices(new Uint8Array(vertices.buffer), 16),
		'skeleton.ozz': tools.buildSkeleton([{ parent: -1, name: 'root' }, { parent: 0, translation: [0, 1, 0], name: 'child' }]),
		'animation.ozz': tools.buildAnimation(1, [
			{ translations: [[0, 0, 0, 0]], rotations: [[0, 0, 0, 0, 1], [1, 0, 0, s, s]], scales: [[0, 1, 1, 1]] },
			{ translations: [[0, 0, 1, 0]], rotations: [[0, 0, 0, 0, 1]], scales: [[0, 1, 1, 1]] },
		]),
	};
	tools.shutdown();
	return fixtures;
}

const fixtures = await makeFixtures();
const types = { '.html': 'text/html', '.mjs': 'text/javascript', '.js': 'text/javascript', '.map': 'application/json', '.wasm': 'application/wasm' };

createServer(async (request, response) => {
	const url = new URL(request.url, 'http://localhost');
	const headers = {
		'Cross-Origin-Opener-Policy': 'same-origin',
		'Cross-Origin-Embedder-Policy': 'require-corp',
		'Cache-Control': 'no-store',
	};
	try {
		let body, file;
		if (url.pathname.startsWith('/fixtures/')) {
			body = fixtures[url.pathname.slice('/fixtures/'.length)];
			if (!body) throw new Error('unknown fixture');
		} else {
			file = url.pathname === '/' ? path.join(here, 'index.html') : path.join(dist, path.normalize(url.pathname.replace(/^\/dist\//, '')));
			if (!file.startsWith(here) && !file.startsWith(dist)) throw new Error('outside served folders');
			body = await readFile(file);
		}
		response.writeHead(200, { ...headers, 'Content-Type': types[path.extname(file ?? '')] ?? 'application/octet-stream' });
		response.end(body);
	} catch {
		response.writeHead(404, headers);
		response.end();
	}
}).listen(port, () => console.log(`StrideNative browser tests on http://localhost:${port}/`));
