// 从 C 头文件（Include/StrideNative/*.h 的 SN_API 声明）生成 src/native-exports.ts：每个 C 函数在 Emscripten 模块上的
// 签名（参数名、个数、文档注释照抄头文件），以及每个模块导出哪些函数。构建时（pnpm run build）先跑它；生成的文件也提交，
// IDE 不构建也能看到。用法：node scripts/generate-exports.mjs [--check]（--check 时只检查是否需要重新生成）
import { readdirSync, readFileSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const includes = path.join(root, '..', 'Include', 'StrideNative');
const output = path.join(root, 'src', 'native-exports.ts');

/** 每个模块导出的头文件：核心的 C API 每个模块都有；侧模块的导出挂在运行时模块上；工具模块还导出导航的默认配置。 */
const MODULES = {
	CoreExport: { headers: ['Core.h', 'Debug.h'] },
	RuntimeExport: { headers: ['Runtime.h', 'Plugin.h', 'Physics.h', 'Texture.h', 'Navigation.h'] },
	ToolsExport: { headers: ['Tools.h'], extra: ['sn_nav_default_config'] },
};

const VOID = /^\s*void\s*$/;

/** 头文件里的 SN_API 声明：名字、返回类型、参数名、前面的 /// 注释。 */
function parseHeader(file) {
	const lines = readFileSync(path.join(includes, file), 'utf8').split(/\r?\n/);
	const functions = [];
	for (let i = 0; i < lines.length; i++) {
		if (!lines[i].startsWith('SN_API ')) continue;
		const start = i;
		let declaration = lines[i];
		while (!declaration.includes(');')) declaration += ' ' + lines[++i].trim();
		const match = /^SN_API\s+(.+?)\b(s[nt]_\w+)\s*\((.*)\)\s*;/.exec(declaration);
		if (!match) throw new Error(`${file}: cannot parse '${declaration}'`);
		const [, returnType, name, params] = match;
		const docs = [];
		for (let j = start - 1; j >= 0 && lines[j].startsWith('///'); j--) docs.unshift(lines[j].replace(/^\/\/\/\s?/, ''));
		functions.push({
			name,
			returns: /^\s*void\s*$/.test(returnType) ? 'void' : 'number',
			params: VOID.test(params) || params.trim() === '' ? [] : splitParams(params).map(paramName),
			docs,
			file,
		});
	}
	return functions;
}

/** 按最外层的逗号分开参数（函数指针参数里的逗号不算）。 */
function splitParams(params) {
	const parts = [];
	let depth = 0, current = '';
	for (const c of params) {
		if (c === ',' && depth === 0) {
			parts.push(current);
			current = '';
			continue;
		}
		if (c === '(') depth++;
		if (c === ')') depth--;
		current += c;
	}
	return [...parts, current];
}

/** 参数名：普通参数是最后一个词，函数指针 `void (*fn)(void *user)` 是括号里的名字。 */
function paramName(param) {
	return /\(\s*\*\s*(\w+)\s*\)/.exec(param)?.[1] ?? param.trim().split(/[\s*]+/).at(-1);
}

const byName = new Map();
for (const file of readdirSync(includes).filter((name) => name.endsWith('.h'))) {
	for (const fn of parseHeader(file)) byName.set(fn.name, fn);
}

const camel = (name) => name.replace(/_([a-z])/g, (_, c) => c.toUpperCase());
const out = [
	'// 由 scripts/generate-exports.mjs 从 Include/StrideNative/*.h 生成，不要手改：改头文件后重新生成（pnpm run build 会做）。',
	'// 指针、句柄、整数和 float 在 JS 里都是 number。',
	'',
	'/** Emscripten 模块上的 C 函数（名字前加 `_`），参数名和注释同头文件。 */',
	'export interface NativeExports {',
];
for (const fn of [...byName.values()].sort((a, b) => a.name.localeCompare(b.name))) {
	const docs = [...fn.docs, `C: ${fn.file}`];
	out.push(docs.length === 1 ? `\t/** ${docs[0]} */` : `\t/**\n${docs.map((line) => `\t * ${line}`.trimEnd()).join('\n')}\n\t */`);
	out.push(`\t_${fn.name}(${fn.params.map((param) => `${camel(param)}: number`).join(', ')}): ${fn.returns};`);
}
out.push('}', '');
for (const [type, { headers, extra = [] }] of Object.entries(MODULES)) {
	const names = [...byName.values()].filter((fn) => headers.includes(fn.file)).map((fn) => fn.name).concat(extra);
	for (const name of extra) if (!byName.has(name)) throw new Error(`${type}: ${name} is not declared in any header`);
	out.push(`/** ${headers.join('、')}${extra.length ? ` 加上 ${extra.join('、')}` : ''} 里的函数。 */`);
	out.push(`export type ${type} =\n${names.map((name) => `\t| '${name}'`).join('\n')};`, '');
}
const text = out.join('\n');

if (process.argv.includes('--check')) {
	const current = readFileSync(output, 'utf8').replace(/\r\n/g, '\n');
	if (current !== text) {
		console.error('src/native-exports.ts is out of date: run node scripts/generate-exports.mjs');
		process.exit(1);
	}
} else {
	writeFileSync(output, text);
	console.log(`wrote ${path.relative(root, output)}: ${byName.size} functions`);
}
