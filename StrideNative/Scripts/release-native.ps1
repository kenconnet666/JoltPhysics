# 把 npm 包 @stride-ts/native 发布成 GitHub Release：仓库 kenconnet666/JoltPhysics（origin，永远不是上游 jrouwe），
# 标签 native-v<版本>，附件是 build-wasm.ps1 用 pnpm pack 打出的 JS/stride-ts-native-<版本>.tgz。
# 使用方（stride-ts）在 pnpm-workspace.yaml 的 overrides 里引用打印出来的下载地址。以后发布到 npm 就不再需要它。
#   1. 改 JS/package.json 的 version
#   2. pwsh StrideNative/Scripts/build-wasm.ps1      （构建、测试、打包）
#   3. 提交并推送到 origin 的 stride 分支
#   4. pwsh StrideNative/Scripts/release-native.ps1
param(
    [string] $Repo = 'kenconnet666/JoltPhysics'
)

$ErrorActionPreference = 'Stop'
$PSNativeCommandUseErrorActionPreference = $true

$root = Split-Path $PSScriptRoot -Parent
$package = Join-Path $root 'JS'
$version = (Get-Content (Join-Path $package 'package.json') -Raw | ConvertFrom-Json).version
$tag = "native-v$version"
$name = "stride-ts-native-$version.tgz"
$tarball = Join-Path $package $name

if (-not (Test-Path $tarball)) {
    throw "$tarball is missing: run Scripts/build-wasm.ps1 first"
}
# 发布的包要能对应到一个已经推送的提交
if (git -C $root status --porcelain -- .) {
    throw 'StrideNative has uncommitted changes: commit and push them before releasing'
}
$head = git -C $root rev-parse HEAD
git -C $root fetch --quiet origin stride
git -C $root merge-base --is-ancestor $head origin/stride
if ($LASTEXITCODE -ne 0) {
    throw "commit $head is not on origin/stride yet: push it before releasing"
}
# 同一个版本只发一次：使用方的锁文件记着包的校验和，换内容要换版本号
$PSNativeCommandUseErrorActionPreference = $false
gh release view $tag --repo $Repo *> $null
$exists = $LASTEXITCODE -eq 0
$PSNativeCommandUseErrorActionPreference = $true
if ($exists) {
    throw "$tag is already released: bump the version in JS/package.json"
}

gh release create $tag $tarball --repo $Repo --target $head `
    --title "@stride-ts/native $version" `
    --notes "npm package @stride-ts/native $version (StrideNative wasm modules and TypeScript bindings), built from $head. Install it with a pnpm override pointing at the attached tarball."

Write-Output "https://github.com/$Repo/releases/download/$tag/$name"
