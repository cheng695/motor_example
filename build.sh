#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
usage() {
    echo "用法: $0 [Debug|Release] [--clean]"
    echo "环境变量: GCC_ARM_PATH=工具链bin目录 NINJA_PATH=Ninja所在目录 JOBS=并行数"
}
if [[ ${1:-} == -h || ${1:-} == --help ]]; then
    usage
    exit 0
fi
BUILD_TYPE="${1:-Debug}"
case "$BUILD_TYPE" in
    Debug|Release) ;;
    *) usage >&2; exit 2 ;;
esac
if (( $# > 2 )) || [[ -n ${2:-} && ${2:-} != --clean ]]; then
    usage >&2
    exit 2
fi

# Prefer PATH/explicit overrides; otherwise use STM32Cube's bundled tools.
export PATH="${GCC_ARM_PATH:+${GCC_ARM_PATH}:}${NINJA_PATH:+${NINJA_PATH}:}$PATH"
find_cube_tool() {
    local tool="$1" bundle="$2" candidate
    local candidates=()
    command -v "$tool" >/dev/null 2>&1 && return 0
    mapfile -t candidates < <(compgen -G "${HOME}/.local/share/stm32cube/bundles/${bundle}/*/bin/${tool}" | sort -V || true)
    if (( ${#candidates[@]} > 0 )); then
        candidate="${candidates[${#candidates[@]}-1]}"
        export PATH="$(dirname -- "$candidate"):$PATH"
    fi
    if ! command -v "$tool" >/dev/null 2>&1; then
        echo "缺少工具: $tool，请安装或设置 PATH。" >&2
        exit 1
    fi
}
find_cube_tool arm-none-eabi-gcc gnu-tools-for-stm32
find_cube_tool ninja ninja
for tool in cmake arm-none-eabi-objcopy arm-none-eabi-size; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        echo "缺少工具: $tool，请安装或设置 PATH。" >&2
        exit 1
    fi
done
JOBS="${JOBS:-$(getconf _NPROCESSORS_ONLN)}"
if [[ ! $JOBS =~ ^[1-9][0-9]*$ ]]; then
    echo "JOBS 必须为正整数。" >&2
    exit 2
fi

cd -- "$ROOT_DIR"
cmake --preset "$BUILD_TYPE"
BUILD_ARGS=(--build --preset "$BUILD_TYPE" --parallel "$JOBS")
if [[ ${2:-} == --clean ]]; then
    BUILD_ARGS+=(--clean-first)
fi
cmake "${BUILD_ARGS[@]}"
echo "编译完成: $ROOT_DIR/build/$BUILD_TYPE/motor_example.{elf,hex,bin}"
