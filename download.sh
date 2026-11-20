#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
usage() {
    echo "用法: $0 [Debug|Release]"
    echo "先编译，再通过 J-Link 下载 ELF 并复位运行。"
    echo "环境变量: JLINK_EXE JLINK_DEVICE JLINK_INTERFACE JLINK_SPEED JLINK_SN"
}
if [[ ${1:-} == -h || ${1:-} == --help ]]; then
    usage
    exit 0
fi
BUILD_TYPE="${1:-Debug}"
if (( $# > 1 )) || [[ $BUILD_TYPE != Debug && $BUILD_TYPE != Release ]]; then
    usage >&2
    exit 2
fi
JLINK_EXE="${JLINK_EXE:-JLinkExe}"
if ! command -v "$JLINK_EXE" >/dev/null 2>&1; then
    echo "找不到 J-Link Commander，请安装或设置 JLINK_EXE=/路径/JLinkExe。" >&2
    exit 1
fi
JLINK_DEVICE="${JLINK_DEVICE:-STM32F427II}"
JLINK_INTERFACE="${JLINK_INTERFACE:-SWD}"
JLINK_SPEED="${JLINK_SPEED:-4000}"

"$ROOT_DIR/build.sh" "$BUILD_TYPE"
FIRMWARE="$ROOT_DIR/build/$BUILD_TYPE/motor_example.elf"
COMMAND_FILE="$(mktemp "${TMPDIR:-/tmp}/motor-example-jlink.XXXXXX")"
trap 'rm -f -- "$COMMAND_FILE"' EXIT
cat > "$COMMAND_FILE" <<EOF
ExitOnError 1
r
h
loadfile "$FIRMWARE"
r
g
exit
EOF

JLINK_ARGS=(-device "$JLINK_DEVICE" -if "$JLINK_INTERFACE" -speed "$JLINK_SPEED"
    -autoconnect 1 -ExitOnError 1 -NoGui 1 -CommandFile "$COMMAND_FILE")
if [[ -n ${JLINK_SN:-} ]]; then
    JLINK_ARGS+=(-USB "$JLINK_SN")
fi
echo "J-Link 下载: $JLINK_DEVICE / $JLINK_INTERFACE / $JLINK_SPEED kHz"
"$JLINK_EXE" "${JLINK_ARGS[@]}"
echo "下载完成，目标已复位运行。"
