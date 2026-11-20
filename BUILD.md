# 编译与下载

需要 Bash、CMake（>= 3.22）、Ninja、Arm GNU Toolchain 和 SEGGER J-Link。
`build.sh` 优先使用 PATH 中的工具，也会查找 Linux 下 STM32Cube 安装的工具包。
自定义路径可设置 `GCC_ARM_PATH`、`NINJA_PATH`（均指向可执行文件所在目录）。

```bash
./build.sh                 # Debug 增量编译
./build.sh Release         # Release 编译
./build.sh Debug --clean   # 清理后重新编译
./download.sh              # 编译 Debug，然后下载并复位运行
./download.sh Release      # 编译并下载 Release
```

产物位于 `build/Debug/` 或 `build/Release/`：
`motor_example.elf`、`motor_example.hex`、`motor_example.bin`、`motor_example.map`。
`User/` 下的 C 源文件及头文件目录自动加入 CMake。

也可以使用 `make`、`make release`、`make rebuild`、`make download`、
`make download BUILD_TYPE=Release`、`make clean`。
脚本可从任意工作目录调用；Makefile 入口在工程根目录使用。

默认目标为 `.ioc` 中的 STM32F427IIH6，对应 J-Link 设备名 `STM32F427II`，
接口 SWD，速度 4000 kHz。连接 SWDIO、SWCLK、GND、VTref，建议连接 NRST，
并给目标板供电。多个探针时用 `JLINK_SN` 指定序列号。

```bash
JLINK_SPEED=1000 JLINK_SN=123456789 ./download.sh
JLINK_EXE=/opt/SEGGER/JLink/JLinkExe ./download.sh
GCC_ARM_PATH=/opt/gcc-arm/bin NINJA_PATH=/opt/ninja JOBS=8 ./build.sh
```

下载使用带地址信息的 ELF，无需手动指定 Flash 地址。
Commander 加载固件时执行写入校验，成功后复位运行；发生错误立即退出。
命令参数参见 [SEGGER J-Link Commander 文档](https://kb.segger.com/J-Link_Commander)。
