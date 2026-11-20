# 编译与下载

Windows 下无需 Bash，使用 PowerShell 5.1 或更新版本，安装 CMake（>= 3.22）、Ninja、
Arm GNU Toolchain 和 SEGGER J-Link，并将编译工具加入 PATH。
脚本自动查找标准安装目录下的 CMake 和 J-Link。

```powershell
.\build.bat                       # Debug 增量编译
.\build.bat Release               # Release 编译
.\build.bat Debug -Clean -Jobs 8   # 清理后重新编译
.\download.bat                    # 编译 Debug，通过 J-Link 下载并复位运行
.\download.bat Release            # 编译并下载 Release
.\download.bat -SkipBuild          # 下载已有 Debug 固件
.\download.bat -DryRun             # 编译并查看下载命令，不连接硬件
```

也可以直接执行 `build.ps1`、`download.ps1`。BAT 入口自动使用当前进程的执行策略运行
PowerShell 脚本，无需更改系统执行策略。所有脚本均可从其他工作目录调用。
失败时返回非零退出码，编译失败后不会继续下载。

```powershell
$env:GCC_ARM_PATH = 'D:\CodeTools\Compiler\arm-none-eabi-gcc\bin'
$env:NINJA_PATH = 'C:\Tools\Ninja'
$env:CMAKE_PATH = 'C:\Program Files\CMake\bin'
$env:JLINK_EXE = 'C:\Program Files\SEGGER\JLink_V892\JLink.exe'
.\download.bat -Speed 1000 -SerialNumber 123456789
```

上述工具路径仅为示例，按本机安装位置设置；已在 PATH 中的工具无需设置。
下载参数支持 `-Device`、`-Interface`、`-Speed`、`-SerialNumber`、`-JLinkExe`，
也兼容 `JLINK_DEVICE`、`JLINK_INTERFACE`、`JLINK_SPEED`、`JLINK_SN`、`JLINK_EXE` 环境变量。

clangd 使用 `build/Debug/compile_commands.json` 中的芯片宏和头文件路径，
首次打开或新增源文件/头文件目录后执行一次 `build.bat`。
首次仅编译 Release 时，Windows 脚本也会自动配置 Debug 数据库。
`.vscode/settings.json` 允许 clangd 查询 Arm GCC 的标准库路径；
`.clangd` 处理 GCC 专有选项及 HAL 配置头的独立解析。
若编辑器仍显示旧错误，执行 VS Code 命令 `clangd: Restart language server`。

Linux / Bash 使用方式：

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
