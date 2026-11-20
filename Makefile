.DEFAULT_GOAL := all
BUILD_TYPE ?= Debug

.PHONY: all build debug release rebuild download flash clean help
all: build

build:
	@./build.sh "$(BUILD_TYPE)"

debug:
	@./build.sh Debug

release:
	@./build.sh Release

rebuild:
	@./build.sh "$(BUILD_TYPE)" --clean

download flash:
	@./download.sh "$(BUILD_TYPE)"

clean:
	@if [ -f "build/$(BUILD_TYPE)/CMakeCache.txt" ]; then \
		cmake --build "build/$(BUILD_TYPE)" --target clean; \
	fi

help:
	@echo 'make [build|debug|release|rebuild|download|flash|clean] [BUILD_TYPE=Debug|Release]'
	@echo 'JLINK_DEVICE=STM32F427II JLINK_INTERFACE=SWD JLINK_SPEED=4000 JLINK_SN=序列号'
