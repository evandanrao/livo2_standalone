# Makefile — standalones/livo2 (hesai_livo2)
#
# Usage:
#   make            → x86 native build (default)
#   make x86        → x86 native build
#   make nx         → incremental rsync + build on Jetson NX (rad-obc)
#   make nx-full    → first-time full setup on Jetson NX
#   make clean      → remove local build/
#
# Parallelism is forwarded automatically via MAKEFLAGS:
#   make -j12       → cmake --build build -- -j12
#   make -j12 nx    → build with -j4 on Jetson (nx controls its own -j)

BUILD_DIR = build

.DEFAULT_GOAL := x86

.PHONY: x86 nx nx-full clean

x86:
	cmake -B $(BUILD_DIR) -DCMAKE_BUILD_TYPE=Release -Wno-dev
	cmake --build $(BUILD_DIR) -- $(MAKEFLAGS)

nx:
	bash scripts/deploy.sh

nx-full:
	bash scripts/deploy.sh --full

clean:
	rm -rf $(BUILD_DIR)
