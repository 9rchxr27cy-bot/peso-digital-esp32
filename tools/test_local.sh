#!/bin/sh
# Roda os mesmos testes de `pio test -e native` sem PlatformIO (so g++ e git).
# Uso: tools/test_local.sh [pasta_do_Unity]
set -e
cd "$(dirname "$0")/.."
UNITY="${1:-.pio/unity}"
[ -d "$UNITY/src" ] || git clone -q --depth 1 https://github.com/ThrowTheSwitch/Unity "$UNITY"
mkdir -p .pio
gcc -c -O2 -DUNITY_INCLUDE_DOUBLE "$UNITY/src/unity.c" -o .pio/unity.o
g++ -std=gnu++17 -Wall -Wextra -O2 -Ilib/core -Ilib/sim -I"$UNITY/src" \
  lib/core/*.cpp lib/sim/*.cpp test/test_native/test_main.cpp .pio/unity.o -o .pio/test_local
.pio/test_local
