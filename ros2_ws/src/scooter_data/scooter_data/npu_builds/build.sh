#!/usr/bin/env bash
# Build libsnpe_infer.so on the device (Rubik Pi).
# Needs: g++, the SNPE headers (copy $QAIRT_SDK_ROOT/include/SNPE from your PC),
# and libSNPE.so (the 2.50 one you already copied).
set -e
QAIRT=${QAIRT:-$HOME/qairt}
INC=${INC:-$QAIRT/include}
LIB=${LIB:-$QAIRT/lib}
cd "$(dirname "$0")"

[ -f "$INC/SNPE/SNPE.h" ] || { echo "SNPE headers not found in $INC (set INC=...)"; exit 1; }
[ -f "$LIB/libSNPE.so" ] || { echo "libSNPE.so not found in $LIB (set LIB=...)"; exit 1; }

g++ -O2 -std=c++17 -shared -fPIC snpe_infer.cpp \
    -I"$INC" -L"$LIB" -lSNPE -Wl,-rpath,"$LIB" \
    -o libsnpe_infer.so
echo "Built $(pwd)/libsnpe_infer.so"