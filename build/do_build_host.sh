#!/usr/bin/env sh

SCAB_TOP_DIR=$HOME/repo/scab

gcc -Wall \
-I../src/host \
-I$SCAB_TOP_DIR/src/api \
-I$SCAB_TOP_DIR/src/common \
../src/host/main.c \
../src/host/hex.c \
-L$SCAB_TOP_DIR/build/api -lscab_api
