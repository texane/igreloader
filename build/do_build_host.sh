#!/usr/bin/env sh

gcc -Wall \
-I../src/host \
../src/host/main.c \
../src/host/hex.c \
../src/host/serial.c
