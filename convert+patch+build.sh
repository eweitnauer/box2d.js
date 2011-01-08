#!/bin/sh
python convert.py
git apply changes.patch
./build.sh
