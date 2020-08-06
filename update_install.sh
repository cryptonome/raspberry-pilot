#!/bin/bash
wget https://github.com/lhelontra/tensorflow-on-arm/releases/download/v2.3.0/tensorflow-2.3.0-cp37-none-linux_aarch64.whl
python -m pip install tensorflow-2.3.0-cp37-none-linux_aarch64.whl
#rm tensorflow-2.3.0-cp37-none-linux_aarch64.whl