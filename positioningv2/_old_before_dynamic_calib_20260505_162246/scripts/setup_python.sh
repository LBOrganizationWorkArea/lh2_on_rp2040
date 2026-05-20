#!/usr/bin/env bash

set -e

python3 -m venv .venv
source .venv/bin/activate

pip install --upgrade pip
pip install pyserial numpy pandas matplotlib scipy opencv-python

echo "Python environment ready."
echo "Activate it with:"
echo "source .venv/bin/activate"
