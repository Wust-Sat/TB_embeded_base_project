#!/bin/bash

sudo apt-get install -y cmake ninja-build clang ccache dfu-util stlink-tools python3-pip python3-venv

git submodule update --init --recursive