#!/bin/bash

./after_ioc.sh

cmake -B build -G "Ninja"
cmake --build build
