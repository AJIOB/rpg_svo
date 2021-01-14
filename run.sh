#!/bin/bash

echo "Start"
cd rpg_svo/svo
mkdir -p build
cd build
cmake ..
make

export SVO_DATASET_DIR="/q/Datasets"
../bin/test_pipeline
