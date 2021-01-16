#!/bin/bash

echo "Start"
cd rpg_svo/svo
mkdir -p build
cd build
cmake ..
make

# Remove all prev iterations
rm -f svo.*.out.csv

export SVO_DATASET_DIR="/q/Datasets"
for i in {1..100}
do
    echo "    Iteration $i"
    ../bin/test_pipeline
    sleep 1
done
