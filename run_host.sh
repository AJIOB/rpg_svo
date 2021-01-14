#!/bin/bash
# Datasets must be located near rpg_svo folder

echo "Start host"
cd "$(dirname "$0/../")"
docker run --rm -it ajiob/svo-run:1 ./rpg_svo/run.sh
