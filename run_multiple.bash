#!/bin/bash

#python -m evaluation.parallel_simulation.start_parallel --nparallel 2 --config evaluation/parallel_simulation/defs/straight_road_80m_centercam_pos1.json --repeats 100 --out evaluation/parallel_simulation/gen --continue

#python -m evaluation.parallel_simulation.start_parallel --nparallel 2 --config evaluation/parallel_simulation/defs/straight_road_80m_centercam_pos2.json --repeats 100 --out evaluation/parallel_simulation/gen --continue

python -m evaluation.parallel_simulation.start_parallel --nparallel 1 --config evaluation/parallel_simulation/defs/straight_road_80m_centercam_pos3.json --repeats 1000 --out evaluation/parallel_simulation/gen --continue
