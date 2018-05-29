#!/bin/bash

#python -m evaluation.parallel_simulation.start_parallel --nparallel 2 --config evaluation/parallel_simulation/defs/calibration_circuit.json --repeats 5 --out evaluation/parallel_simulation/gen --restart
rm -rf ~/.ros/log
#python -m evaluation.parallel_simulation.start_parallel --nparallel 2 --config evaluation/parallel_simulation/defs/calibration_circuit_mirror.json --repeats 5 --out evaluation/parallel_simulation/gen --restart
rm -rf ~/.ros/log

#python -m evaluation.parallel_simulation.start_parallel --nparallel 1 --config evaluation/parallel_simulation/defs/straight_road_80m_centercam_pos1.json --repeats 1 --out evaluation/parallel_simulation/gen --continue
rm -rf ~/.ros/log
#python -m evaluation.parallel_simulation.start_parallel --nparallel 1 --config evaluation/parallel_simulation/defs/straight_road_80m_centercam_pos2.json --repeats 1 --out evaluation/parallel_simulation/gen --continue
rm -rf ~/.ros/log
#python -m evaluation.parallel_simulation.start_parallel --nparallel 1 --config evaluation/parallel_simulation/defs/straight_road_80m_centercam_pos3.json --repeats 1 --out evaluation/parallel_simulation/gen --continue
rm -rf ~/.ros/log

#python -m evaluation.parallel_simulation.start_parallel --nparallel 1 --config evaluation/parallel_simulation/defs/straight_road_80m_halfendcam_pos1.json --repeats 1 --out evaluation/parallel_simulation/gen --continue
rm -rf ~/.ros/log
#python -m evaluation.parallel_simulation.start_parallel --nparallel 1 --config evaluation/parallel_simulation/defs/straight_road_80m_halfendcam_pos2.json --repeats 1 --out evaluation/parallel_simulation/gen --continue
rm -rf ~/.ros/log
#python -m evaluation.parallel_simulation.start_parallel --nparallel 1 --config evaluation/parallel_simulation/defs/straight_road_80m_halfendcam_pos3.json --repeats 1 --out evaluation/parallel_simulation/gen --continue
rm -rf ~/.ros/log
#python -m evaluation.parallel_simulation.start_parallel --nparallel 1 --config evaluation/parallel_simulation/defs/straight_road_80m_halfstartcam_pos1.json --repeats 1 --out evaluation/parallel_simulation/gen --continue
rm -rf ~/.ros/log
#python -m evaluation.parallel_simulation.start_parallel --nparallel 1 --config evaluation/parallel_simulation/defs/straight_road_80m_halfstartcam_pos2.json --repeats 1 --out evaluation/parallel_simulation/gen --continue
rm -rf ~/.ros/log
#python -m evaluation.parallel_simulation.start_parallel --nparallel 1 --config evaluation/parallel_simulation/defs/straight_road_80m_halfstartcam_pos3.json --repeats 1 --out evaluation/parallel_simulation/gen --continue
rm -rf ~/.ros/log


python -m evaluation.parallel_simulation.start_parallel --nparallel 1 --config evaluation/parallel_simulation/defs/radius_20_120m_centercam_pos1.json --repeats 1 --out evaluation/parallel_simulation/gen --continue
rm -rf ~/.ros/log
python -m evaluation.parallel_simulation.start_parallel --nparallel 1 --config evaluation/parallel_simulation/defs/radius_20_120m_centercam_pos2.json --repeats 1 --out evaluation/parallel_simulation/gen --continue
rm -rf ~/.ros/log
python -m evaluation.parallel_simulation.start_parallel --nparallel 1 --config evaluation/parallel_simulation/defs/radius_5_31m_centercam_pos1.json --repeats 1 --out evaluation/parallel_simulation/gen --continue
rm -rf ~/.ros/log
