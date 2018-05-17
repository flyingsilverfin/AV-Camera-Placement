import time
import multiprocessing
import subprocess
import argparse
import os
import json
from ..LoggingServer import Logger 
from ..SimulationExperimentRunner import Runner 


# general idea: use Popen to launch a couple of instance launchers
# pass this instance launcher the config file to use
# and its port to use...


def wait_for_finish(popen_runners):
    finished = False
    start_time = time.time()
    while not finished:
        time.sleep(0.5)
        finished = True
        for runner in popen_runners:
            if runner.poll() is None:
                finished = False

    elapsed = time.time() - start_time
    print("------Finished, took {0} seconds -------".format(elapsed))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Launch parallel instances of simulation, with a specified config file')

    parser.add_argument("--config", required=True, help="Config file (or folder if implemented) that defines experiment to launch")
    parser.add_argument("--nparallel", type=int, help="Number of instances to launch in parallel (defaults to cores/2)")
    parser.add_argument("--repeats", type=int, help="Number of times to perform the experiment, defaults to specification in config file or 1000")
    parser.add_argument("--out", required=True, help="Root Directory to save simulation results/data to")
    parser.add_argument('--rviz', dest='rviz', action='store_true')
    parser.add_argument('--no-rviz', dest='rviz', action='store_false')
    parser.add_argument('--continue', dest='continue_exec', action='store_true')
    parser.add_argument('--restart', dest='continue_exec', action='store_false')

    parser.set_defaults(continue_exec=True) # by default, don't overwrite existing runs
    parser.set_defaults(rviz=False)

    args = parser.parse_args()

    config = args.config
    config_name = config.split('/')[-1].split('.')[0]

    with open(config) as f:
        blob = json.load(f)


    if args.repeats is None:
        if "repeats" in blob:
            total_runs = int(blob["repeats"])
        else:
            total_runs = 1000
    else:
        total_runs = args.repeats

    rviz = args.rviz
    rviz_string = "--rviz" if rviz else "--no-rviz"

    if args.nparallel is None:
        n = int(multiprocessing.cpu_count()/2)
    else:
        n = args.nparallel

    continue_string = "--continue" if args.continue_exec else "--restart"
    

    runs_per_instance = total_runs/n # deal with rounding, last one will do fewer experiments
    last_instance_runs = total_runs - runs_per_instance*(n-1) 
    save_dir = os.path.join(args.out, config_name) 

    try:
        os.makedirs(save_dir)
    except OSError as e:
        print(e)


    base_ros_port = 11000
    base_gazebo_port=13000
    runners = []
    for i in range(n-1):

        ros_port = base_ros_port + i
        gazebo_port = base_gazebo_port + i
        out_dir = os.path.join(save_dir, "runner_{}".format(i))
        s = 'export ROS_MASTER_URI=http://localhost:{0}/; export GAZEBO_MASTER_URI=${{GAZEBO_MASTER_URI:-"http://localhost:{6}"}}; python -m {4}.start_single --port={0} --repeats={1} --config={2} --out={3} {5} {7}'.format(ros_port, runs_per_instance, config, out_dir, __package__, rviz_string, gazebo_port, continue_string)
        print("Launching with: " + s)
        proc = subprocess.Popen([s], shell=True)
        runners.append(proc)

    # last one may do fewer repeats
    ros_port = base_ros_port + n - 1
    gazebo_port = base_gazebo_port + n - 1
    out_dir = os.path.join(save_dir, "runner_{}".format(n-1))

    s = 'export ROS_MASTER_URI=http://localhost:{0}/; export GAZEBO_MASTER_URI=${{GAZEBO_MASTER_URI:-"http://localhost:{6}"}}; python -m {4}.start_single --port={0} --repeats={1} --config={2} --out={3} {5} {7}'.format(ros_port, last_instance_runs, config, out_dir, __package__, rviz_string, gazebo_port, continue_string)
    print("Launching with: " + s)
    proc = subprocess.Popen([s], shell=True)
    runners.append(proc)

    wait_for_finish(runners)
    


