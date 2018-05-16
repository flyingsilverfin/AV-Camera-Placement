import time
import multiprocessing
import subprocess
import argparse
import os
from ..LoggingServer import Logger 
from ..SimulationExperimentRunner import Runner 


# general idea: use Popen to launch a couple of instance launchers
# pass this instance launcher the config file to use
# and its port to use...


def wait_for_finish(popen_runners):
    finished = False
    while not finished:
        time.sleep(1)
        finished = True
        for runner in popen_runners:
            if runner.poll() is None:
                finished = False


if __name__ == "__main__":
    print("pkg: {0}".format(__package__))
    parser = argparse.ArgumentParser(description='Launch parallel instances of simulation, with a specified config file')

    parser.add_argument("--config", required=True, help="Config file (or folder if implemented) that defines experiment to launch")
    parser.add_argument("--nparallel", type=int, help="Number of instances to launch in parallel (defaults to cores/2)")
    parser.add_argument("--repeats", type=int, help="Number of times to perform the experiment", required=True)
    parser.add_argument("--out", required=True, help="Root Directory to save simulation results/data to")

    args = parser.parse_args()

    config = args.config
    config_name = config.split('/')[-1].split('.')[0]


    if args.nparallel is None:
        n = int(multiprocessing.cpu_count()/2)
    else:
        n = args.nparallel

    

    repeats = args.repeats/n # deal with rounding, last one will do fewer experiments
    last_exec_repeats = args.repeats - (repeats-1)*n 
    save_dir = os.path.join(args.out, config_name) 

    try:
        os.makedirs(save_dir)
    except OSError as e:
        print(e)


    base_port = 11311 
    runners = []
    for i in range(n-1):

        port = base_port + i
        out_dir = os.path.join(save_dir, "runner_{}".format(i))
        s = "export ROS_MASTER_URI=http://localhost:{0}/; python -m {4}.start_single --port={0} --repeats={1} --config={2} --out={3}".format(port, repeats, config, out_dir, __package__, i)
        print("Launching with: " + s)
        proc = subprocess.Popen([s], shell=True)
        runners.append(proc)

    # last one may do fewer repeats
    port = base_port + n - 1
    out_dir = os.path.join(save_dir, "runner_{}".format(n))

    s = "export ROS_MASTER_URI=http://localhost:{0}/; python -m {4}.start_single --port={0} --repeats={1} --config={2} --out={3}".format(port, last_exec_repeats, config, out_dir, __package__, n-1)
    print("Launching with: " + s)
    proc = subprocess.Popen([s], shell=True)
    runners.append(proc)

    wait_for_finish(runners)
    


