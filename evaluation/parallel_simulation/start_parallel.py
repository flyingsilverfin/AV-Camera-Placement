import time
import multiprocessing
import subprocess
import argparse
import os
import json
from ..LoggingServer import Logger 
from ..SimulationExperimentRunner import Runner 
import bigfloat as bf
import numpy as np

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


def get_all_summary_files(experiment_dir, name='summary.json'):
    # directory structure is
    # experiment dir => runner_x => run_x => .bags
    # summaries are under runner_

    # just use walk to handle it...
    summary_files = []
    for path, subdirs, files in os.walk(experiment_dir):
        for f in files:
            if f == name:
                summary_files.append(os.path.join(path, f))

    return summary_files

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
    


    # aggregate all the summaries computed by the runners
    summary_files = get_all_summary_files(save_dir)
    
    aggregate_metrics = {
        "means": {
            "mean total trace": 0,
            "var total trace": 0,
            "stddev total trace": 0,
            "mean final trace": 0,
            "var final trace": 0,
            "stddev final trace": 0,
            "mean total differential entropy": 0,
            "var total differential entropy": 0,
            "stddev total differential entropy": 0,
            "mean final differential entropy": 0,
            "var final differential entropy": 0,
            "stddev final differential entropy": 0,
            "mean mean crosstrack error": 0,
            "var crosstrack error": 0,
            "stddev crosstrack error": 0,
            "num_bags": 0
        },
        "mean mutual inf": {
            "mutual inf": 0,
            "variance": 0,
            "stddev": 0,
            "num_bags": 0
        }
    }

    agg_means = aggregate_metrics['means']
    agg_MI = aggregate_metrics['mean mutual inf']

    for summary in summary_files:
        with open(summary) as f:
            metrics = json.load(f)
        
        means = metrics['means']
        # multiply partial means and variances by number of bags
        num_bags = means['num_bags']
        for key in agg_means:
            if key == 'num_bags':
                agg_means[key] += num_bags
            elif not key.startswith('stddev'):
                agg_means[key] += num_bags * means[key] # stddevs will be * 0 so ignore


        # also expand mutual information same way
        mi = metrics['mean mutual inf']
        mi_num_bags = mi['num_bags']
        # add how many bags were comptued over
        agg_MI['num_bags'] += mi_num_bags 
        agg_MI['mutual inf'] += mi_num_bags * bf.BigFloat(mi['mutual inf'])
        agg_MI['variance'] += mi_num_bags * bf.BigFloat(mi['variance']) # since variance is just mean of squared diffs
        
    # re-average to compute overall means
    total_num_bags_means = agg_means['num_bags']
    for key in agg_means:
        if key == 'num_bags':
            continue
        agg_means[key] = agg_means[key]/total_num_bags_means
        if key.startswith('var'):
            # also insert a stddev key, should already exist or python complains
            stddev_key = 'stddev' + key[3:]
            agg_means[stddev_key] = agg_means[key]**0.5

    # re-average MI
    agg_MI['mutual inf'] /= agg_MI['num_bags']
    agg_MI['variance'] /= agg_MI['num_bags']
    agg_MI['stddev'] = agg_MI['variance']**0.5
    agg_MI['mutual inf'] = str(agg_MI['mutual inf']) # make serializable
    agg_MI['variance'] = str(agg_MI['variance'])
    agg_MI['stddev'] = str(agg_MI['stddev'])

    with open(os.path.join(save_dir, "metrics_summary.json"), 'w') as f:
        json.dump(aggregate_metrics, f)





