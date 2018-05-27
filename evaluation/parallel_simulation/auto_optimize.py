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


def count_bagfiles_below(directory):
    count = 0
    for (_, _, files) in os.walk(directory):
        for f in files:
            if f == 'sim_data.bag':
                count += 1
    return count

def num_already_completed(exp_dir, optimization_metric):
    if not os.path.exists(exp_dir):
        return 0, None
    contents = os.listdir(exp_dir)
    if 'metrics_summary.json' in contents:
        with open(os.path.join(exp_dir, "metrics_summary.json")) as f:
            metrics = json.load(f)
        if optimization_metric == 'mutual information':
            num_completed = metrics['mean mutual inf']['num_bags']
        else:
            num_completed = metrics['means']['num_bags']
        print("metrics_summary.json indicates already completed {0} runs".format(num_completed))
        return num_completed, metrics
    else:
        return 0, None

def generate_config(auto_opt_config, camera_placements):
    # outline:
    # make a copy of the auto opt config
    # strip out the 'cameras/placements_groups' bit
    # add in 'cameras/placements' as a list
    # append each of the camera placements into it

    #copy
    copy = json.loads(json.dumps(auto_opt_config))
    
    # delete specific key
    del copy['cameras']['placement_groups']
    copy['cameras']['placements'] = []
    for placement in camera_placements:
        copy['cameras']['placements'].append(placement)

    return copy


def do_execution(config, current_placements, exp_name, gen_dir, nparallel, nrepeats, optimization_metric):

    this_exp_dir = os.path.join(gen_dir, exp_name)
    already_completed, metrics_summary = num_already_completed(this_exp_dir, optimization_metric)

    # make the subdir for this experiment
    try:
        os.makedirs(this_exp_dir)
    except Exception:
        # already made
        pass

    # decrease by number indicated in metrics
    # or number of bagfiles found
    if metrics_summary is None:
        num_bagfiles = count_bagfiles_below(this_exp_dir)
        if num_bagfiles > 0:
            nrepeats -= num_bagfiles
    else:
        nrepeats -= already_completed

    if already_completed >= nrepeats and metrics_summary is not None:
        print("\nAlready have enough information on {0}!!!\n".format(this_exp_dir))
        return metrics_summary # already done this experiment!

    # force regeneneration of metrics_summary if we already have enough
    # by generating 1 more
    nrepeats = max(nparallel, nrepeats)





    # modify auto-opt config and generate one for this experiment
    # write experiment definition to a file
    this_experiment_config = generate_config(config, current_placements)
    exp_config_path = os.path.join(this_exp_dir, "{0}.json".format(exp_name))
    with open(exp_config_path, 'w') as f:
        json.dump(this_experiment_config, f, indent=4, sort_keys=True)


    # launch run_parallel.py

    # launch parallel execution script
    # wait until finished
    # read in the metrics_summary.json
    # return that


    call_string = "python -m {4}.start_parallel --config={0} --nparallel={1} --repeats={2} --out={3} --no-rviz --continue".format(exp_config_path, nparallel, nrepeats, gen_dir, __package__) # start_parallel generates own subdir from config name!
    runner = subprocess.Popen([call_string], shell=True)



    while True:
        time.sleep(0.5)
        if runner.poll() is not None:
            break 

    # need to return produced metrics!

    with open(os.path.join(this_exp_dir, "metrics_summary.json")) as f:
        metrics = json.load(f)
    return metrics


def get_score_for_metric(metric, metrics_summary):
    mean_metrics = metrics_summary['means']
    if metric == 'mutual information':
        mi_metrics = metrics_summary['mean mutual inf']
        mean_mi = mi_metrics['mutual inf'] # really conditional entropy
        return bf.BigFloat(mean_mi) # make sure we actually use bigfloat

    elif metric == 'total trace':
        return mean_metrics['mean total trace']
    elif metric == 'final trace':
        return mean_metrics['mean final trace']
    elif metric == 'final differential entropy':
        return mean_metrics['mean final differential entropy']

    else:
        raise Exception("Unknown metric: {0}".format(metric))


def optimize(config, gen_dir):
    """ Greedily optimize a given metric """

    opt_params = config['analysis']
    budget = opt_params['budget']
    nrepeats = opt_params['nrepeats']
    nparallel = opt_params['nparallel']
    opt_criterion = opt_params["optimize"]

    # this is all expensive so probably want to
    # save some progress if we can
    log = []
    logfile = os.path.join(gen_dir, "optimization_log_{0}.txt".format(opt_criterion))

    possible_placement_blocks = config['cameras']['placement_groups']
   
    remaining_blocks = list(range(len(possible_placement_blocks))) # will remove indices
    current_placements = []
    current_name = "conf_" 

    for iteration in range(budget):
        current_scores = [] # reset scores on each iteration
        for blocknum in remaining_blocks:
            block = possible_placement_blocks[blocknum]
            current_scores.append([]) # a new holder for scores

            for orient_num, placement in enumerate(block):
                current_placements.append(placement)
                name = current_name + "_block_{0}_orient_{1}".format(blocknum, orient_num)

                print("Testing placements: {0}".format(current_placements))

                # run optimization, save score
                metrics = do_execution(config, current_placements, name, gen_dir, nparallel, nrepeats, opt_criterion)

                score = get_score_for_metric(opt_criterion, metrics)
                current_scores[-1].append(score)
    
                current_placements.pop(-1) # delete it again
      
        print(current_scores)
        # find the best placement
        best_block_num, best_orient_num = -1, -1
        best_score = 9999999999 # we always minimize
        for i, b_num in enumerate(remaining_blocks): # iterate over remaining blocks
            block_scores = current_scores[i]
            for orient_num, score in enumerate(block_scores):
                if score < best_score:
                    best_score = score
                    best_block_num = b_num
                    best_orient_num = orient_num
       
        # choose the best placement and append it to the chosen ones
        best_placement = possible_placement_blocks[best_block_num][best_orient_num]
        current_placements.append(best_placement)
        current_name += "__block_{0}_orient{1}".format(best_block_num, best_orient_num)

        #find index of the chosen block and remove it, no longer allowed
        index = remaining_blocks.index(best_block_num)
        remaining_blocks.pop(index) # remove it

        log.append("All scores from this iteration {0} using {1} metric: {2}".format(iteration, opt_criterion, current_scores)) 
        log.append("Chose placement block {0}, index {1}: {2}".format(best_block_num, best_orient_num, best_placement))

        # overwrite each iteration
        with open(logfile, 'w') as f:
            for line in log:
                f.write(line)
                f.write('\n')
        




if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Launch auto-optimization, with a specified config file')

    parser.add_argument("--config", required=True, help="Config file that defines experiment to greedily optimize")
    parser.add_argument("--out", required=True, help="Root Directory to save intermediate simulation results/data to")


    args = parser.parse_args()

    auto_opt_config = args.config
    config_name = auto_opt_config.split('/')[-1].split('.')[0]

    with open(auto_opt_config) as f:
        blob = json.load(f)

    save_dir = os.path.join(args.out, config_name) 

    optimize(blob, save_dir) 





