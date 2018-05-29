import rosbag
import numpy as np
import json
import os
import glob
import matplotlib.pyplot as plt
import tf_conversions
import scipy.stats as st

from matplotlib.patches import Ellipse

from custom_messages.msg import SimulationDataMsg

import sys
sys.path.append("/home/joshua/Documents/Uni/Year4/dissertation/catkin_ws/src/linefollow_gazebo/scripts/")
# sys.path.append("/home/ubuntu/catkin_ws/src/linefollow_gazebo/scripts/")
import CameraNetwork
import Path 
import RoadModel
import Tracer
import helper as h

import bigfloat as bf


def point_to_numpy(point):
    return np.array([point.x, point.y, point.z])

def vec_to_numpy(vec):
    return point_to_numpy(vec)
def quat_to_numpy(quat):
    return np.array([quat.x, quat.y, quat.z, quat.w])
def quat_to_rpy(quat):
    # note: need numpy quaternion
    return tf_conversions.transformations.euler_from_quaternion(quat)

class BasicEntropyMetric(object):

    def __init__(self):
        pass

    def sum_step_entropies(self, bag, max_steps):
        """ Computes the sum of all differential entropies """

        total_entropy = 0
        for i, msg in enumerate(bag):
            if i >= max_steps:
                break
            msg = msg.message
            ekf_cov = np.array(msg.ekf_odom.pose.covariance).reshape(6,6)[:3, :3]
            # have covariance truncated to x, y, theta only
            
            # diff. entropy = H(xt | z1:t, u1:t)
            differential_entropy = np.log(np.sqrt((2*np.pi*np.e)**(3) * np.linalg.det(ekf_cov)))
            total_entropy += differential_entropy
        
        return total_entropy
        
    def get_final_entropy(self, bag, max_steps):
        """ Computes the entropy of a the last step of the bagfile """
        for i, msg in enumerate(bag):
            if i == max_steps:
                msg = msg.message
                ekf_cov = np.array(msg.ekf_odom.pose.covariance).reshape(6,6)[:3, :3]
                differential_entropy = np.log(np.sqrt((2*np.pi*np.e)**(3) * np.linalg.det(ekf_cov)))
                return differential_entropy
        
        raise Exception("Not enough messages in bag {0} to hit {1} steps".format(bag, max_steps))


class CovarianceTraceMetrics(object):

    def get_summed_trace(self, bag, max_steps):
        total_trace = 0
        for i, msg in enumerate(bag):
            if i >= max_steps:
                break
            msg = msg.message
            ekf_cov = np.array(msg.ekf_odom.pose.covariance).reshape(6,6)[:3, :3]
            total_trace += np.sum(np.diag(ekf_cov))
        return total_trace
        
    def get_final_trace(self, bag, max_steps):
        for i, msg in enumerate(bag):
            if i == max_steps:
                msg = msg.message
                ekf_cov = np.array(msg.ekf_odom.pose.covariance).reshape(6,6)[:3, :3]
                return np.sum(np.diag(ekf_cov))
        raise Exception("Not enough msgs in bag {0} for {1} steps".format(bag, max_steps))


class ConditionalEntropyMetric(object):

    def pdf2(self, mean, stddev, value, dx=0.00000001):
        return st.norm.cdf(value-mean, scale=stddev) - st.norm.cdf(value-mean-dx, scale=stddev)

    def log_motion_model(self, ekf_start, ekf_end, ekf_start_heading, 
                     ekf_end_heading, actual_start, actual_end, 
                     actual_start_heading, actual_end_heading, 
                     alpha=np.array([0.0001, 0.0001, 0.0001, 0.0001]), scaling=1.0, verbose=False):
        """ Implements posterior distribution of taking a single step (motion model)"""
    
        
        if verbose:
            print("\nActual points: {0}, {1}".format(actual_start, actual_end))
            print("Actual start heading: {0}, end heading: {1}".format(actual_start_heading, actual_end_heading))
            print("EKf points: {0}, {1}".format(ekf_start, ekf_end))
            print("EKF start heading: {0}, EKF end heading: {1}".format(ekf_start_heading, ekf_end_heading))
        
        if ekf_start_heading < 0:
            ekf_start_heading += 2*np.pi
        if ekf_end_heading < 0:
            ekf_end_heading += 2*np.pi
        if actual_start_heading < 0:
            actual_start_heading += 2*np.pi
        if actual_end_heading < 0:
            actual_end_heading += 2*np.pi
        
        # compute ground truth deltas
        diff_actual = actual_end - actual_start
        d_trans = np.linalg.norm(diff_actual)
        angle1 = np.arctan2(diff_actual[1], diff_actual[0])
        
        if verbose:
            print("Actual movement angle: {0}".format(angle1))
        if angle1 < 0:
            angle1 += 2*np.pi
        # just shift away from 2pi/0 boundaries when close
        if angle1 > 5 or angle1 < 1.2 or actual_start_heading > 5 or actual_start_heading < 1.2 or actual_end_heading > 5 or actual_end_heading < 1.2:
            angle1 = (angle1 + np.pi) % (2*np.pi) # this is safe since we only use d_rot1 and d_rot2 in abs!!
            actual_start_heading = (actual_start_heading + np.pi) % (2*np.pi)
            actual_end_heading = (actual_end_heading + np.pi) % (2*np.pi)
        d_rot1 = angle1 - actual_start_heading
        d_rot2 = actual_end_heading - angle1
        
        # compute believed deltas
        diff_ekf = ekf_end - ekf_start
        dhat_trans = np.linalg.norm(diff_ekf)
        angle2 = np.arctan2(diff_ekf[1], diff_ekf[0])
        
        if verbose:
            print("EKF movement angle: {0}".format(angle2))
    
            
        if angle2 < 0:
            angle2 += 2*np.pi
    
        # just shift away from 2pi/0 boundaries when close
        if angle2 > 5 or angle2 < 1.2 or ekf_start_heading > 5 or ekf_start_heading < 1.2 or ekf_end_heading > 5 or ekf_end_heading < 1.2:
            angle2 = (angle2 + np.pi) % (2*np.pi)
            ekf_start_heading = (ekf_start_heading + np.pi) % (2*np.pi)
            ekf_end_heading = (ekf_end_heading + np.pi) % (2*np.pi)
    
            
        dhat_rot1 = angle2 - ekf_start_heading
        dhat_rot2 = ekf_end_heading - angle2
        
        # compute probabilities of these occuring
        stddev1 = alpha[0]*np.abs(dhat_rot1) + alpha[1]*dhat_trans
        stddev2 = alpha[2]*dhat_trans + alpha[3]*(np.abs(dhat_rot1) + np.abs(dhat_rot2))
        stddev3 = alpha[0]*np.abs(dhat_rot1) + alpha[1]*dhat_trans
        
        p1 = np.log(self.pdf2(0, stddev1, d_rot1 - dhat_rot1, dx=1e-5))
        p2 = np.log(self.pdf2(0, stddev2, d_trans - dhat_trans, dx=1e-5))
        p3 = np.log(self.pdf2(0, stddev3, d_rot2 - dhat_rot2, dx=1e-5))
        
        value = p1 + p2 + p3
        
        if verbose:
            print("d_rot1: {0}, d_rot2: {1}, d_trans: {2}".format(d_rot1, d_rot2, d_trans))
            print("dhat_rot1: {0}, dhat_rot2: {1}, dhat_trans: {2}".format(dhat_rot1, dhat_rot2, dhat_trans))
            print("Diff rot1: {0}, diff trans: {1}, diff rot2: {2}".format(d_rot1 - dhat_rot1, d_trans-dhat_trans, d_rot2-dhat_rot2))
            print("stddev1: {0}, stddev2: {1}, stddev3: {2}".format(stddev1, stddev2, stddev3))
    
            print("LOG P1: {0}, p2: {1}, P3: {2}, value: {3}".format(p1, p2, p3, value))
        
        return value

    
    def norm_2d_pdf(self, mean, cov, point):
        """ Evaluates 2D Gaussian Density function at a point """
        cov = cov.reshape((2,2))*2
        inv = np.linalg.inv(cov)
        det = np.linalg.det(cov)
        coeff = np.sqrt(4*np.pi*np.pi*det)
        centered = (point - mean)[:2]
        power = -0.5 * np.matmul(np.matmul(centered.T, inv), centered)
        return coeff * np.exp(power)
    
    def sensor_model(self, camera_network, camera_id, camera_position, ekf_pos):
        # General idea:
        # Use camera network, camera ID => camera instance
        # Then check p(measurement | state)
        # use the camera placement to obtain an error bound at that position
        # => use normal distribution to check p(get actual measurement)
        
        placement = camera_network.get_placement_by_id(camera_id)
        
        # get error bounds
        _, (major, minor, angle) = camera_network.get_location_and_error(placement, ekf_pos, ellipse=True)
        major = major/3.0
        minor = minor/3.0
        major_sq, minor_sq, cos_sq, sin_sq = major**2, minor**2, np.cos(angle)**2, np.sin(angle)**2
        varx = major_sq * cos_sq + minor_sq*sin_sq
        vary = major_sq*sin_sq + minor_sq*cos_sq
        covxy = (major_sq - minor_sq)*cos_sq * sin_sq
        cov = np.array([[varx, covxy], [covxy, vary]])
        
        return self.norm_2d_pdf(ekf_pos, cov, camera_position)
        
    
    def get_normalizer(self, camera_network, camera_estimate, camera_id, ekf_pos, ekf_cov, n_samples=1500):
        placement = camera_network.get_placement_by_id(camera_id)
        # Need to compute: sum( p(zt | xt) * p(xt | u1:t, z1:t-1))
        # basically going to do via a mini monte-carlo
        # where samples are drawn from the distribution of x_t as given by the EKF!
        total = 0
        
        # pretty sure this would be VERY close if just used covariance of the ekf_pos
        # then sample around it, and generate probabilities with that one covariance
        # rather than sample, get covariance, get probability, repeat
        # but ok
        placement = camera_network.get_placement_by_id(camera_id)
        
        for _ in range(n_samples):
            # sample 2D normal according to N(ekf_pos, ekf_cov)
            sample_position = np.random.multivariate_normal(ekf_pos, ekf_cov)
            
            # get its covariance for position estimation
            pos = np.array([sample_position[0], sample_position[1], 0.0])
            _, (major, minor, angle) = camera_network.get_location_and_error(placement, pos, ellipse=True)
            major = major/3.0
            minor = minor/3.0
            major_sq, minor_sq, cos_sq, sin_sq = major**2, minor**2, np.cos(angle)**2, np.sin(angle)**2
            varx = major_sq * cos_sq + minor_sq*sin_sq
            vary = major_sq*sin_sq + minor_sq*cos_sq
            covxy = (major_sq - minor_sq)*cos_sq * sin_sq
            sample_covariance = np.array([[varx, covxy], [covxy, vary]])
            
            total += self.norm_2d_pdf(sample_position, sample_covariance, camera_estimate)
        return total
    
    
    def calculate_log_probability(self, bag, camera_network, max_steps, verbose=False):
        prior_prob = 1.0
        log_motion_model_prob = np.log(prior_prob)
        prob_movements = []
        n_skipped = 0
        timesteps = 0
        # quite important we discount ones without enough steps
        # because each step adds like 10^-3 to the probability...
        if bag.get_message_count() < max_steps:
            raise Exception("Bag does not have enough messages")

        for i, msg in enumerate(bag):
            if i >= max_steps:
                break
            timesteps += 1
            msg = msg.message
            true_odom = msg.true_odom
            ekf_odom = msg.ekf_odom
            true_pos = point_to_numpy(true_odom.pose.pose.position)
    
            ekf_pos = point_to_numpy(ekf_odom.pose.pose.position)
            true_rpy = quat_to_rpy(quat_to_numpy(true_odom.pose.pose.orientation))
            true_heading = true_rpy[2]
            ekf_rpy = quat_to_rpy(quat_to_numpy(ekf_odom.pose.pose.orientation))
            ekf_heading = ekf_rpy[2]
            ekf_vel = point_to_numpy(ekf_odom.twist.twist.linear)
            ekf_time = ekf_odom.header.stamp.to_sec()
            ekf_positional_cov = np.array(ekf_odom.pose.covariance).reshape(6,6)[:2, :2]
    
            last_ekf_pos = np.array(list(msg.last_ekf_state[:2]) + [0.0])
            last_ekf_heading = msg.last_ekf_state[2]
            last_true_pos = point_to_numpy(msg.last_true_pos)
            last_true_heading = msg.last_true_heading
            
            camera_update = msg.camera_update
            if msg.has_camera_update:
                
                # no longer need to correct since sampling normal => add to true pos has replaced direct raytracing :'(
    #             corrected_camera_position = apply_time_correction(camera_update, ekf_vel, ekf_time)
                corrected_camera_position = point_to_numpy(camera_update.position)
                prob_sensor_measurements = self.sensor_model(camera_network, camera_update.source_camera_id, corrected_camera_position, ekf_pos)
    
                eta = self.get_normalizer(camera_network, corrected_camera_position[:2], camera_update.source_camera_id, ekf_pos[:2], ekf_positional_cov, n_samples=1000)
    
            else:
                prob_sensor_measurements = 1 # ie. there were none
                eta = 1
            
            if np.array_equal(last_true_pos, true_pos):
                continue
            
            
            log_prob_movement = self.log_motion_model(last_ekf_pos, ekf_pos, 
                                         last_ekf_heading, ekf_heading, 
                                         last_true_pos, true_pos,
                                         last_true_heading, true_heading, scaling=1, verbose=verbose)
            
            
            
            value = log_prob_movement + bf.log(prob_sensor_measurements/eta)
    #         value = log_prob_movement
        
            if value == 0.0:
                print "*********** 0.0!! ********"
                n_skipped += 1
                continue
            log_motion_model_prob += value
            if verbose:
                print("Log prob xt -> xt+1: {0}".format(log_prob_movement))
                print("prob sensor measurement: {0}".format(prob_sensor_measurements))
                print("eta: {0}".format(eta))
                print("Current log prob: {0}".format(log_motion_model_prob))
            
    #         prob_movements.append(prob_movement)
    #         if motion_model_prob > 1000:
    #             print(motion_model_prob)
    
        return log_motion_model_prob, timesteps, n_skipped
    
    
    def calculate_mean_entropy_of_runs(self, def_file, bag_files, verbose=False, max_steps=120):
        # need def file to set up camera network
        if type(def_file) == str:
            with open(def_file) as f:
                config = json.load(f)   
        else:
            config = def_file
        
        #  ----- create the camera network -----
        
    
        # get error magnitudes
        camera_config = config['cameras']
        errors = camera_config['errors']
        alg_error = errors['alg_error']
        pos_error_stddev = errors['pos_error_stddev']
        orient_error_stddev = errors['orient_error_stddev']
        
        camera_network = CameraNetwork.CameraNetwork(positional_error_stddev=pos_error_stddev,
                                    orient_error_stddev=orient_error_stddev, 
                                    alg_error=alg_error)
    
    
        # get road definition
        road_width = config['road']['width']
        side_offset = config['road']['side_offset']
        segments = config['road']['path']['segments']
        looping = config['road']['path']['loop']
        
        path = Path.Path(loop=looping)
        for segment in segments:
            curvature = segment["curvature"]
            path.add_segment(curvature=segment["curvature"], length=segment["length"])
    
        # don't actually need this, mostly need width and side offset...
        # may be used further later though!
        road = RoadModel.Road(path, width=road_width, side_offset=side_offset)
    
        for segment in path.segments:
            # create the world objects which define boundaries
            if segment.curvature == 0.0:
                continue # skip, only curved boundaries for now
            midpoint_dist = segment.start_time + segment.get_length()/2.0
            midpoint = segment.point_at(midpoint_dist)
            normal = segment.normal_at(midpoint_dist)
    
            road_radius = np.abs(1.0/segment.curvature) 
            cylinder_radius = road_radius - road.halfwidth - road.side_offset 
            cylinder_center = midpoint + normal * road_radius
            cylinder_direction = np.array([0.0, 0.0, 1.0])
            
            camera_network.add_world_object(Tracer.Cylinder(cylinder_center, cylinder_direction, cylinder_radius))
    
        cameras = camera_config['placements']    
        for i, conf in enumerate(cameras):
    
            placement = CameraNetwork.CameraPlacement(i, np.array(conf['position']), 
                         conf['pitch_degrees'], 
                         conf['yaw_degrees'], 
                         model=conf['model'], 
                         resolution=conf['resolution'], 
                         fov=conf['fov'],
                         positional_error_stddev=pos_error_stddev,
                         orientation_error_stddev=orient_error_stddev
                     )
    
            # position in list will be its ID
            # TODO brittle
            camera_network.add_placement(placement)
    
        
        entropies = [] 
        for i, filename in enumerate(bag_files):
            try:
                bag = rosbag.Bag(filename)
                log_probability, timesteps, n_skipped = self.calculate_log_probability(bag, camera_network, verbose=verbose, max_steps=max_steps)
                entropy = -1*bf.exp(log_probability) * (log_probability) # use bigfloat
                entropies.append(entropy)
                print("Log Probability of run {0}: {1}, Entropy: {2} from {3} timesteps skipped {4}".format(i, log_probability, entropy, timesteps, n_skipped))
            except Exception as e:
                print(e)
                print("(skipping)")
                continue
    #         total_entropy += probability
       
        return np.mean(entropies), np.var(entropies), len(entropies)



class MeasurementMetrics(object):

    def get_mean_crosstrack_error(self, bag, max_steps):
        """ Calculates mean crosstrack error in absolute value """
        # NAVIGATION error
        # this can be quite off to start with, so need to view
        # it as a REDUCTION (%?)
        true_errors = []
        for i, msg in enumerate(bag):
            if i == max_steps:
                break
            msg = msg.message
            target_point = vec_to_numpy(msg.path_update.target_point)
            actual_pos = point_to_numpy(msg.true_odom.pose.pose.position)
            # since the exactly distance appears to be very unreliable (timing mismatch during data recording => record path target point from a different timestep as current point) buffering is a bloody pain
            # compute crosstrack distance here         
            normal = vec_to_numpy(msg.path_update.path_normal)
            diff_actual = actual_pos - target_point
            # project onto normal
            true_errors.append(np.abs(np.dot(normal, diff_actual))) # in absolutes
        return np.mean(true_errors), np.var(true_errors)

    def get_mean_belief_error(self, bag, max_steps):
        # compute the mean absolute difference between the belief and 
        # the true location (this should be easy + accurate)
        # LOCALIZATION error

        differences = []
        for i, msg in enumerate(bag):
            if i == max_steps:
                break
            msg = msg.message
            true_pos = point_to_numpy(msg.true_odom.pose.pose.position)
            ekf_pos = point_to_numpy(msg.ekf_odom.pose.pose.position)
            distance = np.linalg.norm(true_pos - ekf_pos)
            differences.append(distance)

        return np.mean(differences), np.var(differences)

            

def get_bagfiles_for(experiment_path):
    """ Provide an experiment .json """
    if not experiment_path[-1].endswith('.json'):
        print("must provide .json file")
    gen_path = 'gen'
    if len(experiment_path) > 0:
        if len(experiment_path) > 1:
            gen_path = os.path.join(gen_path, os.path.join(*experiment_path[:-1]))
        exp_name = experiment_path[-1].split('.')[-2]
        gen_path = os.path.join(gen_path, exp_name)
    if os.path.isdir(gen_path):
        # collect all .bag files!
        bag_files = []
        for path, dirs, files in os.walk(gen_path):
            for f in files:
                if f.endswith('.bag'):
                    bag_file_path = os.path.join(path, f)
                    bag_files.append(bag_file_path)
        return bag_files
    else:
        print("{0} is not generated!".format(gen_path))

def get_all_bagfiles_below(path):
    
    bagfiles = []
    for (path,dirs,files) in os.walk(path):
        for f in files:
            if f.endswith('.bag'):
                bagfiles.append(os.path.join(path, f))

    return bagfiles

def compute_metrics(definition, bagfiles, max_steps, compute_MI=True, verbose=False):
    metrics = {
        "means": {
            "mean total trace": 0,
            "var total trace": 0,
            "mean final trace": 0,
            "var final trace": 0,
            "mean total differential entropy": 0,
            "var total differential entropy": 0,
            "mean final differential entropy": 0,
            "var final differential entropy": 0,
            "mean mean crosstrack error": 0,
            "var crosstrack error": 0,
            "num_bags": 0
        }
    }
    if compute_MI:
        metrics["mean mutual inf"] = {
            "mutual inf": 0,
            "variance": 0,
            "num_bags": 0
        }

    basic_entropy = BasicEntropyMetric()
    trace = CovarianceTraceMetrics()
    mutual_inf = ConditionalEntropyMetric()
    measurements = MeasurementMetrics() 

    total_traces = []
    final_traces = []
    total_diff_entropies = []
    final_diff_entropies = []
    crosstrack_errors = []
    belief_errors = []

    mean_metrics = metrics['means']
    for f in bagfiles:
        try:
            bag = rosbag.Bag(f)
            if bag.get_message_count() < max_steps:
                # skip bags without enough messages...
                continue
            mean_total_trace = trace.get_summed_trace(bag, max_steps)
            final_trace = trace.get_final_trace(bag, max_steps)
            stepwise_entropies = basic_entropy.sum_step_entropies(bag, max_steps)
            final_entropy = basic_entropy.get_final_entropy(bag, max_steps)
            crosstrack_error = measurements.get_mean_crosstrack_error(bag, max_steps)
            belief_error = measurements.get_mean_belief_error(bag, max_steps)
             # save values if none of them failed
            total_traces.append(mean_total_trace)
            final_traces.append(final_trace)
            total_diff_entropies.append(stepwise_entropies)
            final_diff_entropies.append(final_entropy)
            crosstrack_errors.append(crosstrack_error)
            belief_errors.append(belief_error)
        except Exception as e:
            print(e)
            print("==> (Skipping)")
            continue
    
    print("Total traces: ")
    print(total_traces)
    print("Final traces: ")
    print(final_traces)
    mean_metrics["mean total trace"] = np.mean(total_traces)
    mean_metrics["var total trace"] = np.var(total_traces)
    mean_metrics["mean final trace"] = np.mean(final_traces) 
    mean_metrics["var final trace"] = np.var(final_traces)
    mean_metrics["mean total differential entropy"] = np.mean(total_diff_entropies)
    mean_metrics["var total differential entropy"] = np.var(total_diff_entropies) 
    mean_metrics["mean final differential entropy"] = np.mean(final_diff_entropies)
    mean_metrics["var final differential entropy"] = np.var(final_diff_entropies)
    mean_metrics["mean mean crosstrack error"] = np.mean(crosstrack_errors)
    mean_metrics["var mean crosstrack error"] = np.var(crosstrack_errors)
    mean_metrics["mean mean belief error"] = np.mean(belief_errors)
    mean_metrics["var mean belief error"] = np.var(belief_errors)

    mean_metrics['num_bags'] = len(total_traces)
    
    # average each one that is supposed to be a mean
    # for key in mean_metrics:
        # if key.startswith('mean'):
            # mean_metrics[key] /= mean_metrics['num_bags'] 

    # this one does mean internally
    if compute_MI:
        mean_mi, variance_mi, num_bags_used = mutual_inf.calculate_mean_entropy_of_runs(definition, bagfiles, max_steps=max_steps, verbose=False)
        metrics["mean mutual inf"]['mutual inf'] = mean_mi 
        metrics["mean mutual inf"]['variance'] = variance_mi
        metrics['mean mutual inf']['num_bags'] = num_bags_used
    return metrics


