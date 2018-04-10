# based on 
# https://github.com/ivmech/ivPID/blob/master/PID.py
# and
# ROS PID package implementation


class PID(object):

    def __init__(self, kp=0.2, ki=0.0, kd=0.0, setpoint=0.0, sample_time=0.0, lower_limit=-1.0, upper_limit=1.0, windup=10.0):
        # PID gain constants
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # setpoint for PID
        self.setpoint = 0.0

        # used to limit update rate of PID
        # if 0 then update one each new sample regardless
        self.sample_time = sample_time

        # effort clips and I term limits
        self.effort_limit_low = lower_limit
        self.effort_limit_high = upper_limit
        self.windup = windup

        # state
        self.last_time = None
        self.I = 0.0
        self.last_error = 0.0


    def clear(self):
        self.last_time = None
        self.I = 0.0
        self.last_error = 0.0

    def update(self, time, state):
       
        error = self.setpoint - state
        
        control = 0.0
            
        # contribution by P term
        control += self.kp * error 

        # only add I and D terms if not on first iteration
        if self.last_time is not None:

            dt = time - self.last_time
            d_error = error - self.last_error

            # I term
            self.I += error * dt
            # check I guards
            if self.I < -self.windup:
                self.I = -self.windup
            elif self.I > self.windup:
                self.I = self.windup

            control += self.di * self.I

            # D term
            if dt > 0.0:    # just in case of strange divide by 0 cases
                d = d_error / dt
                control += self.kd * d
       
        # apply control limits
        if control < self.effort_limit_low:
            control = self.effort_limit_low
        elif control > self.effort_limit_high:
            control = self.effort_limit_high
        
        # return control value
        return control


        


