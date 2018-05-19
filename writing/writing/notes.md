#




# Camera Models

http://av.jpn.support.panasonic.com/support/global/cs/dsc/knowhow/knowhow12.html

Realistically, cameras don't support more than 'ultra-wide angle' which corresponds to about 65-85 degree FoV, which isn't even that much! 
* https://en.wikipedia.org/wiki/Wide-angle_lens

* discuss & show results of the two camera models look at extremes of say 20 deg - 85 deg comparing the two models
* also have variation based on height, pitch angle

* Discuss & reference that distortion correction for fish eye lenses normally information at the corners
eg. here's one https://pdfs.semanticscholar.org/7dab/5e558d726778feeb5c2c32dc0570ac554893.pdf (niche - get a different one!)


* look into Tissot's indicatrix

## Pixels taken up by vehicle
* model standard vehicle conservatively as 5m x 2m x 2m (taken from a random sample of vehicles on https://www.automobiledimension.com/)



## Camera mounting
https://www.cctvaware.com/blog/correct-installation-height
Homes - 2.5 ish meters high

TLF: https://consultations.tfl.gov.uk/policy/streetscape-guidance/user_uploads/part-d---section-eight---physical-design-and-materials---safety-and-functionality.pdf
Pole mounted traffic cameras - 6, 8, 10, 12 17 meters

* Good tutorial on IMUs, cooridnate frames etc.
https://arxiv.org/pdf/1704.06053.pdf
* Also has an error analysis for orientation estimation!
  * mention heading (ie. yaw) can have high error, but pitch and roll within 0.5 degrees 
  * mention yaw can be calibrated easily during setup but more difficult to correct due to sudden knocks/changes in heading...

## Vehicle models

### Sensor noise
* One paper "Low cost approach for accurate vehicle localization and Autonomous driving application"
  * Implement dead reckoning using MEMS IMU for rotation & wheel encoders for linear motion
  * Achieve 2.4m mean/ 5.3m error over 1850m ... very accurate!
  * 
* Masters thesis
  * Braking limits: 5 m/s^2 (passenger comfort limit)
  * lateral acceleration limits |a_y| < 2 m/s^2 => v_x < a_y * R (R = radius of curve) *** USE THIS FOR SPEED LIMITS ON CURVES *** 
  * 

* https://hackernoon.com/ghost-iv-sensor-fusion-encoders-imu-c099dd40a7b
* DEAD RECKONING NAVIGATION FOR AUTONOMOUS MOBILE ROBOTS 

* Odometry models
http://ais.informatik.uni-freiburg.de/teaching/ss11/robotics/slides/06-motion-models.pdf
https://robotics.stackexchange.com/questions/964/extended-kalman-filter-using-odometry-motion-model

## EKF
ros EKF package is supposedly only Unicycle model... ackerman might not work
https://answers.ros.org/question/221837/robot_localization-ekf-internal-motion-model/

May try robot_localization
https://vimeo.com/142624091

* note: the standard implementation uses a unicycle model...
https://answers.ros.org/question/221837/robot_localization-ekf-internal-motion-model/

