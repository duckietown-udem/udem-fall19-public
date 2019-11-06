  # Software Exercise 7 - State Estimation
  
 ## Introduction
  From their sensors, robots gather a significant quantity of measurements that contain information about the state of the robot and its environment.
  The state is a representation of the world at time _t_ which is sufficient to allow the robot to take decisions in order to achieve its objective. 
  For example, in the lane following pipeline of the Duckiebot, the objective is to follow the direction of the road while staying in the middle of the right lane. 
  A sufficient representation of the world can then be the robot's angle _phi_ from the road direction and its distance _d_ from the middle of the right lane - this will be sent to a controller, as we have seen at the beginning of the class.
  
  To estimate the state from the several - possibly noisy - possibly indirect - observations, most systems use a *filter*.
  A very popular type of filters are the Bayesian filters: they evaluate a belief, which is a probability distribution over the state.
  
  ### Reminder
  In every filter, the belief is updated in two steps: 
  1. The predicting step propagates the dynamics of the system using a dynamic model. If _x(t)_ is the state at time _t_, _z(t)_ is the set of measurements, and _u(t)_ the control command we issued, the prediction step allows us to get from _p(x(t-1)|z(t-1, t-2, ..., 0), u(t-1, t-2, ..., 0))_ to _p(x(t)|z(t-1, t-2, ..., 0), u(t, t-1, t-2, ..., 0))_. Notice that the main difference is that _x(t-1)_ became _x(t)_ and we incorporated the _u(t)_ information.
  2. The measurement updates the prior belief by including the information obtained from the measurements. The prior belief is _p(x(t)|z(t-1, t-2, ..., 0), u(t, t-1, t-2, ..., 0))_. The likelihood is _p(z(t)|x(t))_. We add the _z(t)_ information and get the posterior _p(x(t)|z(t, t-1, t-2, ..., 0), u(t, t-1, t-2, ..., 0))_ using Bayes' law:  _posterior = n*(prior*likelihood)_, where _n_ is a normalization constant.
  
  You have seen several filters in class, among them: Kalman filters, histogram filters and particle filters. They mainly differ in their representation of the belief distribution.
   
  In this exercise, you will understand the histogram filter used in the current implementation and you will replace it with your own particle filter.
  
  ## Set up
  
  ### Getting the files
  
  Pull the additional files in `udem-fall19-public`:
  ```
  $ git pull upstream master 
  $ git submodule init
  $ git submodule update
  $ git submodule foreach "(git checkout daffy; git pull)"
  ```
  
  Checkout to the right commit for simulation:
  
  ```
  cd simulation
  git checkout 4693556
  ```
   
  You will find in `catkin_ws/src` a folder named `sw07-SE`. There, in the `packages` repo, you will find the ROS package `sw_07_lane_filter`. This is where you will work.
  
  ### Running the simulation
  To run the simulation, from `udem-fall19-public` directory, run:
  ```
  $ docker-compose build 
  $ docker-compose up
  ```
  
  You will need to build the new package. In the notebook terminal, run:
  ```
  $ catkin build --workspace catkin_ws
  $ source catkin_ws/devel/setup.bash
  ```
  Manually start the `car_interface`:
  ```
  $ ./launch_car_interface.sh
  ```  

  And launch the lane following demo using your version of the line detector:
  ```
  $ roslaunch sw_07_lane_filter lane_following.launch
  ```
  
  ### Visualizing
  Remember that you can visualize using noVNC (follow the Visualization and Debugging instructions given in [Hardware Exercise 2](https://github.com/duckietown-udem/udem-fall19-public/blob/master/notebooks/hw-02-pure-pursuit-control.md)).
  
  **Note**: the noVNC password is `quackquack`.
  
  If you want to see the output of the line detector:
  ```
  $ rosparam set /default/line_detector_node/verbose true
  ```
  
 
  ## The node
  Take a look at the file `src/lane_filter_node.py`. 
  2 topic subscribers manage the direct inputs to the filter, and 4 publishers manage the outputs (including visualization).
  
  **Question 1** List these topics and describe shortly the information they transmit. Write your answer in a `07-state-estimation.txt` file.
  
  One of the parameters of the node is the mode: `self.mode` can be `histogram` or `particle`. This is yours to change in the code, and it should allow you to switch between the original histogram filter and the particle filter you will later implement. The object `self.filter` is chosen accordingly. The classes `LaneFilterHistogram` and `LaneFilterParticle` are defined in `include/sw_07_lane_filter/lane_filter.py`.
  
  The main filtering process happens in the callback function `processSegments(segment_list_msg)`. 
  
  There are 4 important calls made to `self.filter`:
  1. `self.filter.predict(dt=dt, v=v, w=w)` is the prediction step - it propagates the dynamic model of the system on the belief.
  2. `self.filter.update(segment_list_msg.segments)` is the measurement update step - it corrects the belief based on the sensor measurements.
  3. `self.filter.getEstimate()` is a function that outputs the filter's estimate as a single pair of values, _phi_ and _d_. This pair is chosen as the one with the maximal belief: it is "maximum a posteriori" estimation.
  4. `self.filter.isInLane()` allows to detect if the Duckiebot is in the lane or completely out. To do so, it checks if the measurements have led the belief to converge to one particular _phi_ and _d_ pair, which means that they are coherent with the expected measurement while in the lane. Otherwise, if the measurements do not seem to converge, it means that the Duckiebot is out of a lane.
   
  
   
  ## The histogram filter
  Now, let's get a look at the histogram filter - which is already implemented in class `LaneFilterHistogram`. In this section, there are a lot of questions. Short answers (one or two sentences) are enough. Write your answers in the `07-state-estimation.txt` file.
  
  **Question 2**
  How is the belief represented in the histogram filter? How is it initialized?

  **Question 3** Look at the measurement `update` function.   Note that from a single segment, one can geometrically determine a corresponding (_d_, _phi_) pair through simple geometric considerations. This is done in the `generateVote()` function. Knowing this, describe shortly how is the likelihood _p(z(t)|x(t))_ computed.
   
  ## The particle filter
  In this part of the assignment, you will have to implement a particle fitler to the robot. The template of the filter is already there, at the beginning of the `lane_filter.py` file. To switch it on, change `self.mode` to `'particle'` in `LaneFilterNode` (lines 21-22). While it does not do anything, the current version should be running without errors.
  
  ### What is a particle filter?
  A particle filter, also called a Monte Carlo filter, is a Bayesian filter that represents the belief with a set a particles. Each particle represents a possible state in which the robot can be. The particle filter algorithm works as follow:
  1. Initialize your particle set by sampling a given initial distribution.
  2. At each time step (here: everytime you receive a measurement):
     1. Update each particle using a dynamic model, maybe with some noise.
     2. Compute the likelihood of the measurements for each particle. Assign this likelihood as a weight for the particles.
     3. Renew your particle set by sampling through the current particles, weighted by their respective weights.
     4. To avoid particle depletion, roughen the particle set by adding some noise.
    
  ### Implementation
  
  #### General information
  You are asked to complete the template in `LaneFilterParticle`. Here is the last bit of general information before you can start.
    
  ##### Parameters
  The initialization, prediction step noise and roughening parameters in `__init__` have been chosen arbitrarily. You may have to tune them to ensure that your filter works well.
  
  ##### Particle class
  Each instance of this class is an individual particle of the set. It has 3 main attributes: _d_, _phi_, and _weight_. Its `predict` and `update` functions are called in the corresponding `predict` and `update` functions of the filter.
  
  ##### Visualization
  The `getBeliefArray(self)` function puts the particles into bins and basically produces an image that is similar to the one produced by the histogram filter. It allows you to see clearly the belief in `rqt_image_view` on the `~belief_img` topic.

  
  #### Particular instructions
  Here is some information that can be useful for your implementation.
  ##### `Particle.predict(self, dt, v, w)`
  When predicting the new _d_, you will need to take into account the angle _phi_.
  
  ##### `Particle.update(self, ds, phis)`
  How can you estimate the likelihood of the measurements given this particular particle? Here, the _d_ and _phi_ values for a given segment can be recovered using function `self.process(segment)`. _Suggestion: remember how it was done in the histogram filter. Maybe you can compute a distance from your particle to each measured pair of (d,phi) and compute a score based on the quantity of pairs that is not further than a given threshold? Other ideas are welcome too!_
  
  ##### `lane_filter.initialize(self)`
  Initialize the particle set using a given distribution. You can use the initialization parameters. Would sampling from a Gaussian distribution be a good idea? Could you also want to sample from an uniform distribution, in order to be able to recover from an initial state that is far from the Gaussian center?
  
  ##### `lane_filter.resample(self)`
  Generate a new set of particles by sampling from the weighted previous set.
  
  ##### `lane_filter.getEstimate(self)`
  What is the best way to give an estimate of the state given a set of particles? Would it be a random sampling? An average? Putting them in bins and chosing the most populated one?
  
  ##### `lane_filter.isInLane(self)`
  Remember the way the histogram filter was determining if the robot is or is not in the lane. What was the idea behind it? How could this be applied to a particle filter? 
    
 Complete these 6 functions in `lane_filter.py`. Try it on the simulation. Don't forget that you can see the belief in `rqt_image_view` on the `~belief_img` topic.
 
 **Question 4** In `07-state-estimation.txt` file, describe your implementation for each of the 6 functions.
 
 **Question 5** Does it work? If not, describe and explain what is happening?
 
 **Question 6** How is the particle filter able to deal with wrong line detections, due to Duckies in the image for example?
 
 **Question 7** Would a particle filter be able to recover from a wrong estimation, or from an unexpected initial state? Which parameters have an influence here? What is the counterbalance of making the particle filter robust to such a possibility?
 
 **Question 8** Explain the influence of the number of particles. What is the limiting factor?
  
 **Question 9** Compare the particle filter to the histogram filter. What are the pros and cons of each? Could a Kalman filter work here?
 

 
 **BONUS**
 Run the launch file on a real robot. Does it work? Describe. If there is a difference with the simulation: why is it so? What can you do to make it more robust? 
