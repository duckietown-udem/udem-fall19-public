# Hardware Exercise 2: Pure Pursuit Control

We've made some changes to the repo, so we're going to do some local updates first. 


## Update your repo

Start by pushing all your local changes to be safe. 

If you are using a fork, then you are going to need to pull from the upstream remote. To do this first create the upstream:

    $ git remote add upstream https://github.com/duckietown-udem/udem-fall19-public.git

then pull from the upstream

    $ git pull upstream master 

and update your submodules to the `daffy` branch

    $ git submodule init
    $ git submodule update
    $ git submodule foreach "(git checkout daffy; git pull)"


## Test to see if things are working - Run the lane following demo in the simulator

from the `udem-fall19-public` directory run:

    $ docker-compose build
    $ docker-compose up
    
You should see now that three containers are starting

```
Starting udem-fall19_sim_1   ... done
Starting udem-fall19_novnc_1        ... done
Recreating udem-fall19_lanefollow_1 ... done
```

Note that the simulator now runs in its own container. 

You can open the notebook just like before by copying the url that looks like:

    http://127.0.0.1:8888/?token={SOME_LONG_TOKEN}

into your browser and open a terminal. 

You no longer need to build the `custom_ws`, but rather only the `catkin_ws`:

    $ catkin build --workspace catkin_ws
    $ source catkin_ws/devel/setup.bash
    
If this is the initial build, it will take around five minutes; Due to the `docker` mounts, the build artifacts will persist, so subsequent builds will be much faster.

You also need to manually start the `car_interface`, which runs the inverse kinematics.

You can do this with:

    $ ./launch_car_interface.sh

Now you can run any demo in the Duckietown codebase. Let's test out one that you have experience with already:

    $ roslaunch duckietown_demos lane_following.launch

which is the launch file that was run when you previously executed `dts duckiebot demo --package_name duckietown_demos --demo_name lane_following` in the last hardware exercise (This is exactly what the  `package_name` and `demo_name` flags do when you run `dts duckiebot demo`).

You should see the nodes launch. 

### Visualization and Debugging

We've recently changed the workflow for viewing the output to avoid having to use X. Now, we are going to use `noVNC`. In your browser, enter the url `http://localhost:6901/vnc.html`, which will bring you to a login page. The password is `quackquack`. 

Once inside, you have a standard GUI environment. From here, you can launch windows inside the browser. Under "Applications" in the top left you can open a terminal. In the terminal, you can run `rqt_image_view` just like before and choose an output to view. 

**NOTE if you want to see some outputs (e.g., the `image_with_lines`) then you need to set the verbose flag to true by typing:** 

    $ rosparam set /default/line_detector_node/verbose true


You might also run `rviz` in the terminal and play around with the debugging outputs that you can add in the viewer. To do so click the "Add" button in the bottom left and then the `By Topic` tab. You might find the `/duckiebot_visualizer/segment_list_markers/` and the filterer version as well as the `/lane_pose_visualizer_node/lane_pose_markers` particularly interesting.

### Making the Robot Move in Simulator

At this point, the virtual robot is not moving. Why?

To remedy the situation you will need to use the virtual joystick just like you did with the real robot. Run on your laptop:

    $ dts duckiebot keyboard_control default --network <network_name>  --sim [--cli] --base_image duckietown/dt-core:daffy
    
where you can find the `network_name` by running `docker network ls` and look for the right one. Likely, the network looks like `udem-fall19-public_duckietown-docker-net`. The `--cli` is optional, but if you are running on Mac, then the GUI is not supported. 

Now, you should be able to use the keyboard to make the Duckiebot move and also have it starting doing autonomous lane following. 

We have now completely reproduced the lane following demo but in the simulator. 


## Create a repo for your packages the template repo

Now we are going to discuss how you might build a new node and incorporate it into the structure of an existing demo. You will be able to test the performance first in the simulator and then try it on the robot. 

Follow the instructions [here](https://github.com/duckietown/template-ros-core) to create a new repo for your packages. Clone this repo inside the folder `udem-fall19-public/catkin_ws/src`. You can now add packages inside the `packages` folder of that repo. 

**Important** put your name in the name of your repo.

Any packages that you put in there will be built when you run:

    $ catkin build --workspace catkin_ws

in the notebook terminal. 

If you have added a new launch file then you can launch it from the notebook terminal with:

    $ roslaunch <your_package_name> <your_launch_file_name>


A good way to build your own launch file would be to use the provided launchfile, `lane_following.launch`, as a template. You can find this file in `duckietown_demos` in `dt-core`. You can "turn off" the `args` that correspond to the new nodes that you don't want to run and then add your own `include`s or `<node>` launching code after. You'll want to create a new launchfile, rather than editing the original `lane_following.launch`. 

## Trying your code on the robot

Once you are happy with the operation of your new package, you can try it on the robot. Go into your repo's base directory (the one you made from the template) and run:

    $ dts devel build --push

This will wrap your code into a docker image that inherits from the `dt-core` repo (so it has all of those packages if you are using them in your launchfile).

Then you can run it on the robot through the same API as the lane following demo:

    $ dts duckiebot demo --demo_name <your-launch-file-name> --package_name <your-package-name> --duckiebot_name <your-duckiebot-name> --image duckietown/<repo-name>:<branch-name>
    
where `<repo-name>` is the name of the repo that you created from the template and `<branch-name>` is the branch that you are working on in your repo. 

when you run it might hang for a bit at the line: ```INFO:dts:Running command roslaunch <your-package-name> <your-launch-file-name>.launch veh:=<your-duckiebot-name>``` since it is pulling the image. You can also manually pull the image before running to see the status. 



## Your Task for this Exercise


In this exercise we are going to replace the existing PD controller with a pure pursuit controller on the robot. 

One issue discussed in class with respect to implementation of the pure pursuit controller is that it requires a reference _trajectory_ rather than just a reference value with which we can compute the tracking error. 

In order to implement the pure pursuit controller, we need to be able to calculate $\alpha$. 

We can calculate an esitimate of $\alpha$ directly from the line detections projected onto the ground plane. 

The output of the ground projection node provides ground plane endpoints (in the robot frame) of the lines that are detected, along with their color. 

The algorithm to follow should be roughly the following:

 1. Filter the line detections to find the white ones and yellow ones that are "close" to your lookahead distance L
 2. Use these line detections to calculate an estimate of \alpha (I suggest to average between the white and yellow ones)
 3. Encapsulate this in a node, wire things up,  and try it on the robot. 

We have provided a function that filters the lane detections and publishes only the inliers. This might be a better choice for this algorithm. It is published by the `lane_filter`. 

This exercise is a bit more difficult to implement than some of the others. It may  require a good amount of debugging, and good performance is not guaranteed. Suggest not to leave to the last minute!

## Deliverables

Inside the report.txt file include:

 - the name of the repo that you created
 - the name of the docker image that you built and pushed
 - a video of your Duckiebot executing lane following 
 - the time it took to complete the course (+ 2 seconds for every tile where there was a lane violation)
 - A plot of the two control values as well as the estimate of two errors (cross-track and angle) from the perception system vs. time
 
 

