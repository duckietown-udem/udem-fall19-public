In this hardware exercise the objectives are that you 

* Load the OS onto your Duckiebot and boot it successfully

* Are successfully able to make it move to ensure that everything is working properly

* Are able to calibrate you Duckiebot

* Are able to reproduce the basic lane following demo in the lab

* Are able to take a log from your Duckiebot performing the lane following demo


# Step 1 - Get accounted with your Duckiebot

Your professor will ask you to **name your Duckiebot** and provide you with two labels with the name of your robot. This is a big decision. 

Once you have received your Duckiebot and labeled it and the box, it is time to flash the SD card. If and only if you are running Ubuntu (Linux?) you should be able to do this for yourself. Otherwise you will need help. 

To flash the SD card will require that you have installed the [duckietown shell](https://github.com/duckietown/duckietown-shell). 

With the shell run the following command:

    $ dts init_sd_card --hostname ![YOUR_ROBOT_NAME] --wifi DaffyTown:iuu4EyjwRArSBYgQeGHw5AwihJis2T2Nv5sURFW --configuration daffy
    
Note: if you plan to take your robot home with you sometimes, you may want to also add the wifi credentials of your home network. You can do so by following the [syntax here](http://docs.duckietown.org/DT19/opmanual_duckiebot/out/setup_duckiebot.html). If you don't know your wifi credentials off by heart you can always set them later by ssh'ing into your robot edit the file `etc/wpa_supplicant/wpa_supplicant.conf` to add a new block with your wifi credentials. It's pretty straightforward, just leave the DaffyTown block there so that the robot will always be able to connect when you come to the lab. 

This will take some time because you will need to pull some docker images which will get loaded on the Duckiebot. 

# Step 2 - Boot your Duckiebot

You need to insert your SD card into the Raspberry Pi on your robot and then connect the battery. You should see TODO

On first boot, the robot takes a relatively long time to initialize. It is extracting and loading the Docker images that you loaded on the SD card. 

You know it's done when TODO

# Step 3 - Make your Duckiebot move

Make sure your robot moves with [these instructions](http://docs.duckietown.org/DT19/opmanual_duckiebot/out/rc_control.html).

# Step 4 - View the camera output

View the camera feed from your camera with [these instructions](http://docs.duckietown.org/DT19/opmanual_duckiebot/out/read_camera_data.html)

# Step 5 - Calibrate your camera

Every camera is a little bit different and every camera is mounted a little bit differently. For this reason we need to calibrate what's called the "intrinsic" and "extrinsic" parameters of your camera. Follow [these instructions](http://docs.duckietown.org/DT19/opmanual_duckiebot/out/camera_calib.html)

# Step 6 - Calibrate your motors

Similarly, every motor is a little bit different. We want it to be the case that when you apply the same signal to each motor, the robot goes straight forward at a roughly predicable speed. To calibrate the trim and gain of your motors follow [these instructions](http://docs.duckietown.org/DT19/opmanual_duckiebot/out/wheel_calibration.html)

# Step 7 - Run the lane following demo

To see that all the previous steps are working correctly, try to get your robot to follow a lane in Duckietown. Don't worry for now what happens when it get's to intersections, we'll worry about that later. Mostly we will follow [these instructions](http://docs.duckietown.org/DT19/opmanual_duckiebot/out/demo_lane_following.html) except for the first `dts` command please do the following:

    dts duckiebot demo --demo_name lane_following --duckiebot_name ![DUCKIEBOT_NAME] --package_name duckietown_demos --image duckietown/dt-core:daffy
    
i.e., add the extra flag `--image duckietown/dt-core:daffy` at the end. 

Please take a short video of your robot doing lane following in the lab with your cell phone. 

# Step 8 - Record a log

Follow [these instructions](http://docs.duckietown.org/DT19/opmanual_duckiebot/out/take_a_log.html) to take a log. Don't do the "Full logging" option just use `make_log_docker`


