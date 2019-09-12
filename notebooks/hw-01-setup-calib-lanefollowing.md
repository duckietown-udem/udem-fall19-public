# Hardware Exercise 1: Setup, Calibration and Lane Following



# Duckietown Hardware Exercise 1: Getting Started


Duckietown is a unique course, as it offers both a software and hardware component. In this exercise, we will be getting started with your Duckiebot, and running some simple commands and demos on it. Specifically, the objectives we'll achieve by the end of the exercise are: 

* Loading the OS onto your Duckiebot and booting it successfully

* Successfully able to move your Duckiebot, ensuring that everything is working properly

* Calibration of your Duckiebot

* Reproduction of the basic lane following demo in the Duckietown lab

* Taking a log from your Duckiebot performing the lane following demo


## Step 1 - Saying Hello to Your New Duckiebot

It's time to **name your Duckiebot**, with the instructor providing you with two robot-name labels. This is a big decision, one not to be taken lightly.

Once you have received your Duckiebot, label it and the accompanying box. From there, we will move to flash the SD card. If and only if you are running Ubuntu (_Note_: These instructions _should_ work on other distributions of Linux, but have not been tested on any distro other than Ubuntu) you should be able to do this for yourself. Otherwise, ask an instructor for access to compatible hardware.

To flash the SD card, you will need to have installed the [duckietown shell](https://github.com/duckietown/duckietown-shell). You will also need to [sign up for an account on duckietown.org](https://www.duckietown.org/site/register), and then set your token in the command line with:

    dts tok set ![YOUR_TOKEN]
    
There are more detailed instructions [in the book](http://docs.duckietown.org/DT19/opmanual_duckiebot/out/dt_account.html). 

To burn the SD card with the shell run the following command:

    dts init_sd_card --hostname ![YOUR_ROBOT_NAME] --wifi DaffyTown:iuu4EyjwRArSBYgQeGHw5AwihJis2T2Nv5sURFW,duckietown:quackquack --configuration daffy
    
**Note**: If you plan to occasionally take your robot home with you, you may want to also add the Wifi credentials of your home network. You can do so by following the [syntax here](http://docs.duckietown.org/DT19/opmanual_duckiebot/out/setup_duckiebot.html). If you don't have the credentials now, you can postpone this step, and later `ssh` into your robot and edit `etc/wpa_supplicant/wpa_supplicant.conf` to add a new block with your Wifi credentials. As long as you leave the Daffytown credentials (i.e append your home network's, don't replace the Daffytown block), you should always be able to connect while you are in the UdeM lab.

These steps will pull Docker images onto your robot, and may take an extended period of time.

## Step 2 - Booting Your Duckiebot

You need to insert your SD card into the Raspberry Pi on your robot and then connect the battery. You may need a pair of pliers to insert the SD card - it goes into the slot right under the top deck of your robot and it's hard to slide it in with your fingers (admittedly not great design). Once you connect the USB power cords to your battery, you should see your LED lights on your Duckiebot (there's 5, 3 in front and 2 in back) go white. 

You should also immediately see the green LED of the Raspberry Pi next to where the SD card was inserted start to blink with activity.

If not, stop, as there is a problem with the SD card initialization (or possibly the Raspberry Pi, but this is unlikely).

After some time, the red and the green LEDs of the Raspberry Pi will start to blink alternately. This means that the necessary Docker containers are being extracted. When the process is finished the red LED will be off and the green will be on.

**Important** do not interrupt power while the Duckiebot is booting up the first time. It will results in an unstable state and you will probably have to start over. 
On first boot, the robot takes a relatively long time to initialize. It is extracting and loading the Docker images that you loaded on the SD card. 

Getting impatient? You can see the progress by ssh'ing (see instructions below) into the robot and doing:

    cat /data/boot-log.txt
    
When all the images are loaded you will see a last message indicating so. 

## Step 3 - Making Your Duckiebot Move

Make sure your robot moves with [these instructions](http://docs.duckietown.org/daffy/opmanual_duckiebot/out/rc_control.html).

## Step 4 - Viewing the Camera Output


View the camera feed from your camera with [these instructions](http://docs.duckietown.org/daffy/opmanual_duckiebot/out/read_camera_data.html).


## Step 5 - Calibrating the Camera

While each Duckiebot is fundamentally identical, small differences in camera mounting and manufacturing require us to _calibrate_ the camera. Traditionally, we'd have to do this for all the sensors on the robot - but since Duckiebots only use a camera for operation, we need only to calibrate the camera. Specifically, we need to [calibrate the "intrinsic" and "extrinsic" parameters](https://www.mathworks.com/help/vision/ug/camera-calibration.html) of your camera. Follow [these instructions](http://docs.duckietown.org/daffy/opmanual_duckiebot/out/camera_calib.html) to do so.

## Step 6 - Calibrating the Motors


Similarly, due to manufacturing, every motor is a little different, and requires calibration. We want it to be the case that when you apply the same signal to each motor, the robot goes straight forward at a roughly predicable speed. To calibrate the trim and gain of your motors, the parameters which map the signal to speed, follow [these instructions](http://docs.duckietown.org/daffy/opmanual_duckiebot/out/wheel_calibration.html).

## Step 7 - Running the Lane Following Demo

To see that all the previous steps are working correctly, we want to make sure your robot can correctly follow a lane in Duckietown (For now, don't worry about what happens when at intersections, which is a later focus of the class). Mostly, we will follow [these instructions](http://docs.duckietown.org/daffy/opmanual_duckiebot/out/demo_lane_following.html).


Please take a short video of your robot doing lane following in the lab with your cell phone. 

## Step 8 - Record a log


Follow [these instructions](http://docs.duckietown.org/daffy/opmanual_duckiebot/out/take_a_log.html) to take a log. Don't do the "Full Logging" option, but instead use `make_log_docker`.

## Submission

You are to submit a `zip` file containing:
- the short video (Step 7)
- the log file (Step 8)

Please to [this link posted on Piazza](https://www.dropbox.com/request/vMaywZNxHn8zz4pIEbiI).

## Some Helpful Tips and Tricks

### Mac OSX


If you are Mac OSX user, you need to correctly setup X forwarding, which is a little more difficult than on Ubuntu. See [Sec 1.4 on the laptop setup page](http://docs.duckietown.org/daffy/opmanual_duckiebot/out/laptop_setup.html).

### Connecting to your Duckiebot

If your laptop and your Duckiebot are on the same network, you can connect through secure shell (ssh). Unless you changed the credentials during the SD card burn, the default username is `duckie` and the password is `quackquack`. Use the following command to `ssh` into the robot:
 
    ssh duckie@![ROBOT_NAME].local
     
### Portainer

However, you should rarely (if ever) need to ssh into your robot (updating the wireless connection config is one rare example), as there is a web interface that allows you to directly monitor everything. This tool is called _Portainer_. When on the same network with the Duckiebot online, navigate to the web address `![DUCKIEBOT_NAME].local:9000`. On Portainer, you can quickly manage and monitor containers, and even see output from individual containers themselves.
