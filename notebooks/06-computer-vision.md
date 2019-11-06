  # Software Exercise 6 - Computer Vision
  
  Computer vision is the field that deals with making computers understand images. While deep learning is a powerful tool, some simpler yet powerful algorithms have been developed and are still very used in robotics.
  In this assignment, you will understand how is the camera image treated by the Duckiebot in the lane following demo.
  
  ## Set up
  
  ### Getting the files
  
  Starting from the setup you had for [Hardware Exercise 2](https://github.com/duckietown-udem/udem-fall19-public/blob/master/notebooks/hw-02-pure-pursuit-control.md), you will need to pull the additional files:
  ```
  $ git pull upstream master 
  $ git submodule init
  $ git submodule update
  $ git submodule foreach "(git checkout daffy; git pull)"
  ```
  
  Then, checkout to the right commit for the simulation:
  ```
  cd simulation
  git checkout 4693556
  ```
   
  You will find in `catkin_ws/src` a folder named `sw06-CV`. There, in the `packages` repo, you will find the ROS package `sw_06_line_detector`. This is where you will work.
  
  ### Running the simulation
  To run the simulation, as in the instructions that were given in Hardware Exercise 2, from `udem-fall19-public` directory, run:
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
  $ roslaunch sw_06_line_detector lane_following.launch
  ```
  
  ### Visualizing
  Remember that you can visualize using noVNC (follow the Visualization and Debugging instructions given in [Hardware Exercise 2](https://github.com/duckietown-udem/udem-fall19-public/blob/master/notebooks/hw-02-pure-pursuit-control.md)).
  
  **Note**: the password has been changed since last time and it is now `quackquack`.
  
  Don't forget to set the verbose parameter to be true for the line detector:
  ```
  $ rosparam set /default/line_detector_node/verbose true
  ```
  
  ## The pipeline
  
  ### Role of the node
  Take a look at the node file `src/line_detector_node.py`. It is subscribed at the topic `~corrected_image/compressed` and publishes `~segment_list` and `~image_with_lines`.
  
  **Question 1**: What is the role of this node? Describe shortly its input and its outputs. Write your answer in a `06-computer-vision.txt` file.
  
  **Question 2**: In the lane following pipeline, the ground projection node uses the outputs of the line detector. Write a short description of what the ground projection node does.
  
  ### Processing the image
  When an image is received by the subscriber, it goes through several transformations and steps until it gets to lines. *Hint: `self.detector_used` can be found in `include/line_detector/line_detector1.py`*
  1. Resize and crop
  2. Setting image in line detector class
     1. Saving a BGR copy of the image
     2. Saving an HSV copy of the image
     3. Detecting the edges of the BGR image
  3. For each color in `white, yellow, red]`, detect lines
     1. Get color filtered image
     2. Dilate color filtered image
     3. Get color filtered edges
     4. Detect lines in color filtered edges
     5. Compute normals and centers of these lines
  4. Normalize the pixel coordinates and create segment list.
     
  **Question 3**: For each element of the list, explain how this process is done and why is it done. Apart for evident processes like 2.1., your explaination should be detailed. You will find useful information in the [OpenCV documentation](https://docs.opencv.org/4.0.0/d2/d96/tutorial_py_table_of_contents_imgproc.html). 
Write your answer in `06-computer-vision.txt`.
  
  ### Write your own computer vision function
  As you probably have noticed, in `line_detector1.py`, several functions are imported from `sw_06_cv_functions.py` where they simply return their equivalent in OpenCV.
  
  Your task is to replace the OpenCV functions by functions of your own in `sw_06_cv_functions.py`.
  The idea is **not** that you replace them exactly with all their options, but that you replace them well enough so that the current implementation works.
  
  You have *two* choices:
  1. Replace all the small functions in category 1.
  
      **OR**
      
  2. Replace the Canny edge detection in category 2.
  
  Add the `sw_06_cv_functions.py` file in your assignment. Make sure that your code is well documented so that we are in a good mood when correcting it :)
  
  **Question 4**: Does it run well on the simulation? Describe and comment on the results.
  
  **Question 5**: Is it as fast as the original OpenCV code? Use the `time` Python library to compare the time does it take to process one image. If there is a difference, why is it the case? 
  
  **Question 6** Try it on the real robot, using the instructions given in [Hardware Exercise 2](https://github.com/duckietown-udem/udem-fall19-public/blob/master/notebooks/hw-02-pure-pursuit-control.md#trying-your-code-on-the-robot). Does it work? Compare again the time that it takes to characterize an image on the RaspberryPi. Is it critical?
  
  Write your answers in `06-computer-vision.txt`. Don't forget to hand it in together with `sw_06_cv_functions.py`!
  
  ### Bonus
  If you feel motivated, you can also replace the Probabilistic Hough Line Detection in category 3. This is more complex! Pseudo code added in `06-computer-vision.txt` will also be accepted.
  
