# Template: template-ros-core

This template provides a boilerplate repository
for developing ROS-based software in Duckietown.

**NOTE:** If you want to develop software that does not use
ROS, check out [this template](https://github.com/duckietown/template-basic).

**NOTE:** This template builds on all of the packages [the dt-core repo](https://github.com/duckietown/dt-core). If you want to build an image without any of the packages in `dt-core` then use [the template-ros repo](https://github.com/duckietown/template-ros)


## How to use it

### 1. Fork this repository

Use the fork button in the top-right corner of the github page to fork this template repository.


### 2. Create a new repository

Create a new repository on github.com while
specifying the newly forked template repository as
a template for your new repository.


### 3. Define dependencies

List the dependencies in the files `dependencies-apt.txt` and
`dependencies-py.txt` (apt packages and pip packages respectively).


### 4. Place your code

Place your ROS packages in the directory `/packages` of
your new repository.

**NOTE:** Do not use absolute paths in your code,
the code you place under `/packages` will be copied to
a different location later.


### 5. Setup the launchfile

Change the file `launch.sh` in your repository to
launch your code.
