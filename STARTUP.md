# Introduction to Duckietown

Welcome to Duckietown! Formally known at the University of Montreal as **IFT 6757: Autnonomous Vehicles**, this Fall, we'll be taking a different approach compared to the last few years. Thanks to the hard work of the Duckietown community, we'll supplement almost every lecture, subject, and topic with a take-home exercise. This class will run _almost_ entirely (see below) through Docker, and we'll talk about set up instructions and system requirements further along in this document.

We've made sure to keep the requirements for this class as minimal as possible, requiring only four things:

1. `git` (Installation instructions [here](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git))
2. `docker` (Installation instructions [here](https://docs.docker.com/install/))
3. A text editor or IDE of your choosing
4. `docker-compose` (Installation instructions [here](https://docs.docker.com/compose/install/))

These four can be installed on all major operating systems.


Once you install these four things, we're ready to get started. Since in the beginning, you will likely be viewing this notebook from a remote viewer, you'll want to follow along in the next steps on your local machine. Once we work through the installation, you'll be able to launch this notebook via Docker on your own machine, and participate in the practical portions later on in the notebook.

You will also want to walk through the documentation regarding accounts and software [here](https://docs.duckietown.org/DT19/AIDO/out/quickstart.html). You will need a few accounts, all of which are free to set up.

Lastly, it is most convenient (if you are on Mac / Linux) to install the `duckietown-shell`, which can be installed using the instructions above. You will want to do this on your local machine.

**Regarding the _almost entirely through Docker_ note above**: Certain parts of the class will require the _local_ installation of the `duckietown-shell`. We do not support Windows, but may be able to offer temporary solutions around this issue. 

# Getting Set Up

For the purposes of these instructions let's call the directory where you want to put all the files for this class as `$AV_ROOT` (whenever you see  you should be in the directory that you are using for the class. 

To start, let's clone the class repository using the terminal:
```
    $ cd $AV_ROOT
    $ git clone https://github.com/duckietown/udem-fall19-public --recursive
```
**Note**: If you prefer, you can fork the repository first before cloning. 

Once you clone the repository, step into the repository:

    $ cd udem-fall19-public

# Repository Walkthrough

In this section, we will briefly describe various, high-level components of the repository:

* `aido/` is the directory that we will use to submit various things to the AI Driving Olympics, hosted at NeurIPS 2019 in Vancouver. Throughout the class, many of the exercises will include components that require submission to AIDO, at which point we'll make more use of the files in this directory.
* `custom/` is a repository where _you_ will write a good amount of _general_ code that you think will be useful across exercises. In this class, the notebooks will mainly serve as _executioners_ and _monitors_ ; as much as possible, we will refrain from writing code in the notebooks, due to the difficulty of code reuse.
* `custom_ws/` is a `catkin_workspace` that will be useful to implement many of the ROS exercises. We'll talk a bit about the basics of ROS later in this notebook. `custom_ws/` also includes the ROS-port to [`gym-duckietown`](https://github.com/duckietown/gym-duckietown), a fully-functioning self-driving car simulator of the Duckietown universe.
* `simulation/` is the directory that contains `gym-duckietown`, cloned locally (rather than `pip` installed) to quickly change between branches, which will serve as different environments to test various parts of understanding throughout the exercises.
* `software/` is the local clone of the [Software](https://github.com/duckietown/Software) repository which contains almost all of the code for Duckietown. We'll be making use of this directory in this notebook, when we run the [ROS Lane Following demo](https://github.com/duckietown/challenge-aido_LF-baseline-duckietown) in simulation later in this notebook.
* `utils/` is a directory similar to `custom/` except it contains the maintainers' helper code. We will stay mostly away from this directory in this class.

In this class, **only** notebooks are to be edited from within the Docker containers. For the rest of the files, we will use our IDE / text editor to edit the files locally, and watch the changes be propogated into the container. To understand how this takes place, we will explain the basics of Docker.


# Docker Overview

From the [Docker website](https://docs.docker.com/get-started/):

Docker is a platform for developers and sysadmins to develop, deploy, and run applications with containers. The use of Linux containers to deploy applications is called containerization. Containers are not new, but their use for easily deploying applications is.

Containerization is increasingly popular because containers are:

* Flexible: Even the most complex applications can be containerized.
* Lightweight: Containers leverage and share the host kernel.
* Interchangeable: You can deploy updates and upgrades on-the-fly.
* Portable: You can build locally, deploy to the cloud, and run anywhere.
* Scalable: You can increase and automatically distribute container replicas.
* Stackable: You can stack services vertically and on-the-fly.

A container is launched by running an image. An image is an executable package that includes everything needed to run an application--the code, a runtime, libraries, environment variables, and configuration files.

A container is a runtime instance of an image--what the image becomes in memory when executed (that is, an image with state, or a user process). You can see a list of your running containers with the command, `docker ps`, just as you would in Linux.

A container runs natively on Linux and shares the kernel of the host machine with other containers. It runs a discrete process, taking no more memory than any other executable, making it lightweight.

By contrast, a virtual machine (VM) runs a full-blown “guest” operating system with virtual access to host resources through a hypervisor. In general, VMs provide an environment with more resources than most applications need.

***

With containerization, we are able to develop applications that can run across platforms and hardware, all without worrying about system setup, dependencies, etc. While Docker has many industrial applications, this becomes incredibly useful for educational purposes, and here at Duckietown, we've been lucky to have some [Docker](https://twitter.com/breandan?lang=en) [geniuses](https://censi.science/) who've even extended the dockerization to hardware, particularly Duckiebots.

In this class, your code will run entirely on containers - either on your local machine, or on our external, AIDO evaluation servers. We won't need to understand the bulk of docker - much of it has been abstracted away into the Duckietown shell - but we will want to cover one topic in particular: **volumes**.

Generally, Docker containers are made to be ephemeral - they spin up, run one process, and spin down. However, in many scenarios, it's useful to have data that persists. In our case, this data will be the hard work you spend writing code for this class. Volumes can be thought of as a shared map - they map directories from _your_ local machine to directories in the container, and allow any data in the volume to persist. This is how all of code will be edited locally on IDEs, and run in its current state on the container.

You can identify a `mount volume` call via the `-v`, which maps from `localdir:containerdir`. Most commands in this class will either be run through the shell or provided to you, but this concept is important to understand. 

# Introduction to ROS

From the [ROS Website](http://wiki.ros.org/ROS/Introduction):

ROS is an open-source, meta-operating system for your robot. It provides the services you would expect from an operating system, including hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management. 

*** 

While ROS is made to run solely on Unix, we take advantage on Docker and run a Unix container on any operating system that we'd like. In this course, we'll mostly use the following main concepts:

* ROS is a centralized (master-slave model) system, which runs _asynchronously_. This is unlike many environments popular in simulation today, such as those common in OpenAI's `gym`.
* ROS processes get spawned (and can be killed) individually, all of which depend on the longevity of the master node. ROS processes can communicate through a both a _publisher-subscriber_ paradigm, or a _service-client_ paradigm. The pubsub model is more common in ROS, and is primarily what we'll use.
* The atom of a ROS ecosystem is a *node*. Nodes broadcast to the master which _topics_ they are able to receive and send messages on.
* ROS processes get grouped and spawned via recipe files called _launchfiles_, which describe which nodes to run, when to run them, and their arguments.
* Asynchrony introduces latency, which can be affected by network bandwidth and code efficiency. Things take time to process, and if they take too long, other nodes in the pipeline can be left waiting.

# Switch to Docker

From here, your system should be mostly setup, and it is time to move into the notebook. Copy and paste the command from below and run in your terminal:


```bash
docker-compose -f docker-compose-lf.yml build
docker-compose -f docker-compose-lf.yml up
```

This will take quite a while the first time, but when it's finished, it will launch the Jupyter notebook server, which you can access to by pointing your web browser to: `http://127.0.0.1:8888/?token={some_long_token}`. From there, you can open up `notebooks/01-classical-baseline.ipynb` and navigate to the **Running the Lane Following Baseline** section. 
