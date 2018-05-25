# Jormungandr
Main Software Repository for Subbots

![alt tag](https://travis-ci.org/ubc-subbots/Jormungandr.svg?branch=master)

## Table of Contents
- [Setup](#setup)
  - [Rosinstall](#rosinstall)
- [Conventions](#conventions)
  - [Github](#github-conventions)
  - [Coding Conventions](#coding-conventions)
  - [Coordinate Systems](#coordinate-systems)
- [Creating a New Node](#creating-a-new-node)
- [Creating a New Simple Node](#creating-a-new-simple-node)
- [Launch Files](#launch-files)
- [Testing](#testing)
  - [GTest](#gtest)
  - [Rostest](#rostest)
- [Running Packages](#running-packages)
  - [Running UWSim](#to-run-uwsim)
  - [Running Phidget IMU](#running-phidget)
  - [Running Camera Driver](#cammera-driver)


## Setup 

### RosInstall

Rosinstall is addressed and run in `install_dependencies.sh`, should there be any need to run manual, rosinstall should be run after all steps from installation and setup has been completed. 

## Conventions

### Github Conventions
- We follow the Forking Workflow: here is what it is [here](https://www.atlassian.com/git/tutorials/comparing-workflows#forking-workflow) and how to use it [here](https://gist.github.com/Chaser324/ce0505fbed06b947d962)
- Only commit files that are essential for the system to run; do not put any photos or videos in here
- All files **must** be formatted properly. Formatting will be enforced with the `clang-format` tool. 
    - To check and fix formatting, from the `Jormungandr` folder run `./clang_format/fix_formatting.sh BRANCH_NAME`, where `BRANCH_NAME` is the name of the branch you intend to merge your code into (eg. `master`). This script will fix any improperly formatted code, but will refuse to change any files with uncommited changes (to prevent you losing work)
- Once your pull request has been reviewed and revised until it looks good from both your and the reviewers' sides, go ahead and Squash and Merge it, which will squash all the commits on your pull request into one and merge it to the target branch.

### Coding Conventions
- Every **.cpp** and **.h** file should start with 
```
/*
 * Created By: Someone
 * Created On: December 1st, 2000
 * Description: A quick description of what this file does/is for
 */
```

- Functions should be commented a la JavaDoc
- The Javadoc comment (below) should be directly above every function in the header file
```
/**
 * One line description of the function
 *
 * A longer and more in depth description of the function
 * if it is needed.
 * 
 * @param param_one the first parameter of the function
 * @param param_two the second parameter of the function whose
 *                  description goes longer than one line
 * @return what the function returns if it returns anything
 * 
 */
```

- Classes are **CamelCase**
- Variables are **non_camel_case**
- Constants are **ALL_CAPS_WITH_UNDERSCORES**
- Functions are **camelCase**
- Indentations are 4 spaces

- `CMakeLists.txt` files should be reduced to only contain the minimum amount of comments. The version in `sample_package` has all the comments left in (for the sake of verbosity), so for a more representative example of what yours should look like, see Snowbots' repo: [`Snowflake/src/sb_vision/CMakeLists.txt`](https://github.com/UBC-Snowbots/Snowflake/blob/master/src/sb_vision/CMakeLists.txt) (or really any package aside from `sample_package`)

### Coordinate Systems
- We try to follow ROS standards, which can be found [here](http://www.ros.org/reps/rep-0103.html)
- x : forward
- y : left
- z : up
```
              +X
              ^
              |
      +θ  +<----->+ -θ
          |   |   |
          V   |   V
+Y <---------------------> -Y
              |
              |
              |
              V
              -X
```

## Creating a new node
- If your node is at all complicated, then this format should be followed. For simple nodes, please [see below](#creating-a-new-simple-node)
- Each node should be class based
- **MyNode.h** should contain your class declaration
- **MyNode.cpp** should contain your class definition
- **my_node.cpp** should be relatively small, and should just contain a **main** function to run your node
- **my-node-test.cpp** should contain all your gtest (unit test)
- **my_node_rostest.cpp** should contain your rostest (integrated test)
- For an example of this, please see `src/sample_package`
<pre>
some_ros_package
|  CMakeLists.txt
|  package.xml
└───src
|   | <b>MyNode.cpp</b> 
|   | <b>my_node.cpp</b>
|
└───launch
|   | <b>my_node.launch</b>
|
└───include
|   | <b>MyNode.h</b>
| 
└───test
|   | <b>my-node-test.cpp</b>
|   | <b>my_node_rostest.cpp</b>
|   | <b>sample_package_test.test</b>
</pre>

## Creating a new **simple** node
- You will not be able to write unit tests for this type of node, so it must be extremely simple
- **my_node.cpp** will contain the entire node, and will probably contain a `while(ros::ok()){}` loop
<pre>
some_ros_package
|  CMakeLists.txt
|  package.xml
└───src
|   | <b>my_node.cpp</b>
</pre>

## Launch files
- A launch file (ends in .launch) is an easy way to run multiple nodes with specified parameters with one single command. This is useful if a package has multiple nodes that needs to be run. A launch file can also launch other smaller launch files so it can be used to start every node needed for the robot to run properly. Reference [here](http://wiki.ros.org/roslaunch).  
- To use the launch file, the command is: `roslaunch package_name package_launch_file.launch`

## Testing
### GTest
- GTest is our primary testing tool at the moment. The ROS wiki has a quick intro to it [here](http://wiki.ros.org/gtest), and we also strongly recommend you read Google's introduction to it [here] (https://github.com/google/googletest/blob/master/googletest/docs/Primer.md), then setup and write a few example tests before you start using it with ROS.
- Once you've setup your tests in ROS, run `catkin_make run_tests` to run them
- To test a specific package, run `catkin_make run_tests_MY_PACKAGE_NAME`

### Rostest
- For tests which require more than one active node, i.e. integrated testing, the rostest framework provides a way to launch your test alongside all the nodes it requires. This is an extension on roslaunch enabling it to run test nodes. Special test nodes are nested within a `<test></test>` tag. This also needs a special entry under CMakelists as shown in the sample package. See more details [here](http://wiki.ros.org/rostest)
- Similar to launch files, the command is: `rostest package_name package_test_file.test`.


## Running Packages

The three packages below are installed by .rosinstall file. 
Here are some common problems: 
Q1: When I type `rosrun`, tab does know show suggestions for the packages below. 
A1: This could be due to the `setup.bash` file not being sources. The `setup.bash` file to source is located in `Jormungandr/external_pkg/setup.bash`

### To Run UWSim:
  First time setting up: 
  1: Run `install_dependencies.sh` script in main repository.   
  2: Add source `~/Jormungandr/devel/setup.sh` to `.bashrc`, or source this file every time.
   
  To run: 
  Run `~/Jormungandr/src/simulator/run_simulator.sh` "scene" (Where "scene" is one of the xml scenes in `Jorgumandr/src/simulator/scenes` ie. `Qualification.xml`)

### Running Phidget

  1. Run `source Jormunganrd/external_pkg/setup.bash` 
  2. `rosrun phidgets_imu phidgets_imu_node`

### Camera Driver

  1. Run `source Jormunganrd/external_pkg/setup.bash` 
  2. `rosrun usb_cam usb_cam_node`
