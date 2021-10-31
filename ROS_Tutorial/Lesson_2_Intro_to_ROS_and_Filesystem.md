
# Lesson 2 - Intro to ROS & Filesystem :card_file_box:

## Helpful Resources :moneybag:
>**Before starting your exploration in ROS, let me list out some resources that you will find really helpful!**
>>**It is extremely recommended to check out some of this resources if you want more in-depth knowledge!**

- RoboMaster ROS Package
  - [RoboRTS](https://robomaster.github.io/RoboRTS-Tutorial/#/en/dev_guide/pre_requisites)
- Cmake 
  - [An Introduction to Modern CMake](https://cliutils.gitlab.io/modern-cmake/)

- ROS
  - [ros wiki: ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)
  - [A Gentle Introduction to ROS](https://www.cse.sc.edu/~jokane/agitr/)
  - [ROBOTIS e-Maunual](https://emanual.robotis.com/docs/en/platform/turtlebot3/learn/#learn)

- Programming 
**ROS uses Python 2.7, which is different from the more popular Python 3 (will be in ROS 2), be sure to check the version in the tutorial**
  - [w3schools: C++](https://www.w3schools.com/cpp/cpp_getstarted.asp)
  - [w3schools: Python](https://www.w3schools.com/python/default.asp)
  - [菜鸟教程: C++](https://www.runoob.com/cplusplus/cpp-tutorial.html)
  - [菜鸟教程: Python](https://www.runoob.com/python/python-tutorial.html)

- Videos
  - [ROS tutorial](https://youtu.be/ehtUb55Rmmg)
  - [ROBOTIS tutorial](https://youtu.be/1tqYrWqrbC8)
  - [古月·ROS入门21讲 ](https://www.bilibili.com/video/BV1zt411G7Vn?from=search&seid=4319717738496040484)
  - [freeCodeCamp: Python tutorial](https://youtu.be/rfscVS0vtbw)
  - [freeCodeCamp: C++ tutorial](https://youtu.be/vLnPwxZdW4Y)
  - [Simplified CMake Tutorial](https://youtu.be/mKZ-i-UfGgQ)
  - [How to CMake Good](https://youtu.be/_yFPO1ofyF0)



## ROS :robot:

**ROS, or Robot Operating System, is a well-established software platform that  provides implementation for various tools and algorithms in robotics.**


## Common ROS Filesystem Commands :keyboard:
> **Using Linux commands will be tedious and inefficient in navigating across many ROS packages. So, that is why we will learn about the following commands.**

**`roscd [package name]`**
  >**moves to the specified ROS package or stack's directory**
  *Example: roscd turtlesim //moves to the turtlesim package's directory*

**`rospack [option name] [package name]`**
  >**gets information about packages**
 *Example: rospack find turtlesim  // finds the absolute path of the turtlesim package*

**`rosls [package name]`**
 >**lists the directory content of the ROS package**
 *Example: rosls turtlesim  // lists the contents in the turtlesim package directory*
 

## Practices :zzz:

**1) moves to the "turtlesim" package directory**

**2) type the command to find the location "turtlesim" package**

**3) lists the "turtlesim" directory content**

## Answer References :100:

1)
```bash
roscd turtlesim
ucsd@ucsd-virt-ubt18:/opt/ros/melodic/share/turtlesim$
```
2)
```bash
rospack find turtlesim
/opt/ros/melodic/share/turtlesim
```
3)
```bash
rosls turtlesim
cmake  images  msg  package.xml  srv
```
---


##### *References*
1. [Navigating the ROS Filesystem](http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem)
2. [ROS Robot Programming](https://emanual.robotis.com/docs/en/platform/turtlebot3/learn/#books)
3. [ROS tutorial #03 Navigating ROS filesystem](https://www.youtube.com/watch?v=VkOC4UiAz_Y&list=PLk51HrKSBQ8-jTgD0qgRp1vmQeVSJ5SQC&index=3)
