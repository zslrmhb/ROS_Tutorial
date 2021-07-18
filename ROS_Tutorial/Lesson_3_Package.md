# Lesson 3 - Package

## Catkin vs. Rosbuild
>**Rosbuild was an outdated buildsystem, just use Catkin!**
>>**See this post if you are curious. [catkin vs rosbuild system?](https://answers.ros.org/question/125901/catkin-vs-rosbuild-system/)** 


## Catkin Package Requirements
- **package.xml (meta information about the package)**
- **CMakeLists.txt**
- **Independent folder**

## Catkin Workspaces
>**This is the recommended method of working with catkin packages, other methods are available upon Google.**
**Below is an example workspace from ros wiki**

```
workspace_folder/         -- WORKSPACE
  src/                    -- SOURCE SPACE
    CMakeLists.txt        -- The 'toplevel' CMake file
    package_1/
      CMakeLists.txt
      package.xml
      ...
    package_n/
      CATKIN_IGNORE       -- Optional empty file to exclude package_n from being processed
      CMakeLists.txt
      package.xml
      ...
  build/                  -- BUILD SPACE
    CATKIN_IGNORE         -- Keeps catkin from walking this directory
  devel/                  -- DEVELOPMENT SPACE (set by CATKIN_DEVEL_PREFIX)
    bin/
    etc/
    include/
    lib/
    share/
    .catkin
    env.bash
    setup.bash
    setup.sh
    ...
  install/                -- INSTALL SPACE (set by CMAKE_INSTALL_PREFIX)
    bin/
    etc/
    include/
    lib/
    share/
    .catkin             
    env.bash
    setup.bash
    setup.sh
    ...
```
>**To understand the difference between 'src' , 'build' ... folders, look at this: [Catkin Workspaces](https://wiki.ros.org/catkin/workspaces)**

## Creating a ROS Workspace
**You should name the [workspace name]**
```bash
$ mkdir -p ~/[workspace-name]/src 
$ cd ~/[workspace-name]/
$ catkin_make
```

> **Upon building the catkin workspace, you should have the 'devel' folder in which there are files with the suffix '.sh' (bash files).**
>> **The command below will source the 'setup.bash', which means to execute the lines in this bash file, and setup the environment variables**
>>>**Need more info? Check this:[What does source command actually do?](https://answers.ros.org/question/188309/what-does-source-command-actually-do/)** 
```bash
$ source devel/setup.bash
```
>**To make sure the setup script ran properly, use the follow command to get the package path!**
>>**If it return your current directory, you are good to go!**
```bash
$ echo $ROS_PACKAGE_PATH
```

## Creating a ROS Package
>**The following commands will create a package, try it and your package name will be "beginner_tutorials"!**
```bash
$ cd ~/[workspace-name]/src
# catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
$ catkin_create_pkg beginner_tutorials std_msgs geometry_msgs turtlesim rospy roscpp
```
## Building a ROS Package
>**This package will be used later in Lesson 6!**
>>**ROS package naming convention: start with a uncapitalized letter, then followed by either uncapitalize letter, number or underscore** 
```bash
$ cd ~/[workspace-name]
$ catkin_make
#add workspace to ROS environment
$ . ~/[workspace-name]/devel/setup.bash
```


##### *References*
1. [Creating a ROS Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
2. [Create a ROS Workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
3. [ROS tutorial #04 Create and build ROS package](https://www.youtube.com/watch?v=4j_jsqdqLoM&list=PLk51HrKSBQ8-jTgD0qgRp1vmQeVSJ5SQC&index=4&ab_channel=ShawnChen)