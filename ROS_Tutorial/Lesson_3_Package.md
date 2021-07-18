# Lesson 3 - Package

## Catkin vs. Rosbuild

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

## Creating a ROS Workspace
**You should name the [workspace name]**
```bash
$ mkdir -p ~/[workspace-name]/src 
$ cd ~/[workspace-name]/
$ catkin_make
```

> add more
```bash
$ source devel/setup.bash
```
>add more
```bash
$ echo $ROS_PACKAGE_PATH
```
## Creating a ROS Package
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