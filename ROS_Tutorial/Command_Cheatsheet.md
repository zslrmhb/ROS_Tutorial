# Command Cheetsheet
>**Other cheatsheets that I found**
- [ROS Cheat Sheet](https://mirror.umd.edu/roswiki/attachments/de/ROScheatsheet.pdf)
- [ROS CHEAT SHEET MELODIC](https://www.generationrobots.com/media/ROS_Cheat_Sheet_Melodic.pdf)
- [ROS CHEAT SHEET](https://kapeli.com/cheat_sheets/ROS.docset/Contents/Resources/Documents/index)
- [Robotic Operating System - Cheat Sheet (ROS Melodic)](https://pk.sedenius.com/wp-content/uploads/2020/08/sedenius_ros_cheatsheet.pdf)

## ROS Workspace

## ROS Packages










## ROS Nodes
>**Basic communication and execution units in ROS, can either publish or receive message(s) via a ROS topic**

```bash 
rosnode info    [node-name]    # print information about node
rosnode kill    [node-name]    # kill a running node
rosnode list                   # list active nodes
rosnode machine [machine-name] # list nodes running on a particular machine or list machine   
rosnode ping    [node-name]    # test connectivity to node
rosnode cleanup                # purge registration information of unreachable nodes
```
## ROS Topics
>**The "lines" that connect the communication between nodes (publishing and subscribing)**

```bash
rostopic bw    [topic-name]   # display bandwidth used by topic
rostopic delay [topic-name]   # display delay for topic which has header
rostopic echo  [topic-name]   # print messages to screen
rostopic find  [message-type] # find topics by type
rostopic hz    [topic-name]   # display publishing rate of topic
rostopic info  [topic-name]   # print information about active topic
rostopic list                 # print information about active topics
rostopic pub   [topic-name][topic-type][data] # publish data to topic
rostopic type  [topic-name]   # print topic type
```

## ROS Services

```bash
rosservice list                      # print information about active services
rosservice call [service-name][args] # call the service with the provided args
rosservice type [service-name]       # print service type
rosservice find [service-type]       # find services by service type
rosservice info [service-name]       # print information about service
rosservice uri  [service-name]       # print service ROSRPC uri
```

## ROS Parameters

```bash
rosparam set    [parameter-name][parameter-value]      # set parameter
rosparam get    [parameter-name]                       # get parameter
rosparam load   [yaml-file][namespace: deafult is "/"] # load parameters from file
rosparam dump   [file-name]                            # dump parameters to file
rosparam delete [parameter-name]                       # delete parameter
rosparam list                                          # list parameter names
```

## Cmake

## roslaunch