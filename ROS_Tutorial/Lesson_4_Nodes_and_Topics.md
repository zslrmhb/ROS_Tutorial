# Lesson 4 - Nodes & Topics
> **Starting from this lesson, we will be focusing on using a package called turtlesim, a wonderful tool for learning ROS**
---

> **Before launching the turtlesim, we will need to type the command below. You can think of this as the "starting button" of ROS. Therefore, keep this terminal active as long as you are running ROS!**
```bash
$ roscore
```
> **Now is the time to launch turtlesim! Open a separate terminal and type the following!**
>> **"rosrun [package-name][node-name]" will run an executable specify in the "node-name" from an arbitrary ROS package**
```bash
$ rosrun turtlesim turtlesim_node
```


>**Let's also enable keyboard control!**
```bash
$ rosrun turtlesim turtle_teleop_key
```

## ROS Nodes
>**ROS Nodes: Basic communication and execution units in ROS, can either publish or receive message(s) via a ROS topic**

```bash 
rosnode info    [node-name]    # print information about node
rosnode kill    [node-name]    # kill a running node
rosnode list                   # list active nodes
rosnode machine [machine-name] # list nodes running on a particular machine or list machine   
rosnode ping    [node-name]    # test connectivity to node
rosnode cleanup                # purge registration information of unreachable nodes
```
> **Let us use the commands above to explore the properties of the "turtlesim_node"!**
> **First, let's use "rosnode list" to list the active nodes"**
```bash
$ rosnode list // also put the output below
```
> **Since we know the current node is "turtlesim", let's find out its information!**
```bash
$ rosnode info /turtlesim
```
> **Let's ping the node to check our connectivity!**
>> **To exit the ping, type "Ctrl + C", this will kill the current running task**
```bash
$ rosnode ping /turtlesim
```
> **You can also practice to relaunch the package by killing this node, it will be optional.**
```bash
$ rosnode kill /turtlesim
```
>**Don't forget to practice other commands!**




## ROS Topics
>**ROS Topics: The "lines" that connect the communication between nodes (publishing and subscribing)**

```bash
rostopic bw    [topic-name]   # display bandwidth used by topic
rostopic delay [topic-name]   # display delay for topic which has header
rostopic echo  [topic-name]   # print messages to screen
rostopic find  [message-type] # find topics by type
rostopic hz    [topic-name]   # display publishing rate of topic
rostopic info  [topic-name]   # print information about active topic
rostopic list                 # print information about active topics
rostopic pub   [topic-name][message-type][args] # publish data to topic
rostopic type  [topic-name]   # print topic type
```
>**Before we hop in the exploration of ROS Topics, we need to install the rqt package, a ROS framework for Graphical User Interface(GUI).**
>>**When prompt to type the password, enter the one that you put for logging into the current operating system**
>>>**For detail info of "apt-get" and relevant commands, check out this post**
>>>[Using apt-get Commands In Linux](https://itsfoss.com/apt-get-linux-guide/)
```bash
$ sudo apt-get install ros-<melodic>-rqt
$ sudo apt-get install ros-<melodic>-rqt-common-plugins
```
>**Then, we run the follow commands**
>>**Put the picture of the vis)  You will see the following visualization of the relationship between nodes. In this case, "/teleop_turtle" and "/turtlesim" communicate through the topic "/turtle1/command_velocity**
```bash
$ rosrun rqt_graph rqt_graph
```


>**Next, the following will help you with the format of the rostopic command. Same thing apply to the "rosnode", too. Try it out !**
```bash
$ rostopic -h
      rostopic bw     display bandwidth used by topic
      rostopic echo   print messages to screen
      rostopic hz     display publishing rate of topic    
      rostopic list   print information about active topics
      rostopic pub    publish data to topic
      rostopic type   print topic type
```
>**Or, you can first type "rostopic" and then press the tab key, and you should got the possible options available. As you move further, you will find out that tab key is super useful if you don't want to type the entire command**
```bash
$ rostopic
bw    echo  find  hz    info  list  pub   type 
```
>**We want to check the current velocity of the turtle, try out the following!**
>>**In order for messages to be publish on this terminal, we will need to move the turtle using arrow keys on the turtle_teleop_key terminal!**
```bash
$ rostopic echo /turtle1/cmd_vel
```


>**Also, take a look at rqt_graph and press the blue refresh button on the upper-left. You will see that rostopic echo (in red) have subscribed to "turtle1/command_velocity"**



>**Likewise, for "rostopic list", we can use the following command to check for its arguments. Same thing apply to the commands in "rosnode" and others**
>>**Feel free to try out the arguments!**
```bash
$ rostopic list -h
     Usage: rostopic list [/topic]
     
     Options:
       -h, --help            show this help message and exit
       -b BAGFILE, --bag=BAGFILE
                             list topics in .bag file
       -v, --verbose         list full details about each topic
       -p                    list only publishers
       -s                    list only subscribers
```
### ROS Messages
> **ROS Messages: The data that are send between ROS Nodes through the ROS Topics**
>>**Publisher -> same type of message (topic type depends on this) -> Subscriber**

> **Now, let us try out the follow command to determine the topic type!**
```bash
$ rostopic type /turtle1/cmd_vel    # rostopic type  [topic-name]
    geometry_msgs/Twist
```
> **Well, what message is contain in this topic (geometry_msgs/Twist)? Try this out!**
```bash
$ rosmsg show geometry_msgs/Twist
     geometry_msgs/Vector3 linear
       float64 x
       float64 y
       float64 z
     geometry_msgs/Vector3 angular
       float64 x
       float64 y
       float64 z
```
>**Do you want to control the turtle without the keyboard? Try this!**
>>**The "-1" is an optional argument that cause rostopic to only publish one message and then exit**
**The 6 input arguments below correspond to the linear and angular x, y, z. To know more info. about this syntax, check out this post! [YAML on the ROS command line](http://wiki.ros.org/ROS/YAMLCommandLine)**
```bash
$ rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0,0.0,0.0]' '[0.0,0.0,1.8]'       
# rostopic pub   [topic-name][message-type][args]
```

>**If your turtle stopped, it is normal! We need a to continue publish the commands to get a continuous movement, and the "-r" argument will do it for us at a rate of 1 Hz!**
>>**Remember to check out the rqt_graph and see what new nodes are added!**
```bash
$ rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
```

>**Since "/turtle1/pose" subscribes to "/turtle1/cmd_vel geometry_msgs/Twist", we can see the data being published by the following**
```bash
$ rostopic echo /turtle1/pose
```
>**We can also see the rate of publishing!**
```bash
$ rostopic hz /turtle1/pose
```
#### Optional
>**We can also plot the data above to a graph, which we will only introduce you with the follow command and not discuss further**
```bash
$ rosrun rqt_plot rqt_plot
```


##### *References*
1. [Understanding ROS Nodes](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)
2. [Understanding ROS Topics](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)
3. [ROS tutorial #05 ROS topic, nodes, messages P1](https://www.youtube.com/watch?v=Hw0l7YE0Yis&list=PLk51HrKSBQ8-jTgD0qgRp1vmQeVSJ5SQC&index=5&ab_channel=ShawnChen)
4. [ROS tutorial #06 ROS topic, nodes, messages P2](https://www.youtube.com/watch?v=a_xeK1HdVPg&list=PLk51HrKSBQ8-jTgD0qgRp1vmQeVSJ5SQC&index=6&ab_channel=ShawnChen)
5. [rostopic](http://wiki.ros.org/rostopic)
6. [rosnode](http://wiki.ros.org/rosnode)