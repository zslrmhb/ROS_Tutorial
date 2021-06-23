# Lesson 4 - Nodes & Topics

## ROS Nodes
>**Basic communication units in ROS, can either publish or receive message(s) via a ROS topic**

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

## Practices
##### *References*
1. [Understanding ROS Nodes](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)
2. [Understanding ROS Topics](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)
3. [ROS tutorial #05 ROS topic, nodes, messages P1](https://www.youtube.com/watch?v=Hw0l7YE0Yis&list=PLk51HrKSBQ8-jTgD0qgRp1vmQeVSJ5SQC&index=5&ab_channel=ShawnChen)
4. [ROS tutorial #06 ROS topic, nodes, messages P2](https://www.youtube.com/watch?v=a_xeK1HdVPg&list=PLk51HrKSBQ8-jTgD0qgRp1vmQeVSJ5SQC&index=6&ab_channel=ShawnChen)
5. [rostopic](http://wiki.ros.org/rostopic)
6. [rosnode](http://wiki.ros.org/rosnode)