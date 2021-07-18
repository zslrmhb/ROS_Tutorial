# Lesson 5 Services & Parameters
>**Before starting this lesson, make sure to have turtlesim running. If not, run the following three commands in separate terminals**
```bash
$ roscore
$ rosrun turtlesim turtlesim_node
$ rosrun turtlesim turtle_teleop_key
```
## ROS Services
>**ROS Services: Different from rostopic, rosservice provide another way of communication between nodes that it allows a "request" and "response". An analogy is that the mechanism of rostopic will be the asynchronous lecture that your professor publish (publisher) and you are receiving the information from the lecture (subscriber). rosservice, on the other hand, will be in-person synchronous learning where you can get direct feedbacks from the professor**

```bash
rosservice list                      # print information about active services
rosservice call [service-name][args] # call the service with the provided args
rosservice type [service-name]       # print service type
rosservice find [service-type]       # find services by service type
rosservice info [service-name]       # print information about service
rosservice uri  [service-name]       # print service ROSRPC uri
```
>**Let us begin our journey by the following command!**
>>**We see that turtlesim has provide us with 9 services (exclude the one that start with '/rosout' and 'teleop_turtle'**
```bash
$ rosservice list
     /clear
     /kill
     /reset
     /rosout/get_loggers
     /rosout/set_logger_level
     /spawn
     /teleop_turtle/get_loggers
     /teleop_turtle/set_logger_level
     /turtle1/set_pen
     /turtle1/teleport_absolute
     /turtle1/teleport_relative
     /turtlesim/get_loggers
     /turtlesim/set_logger_level
```
>**Now, we will take a look at "/clear"**
>>***"Empty" means the service have neither inputs or outputs**
```bash
$ rosservice type /clear    # rosservice type [service-name]
    std_srvs/Empty 
```

>**Let's call this service and see what will happen!**
```bash
$ rosservice call /clear    # rosservice call [service-name][args]
```
> Insert the output image

>**Next is an example about a service that take in arguments**
>>**"|" is the pipe character which redirect the standard output of the command on the left as a standard input of the command on the right. Check out this post for more info. [Piping in Unix or Linux](https://www.geeksforgeeks.org/piping-in-unix-or-linux/)**
>>**From the result of the command, we can see that the service takes 4 arguments as input and have 1 output as seperate by the "---"**
```bash
$ rosservice type /spawn | rossrv show
      float32 x
      float32 y
      float32 theta
      string name
      ---
      string name
```
>**Let us call the service with the required arguments!**
>>**The 4th argument is optional (name for the new turtle) as the default output will be "turtle + generation number"**
```bash
$ rosservice call /spawn 2 2 0.2 ""
```
>**Input image here**

## ROS Parameters
>**Add description!**

```bash
rosparam set    [parameter-name][parameter-value]      # set parameter
rosparam get    [parameter-name]                       # get parameter
rosparam load   [yaml-file][namespace: deafult is "/"] # load parameters from file
rosparam dump   [file-name]                            # dump parameters to file
rosparam delete [parameter-name]                       # delete parameter
rosparam list                                          # list parameter names
```
>**Let's check out the current parameters in the param server!**
```bash
$ rosparam list
    add output here!
```
>**How about changing the params in "/turtlesim/background_b"!**
```bash
$ rosparam set /turtlesim/background_b 0    # rosparam set [parameter-name][parameter-value]
```
>**For the change to take in effect, we need to refresh the screen as follow**
```bash
$ rosservice call /clear
```
>**Insert picture here**

>**What if we want to get a specific param value? Try this!**
>**This will return the green portion of the background rgb**
```bash
$ rosparam get /turtlesim/background_g  # rosparam get [parameter-name]  
```

>**To get the entire content from teh parameter sever, do this!**
```bash
$ rosparam get /
```
#### Optional (THE PART BELOW IS OPTIONAL)
>**To store the current parameter to a file, we can do this!**
```bash
$ rosparam dump params.yaml # rosparam dump [file-name] 
```
>**To get the parameters from the file, try this!**
```bash
$ rosparam load params.yaml #rosparam load   [yaml-file][namespace: deafult is "/"]
$ rosparam get /turtlesim/background_r
```
##### Writing a rosparam
**The following code and explanation is taken and modified from the this tutorial: [ros_21_tutorials](https://github.com/huchunxu/ros_21_tutorials/tree/master/docs/slides)**
>**Let's write a config file for the parameter, this will be a YAML file saved in the "config" file that you create within your workspace**

```YAML
background_b: 255
background_g: 86
background_r: 69
rosdistro: 'melodic'
roslaunch:
  uris: {host_hcx_vpc__43763: 'http://hcx-vpc:43763/'}
rosversion: '1.14.3'
run_id: 077058de-a38b-11e9-818b-000c29d22e4d
```
>**Now, let us write a parameter config!**
>>**C++ version, name this "parameter_config.cpp" (place it in the "src" folder that you need to create within your workspace)**
```C++
/***********************************************************************
Copyright 2020 GuYueHome (www.guyuehome.com).
***********************************************************************/

/**
 * 该例程设置/读取海龟例程中的参数
 */

/**
 * This example will set / read the parameters in the turtlesim 
 */
#include <string>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

int main(int argc, char **argv)
{
	int red, green, blue;

    // ROS节点初始化
    // Initialize ROS Node
    ros::init(argc, argv, "parameter_config");

    // 创建节点句柄
    // Create Nodehandle
    ros::NodeHandle node;

    // 读取背景颜色参数
    // Read the background color parameters
	ros::param::get("/background_r", red);
	ros::param::get("/background_g", green);
	ros::param::get("/background_b", blue);

	ROS_INFO("Get Backgroud Color[%d, %d, %d]", red, green, blue);

	// 设置背景颜色参数
    // Set the background color parameters
	ros::param::set("/background_r", 255);
	ros::param::set("/background_g", 255);
	ros::param::set("/background_b", 255);

	ROS_INFO("Set Backgroud Color[255, 255, 255]");

    // 读取背景颜色参数
    // Read the background color parameters
	ros::param::get("/background_r", red);
	ros::param::get("/background_g", green);
	ros::param::get("/background_b", blue);

	ROS_INFO("Re-get Backgroud Color[%d, %d, %d]", red, green, blue);

	// 调用服务，刷新背景颜色
    // Call the "screen refresh" service
	ros::service::waitForService("/clear");
	ros::ServiceClient clear_background = node.serviceClient<std_srvs::Empty>("/clear");
	std_srvs::Empty srv;
	clear_background.call(srv);
	
	sleep(1);

    return 0;
}
```
>**Add explanation**
>**Python version, name this "parameter_config.py" (place it in the "scripts" folder that you need to create within your workspace)**
```Python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

########################################################################
####          Copyright 2020 GuYueHome (www.guyuehome.com).          ###
########################################################################

# 该例程设置/读取海龟例程中的参数
"""
This example will set / read the parameters in the turtlesim 
"""

import sys
import rospy
from std_srvs.srv import Empty

def parameter_config():
	# ROS节点初始化
    # Initialize ROS Node
    rospy.init_node('parameter_config', anonymous=True)

	# 读取背景颜色参数
    # Read the background color parameters
    red   = rospy.get_param('/background_r')
    green = rospy.get_param('/background_g')
    blue  = rospy.get_param('/background_b')

    rospy.loginfo("Get Backgroud Color[%d, %d, %d]", red, green, blue)

	# 设置背景颜色参数
    # Set the background color parameters
    rospy.set_param("/background_r", 255);
    rospy.set_param("/background_g", 255);
    rospy.set_param("/background_b", 255);

    rospy.loginfo("Set Backgroud Color[255, 255, 255]");

	# 读取背景颜色参数
    # Read the background color parameters
    red   = rospy.get_param('/background_r')
    green = rospy.get_param('/background_g')
    blue  = rospy.get_param('/background_b')

    rospy.loginfo("Get Backgroud Color[%d, %d, %d]", red, green, blue)

	# 发现/clear服务后，创建一个服务客户端，连接名为/clear的service
    """
    When discover service "/clear", create a server and 
	connect it to the service.
    """
    rospy.wait_for_service('/clear')
    try:
        clear_background = rospy.ServiceProxy('/clear', Empty)

		# 请求服务调用，输入请求数据
        # Request the service
        response = clear_background()
        return response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    parameter_config()
```
>**Add explanation**





##### *References*
1. [Understanding ROS Services and Parameters](http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams)
2. [rosservice](http://wiki.ros.org/rosservice)
3. [rosparam](http://wiki.ros.org/rosparam)