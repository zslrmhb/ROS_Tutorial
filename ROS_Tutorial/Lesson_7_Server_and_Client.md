# Lesson 7 Server & Client :bellhop_bell:
## ROS Client :raising_hand_man:
>**ROS Client: A node that sends request(s) to and get response(s) from the ROS Server**
>>**The following code and explanation is taken and modified from the this tutorial: [ros_21_tutorials](https://github.com/huchunxu/ros_21_tutorials/tree/master/docs/slides)**

**C++ version, name this "turtle_spawn.cpp"**
```cpp
/***********************************************************************
Copyright 2020 GuYueHome (www.guyuehome.com).
***********************************************************************/

/**
 * 该例程将请求/spawn服务，服务数据类型turtlesim::Spawn
 */

/**
 * This example will request this service: /spawn，
 * service message type: turtlesim::Spawn
 */

#include <ros/ros.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv)
{
    // 初始化ROS节点
	// Initialize ROS Node
	ros::init(argc, argv, "turtle_spawn");

    // 创建节点句柄
	// Create Nodehandle
	ros::NodeHandle node;

    // 发现/spawn服务后，创建一个服务客户端，连接名为/spawn的service
	/**
	 * When discover service "/spawn", create a server and 
	 * connect it to the service.
	 */

	ros::service::waitForService("/spawn");
	ros::ServiceClient add_turtle = node.serviceClient<turtlesim::Spawn>("/spawn");

    // 初始化turtlesim::Spawn的请求数据
	// Initialize the following request: turtlesim::Spawn
	turtlesim::Spawn srv;
	srv.request.x = 2.0;
	srv.request.y = 2.0;
	srv.request.name = "turtle2";

    // 请求服务调用
	// Request the service
	ROS_INFO("Call service to spwan turtle[x:%0.6f, y:%0.6f, name:%s]", 
			 srv.request.x, srv.request.y, srv.request.name.c_str());

	add_turtle.call(srv);

	// 显示服务调用结果
	// Show request result
	ROS_INFO("Spwan turtle successfully [name:%s]", srv.response.name.c_str());

	return 0;
}
```
>**For detailed explanation of the code, please visit [Writing a Simple Service and Client (C++)](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29), this is the basic format of a client**

>**To compile the above code and run it, write the following in the CMakeLists of the package!**
>>**This will generate a executable from the above source code and automatically link the necessary libraries for you!**
```bash
add_executable(turtle_spawn src/turtle_spawn.cpp)
target_link_libraries(turtle_spawn ${catkin_LIBRARIES})
```
>**To run the program, run the following command in terminal!**
>>**Assuming your current working directory in the terminal is the package location**
```bash
$ cd ~/[workspace-name]
$ catkin_make
$ source devel/setup.bash
$ roscore
$ rosrun turtlesim turtlesim_node
$ rosrun beginner_tutorials turtle_spawn
```

>**Python version, name this "turtle_spawn.py**
```Python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

########################################################################
####          Copyright 2020 GuYueHome (www.guyuehome.com).          ###
########################################################################

# 该例程将请求/spawn服务，服务数据类型turtlesim::Spawn

"""
This example will request this service: /spawn，
service message type: turtlesim::Spawn
"""

import sys
import rospy
from turtlesim.srv import Spawn

def turtle_spawn():
	# ROS节点初始化
	# Initialize ROS Node
    rospy.init_node('turtle_spawn')

	# 发现/spawn服务后，创建一个服务客户端，连接名为/spawn的service
	"""
	When discover service "/spawn", create a server and 
	connect it to the service.
	"""
    rospy.wait_for_service('/spawn')
    try:
        add_turtle = rospy.ServiceProxy('/spawn', Spawn)

		# 请求服务调用，输入请求数据
		# Request the service
        response = add_turtle(2.0, 2.0, 0.0, "turtle2")
        return response.name
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
	# 服务调用并显示调用结果
	# Show request result
    print "Spwan turtle successfully [name:%s]" %(turtle_spawn())

```
>**For detailed explanation of the code, please visit [Writing a Simple Service and Client (Python)](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)**

>**To run the program, run the following command in terminal!**
>>**Assuming your current working directory in the terminal is the package location**
```bash
$ cd ~/[workspace-name]
$ catkin_make
$ source devel/setup.bash
$ roscore
$ rosrun turtlesim turtlesim_node
$ rosrun beginner_tutorials turtle_spawn
```

## ROS Server :facepalm:
>**ROS Server: A node that process request(s) from and send response(s) to the ROS Client**
>> **The following code and explanation is taken and modified from the this tutorial: [ros_21_tutorials](https://github.com/huchunxu/ros_21_tutorials/tree/master/docs/slides)**

**C++ version, name this "turtle_command_server.cpp"**
```cpp
/***********************************************************************
Copyright 2020 GuYueHome (www.guyuehome.com).
***********************************************************************/

/**
 * 该例程将执行/turtle_command服务，服务数据类型std_srvs/Trigger
 */
 
/**
 * This example will execute this service: /turtle_command，
 * service message type: std_srvs/Trigger
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>

ros::Publisher turtle_vel_pub;
bool pubCommand = false;

// service回调函数，输入参数req，输出参数res
// service callback function, input param: req, output param: res
bool commandCallback(std_srvs::Trigger::Request  &req,
         			std_srvs::Trigger::Response &res)
{
	pubCommand = !pubCommand;

    // 显示请求数据
	// Show request data
    ROS_INFO("Publish turtle velocity command [%s]", pubCommand==true?"Yes":"No");

	// 设置反馈数据
	// Set up the response (output param) to the callback function
	res.success = true;
	res.message = "Change turtle command state!"

    return true;
}

int main(int argc, char **argv)
{
    // ROS节点初始化
	// Initialize ROS Node
    ros::init(argc, argv, "turtle_command_server");

    // 创建节点句柄
	// Create Nodehandle
    ros::NodeHandle n;

    // 创建一个名为/turtle_command的server，注册回调函数commandCallback
	/**
	 * Create this server: /turtle_command, and register the 
	 * callback function "commandCallback"
	 */
    ros::ServiceServer command_service = n.advertiseService("/turtle_command", commandCallback);

	// 创建一个Publisher，发布名为/turtle1/cmd_vel的topic，消息类型为geometry_msgs::Twist，队列长度10
	/**
	  * Create a Publisher, and publish the following topic: /turtle1/cmd_vel
	  * topic type: geometry_msgs::Twist, queue size: 10
	  */
	turtle_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    // 循环等待回调函数
	// Wait for callback function
    ROS_INFO("Ready to receive turtle command.");

	// 设置循环的频率
	// Set up loop rate
	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		// 查看一次回调函数队列
		// process the callback queue once
    	ros::spinOnce();
		
		// 如果标志为true，则发布速度指令
		// if pubCommand is true, publish the "vel_msg"
		if(pubCommand)
		{
			geometry_msgs::Twist vel_msg;
			vel_msg.linear.x = 0.5;
			vel_msg.angular.z = 0.2;
			turtle_vel_pub.publish(vel_msg);
		}

		//按照循环频率延时
		// loop rate delay
	    loop_rate.sleep();
	}

    return 0;
}
```
>**For detailed explanation of the code, please visit [Writing a Simple Service and Client (C++)](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29), this is the basic format of a server**

>**To compile the above code and run it, write the following in the CMakeLists of the package!**
>>**This will generate a executable from the above source code and automatically link the necessary libraries for you!**
```bash
add_executable(turtle_command_server src/turtle_command_server.cpp)
target_link_libraries(turtle_command_server ${catkin_LIBRARIES})
```
>**To run the program, run the following command in terminal!**
>>**Assuming your current working directory in the terminal is the package location**
```bash
$ cd ~/[workspace-name]
$ catkin_make
$ source devel/setup.bash
$ roscore
$ rosrun turtlesim turtlesim_node
$ rosrun beginner_tutorials turtle_command_server
$ rosservice call /turtle_command "{}"
```

**Python version, name this "turtle_command_server.py"**
```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

########################################################################
####          Copyright 2020 GuYueHome (www.guyuehome.com).          ###
########################################################################

# 该例程将执行/turtle_command服务，服务数据类型std_srvs/Trigger

"""
This example will execute this service: /turtle_command，
service message type: std_srvs/Trigger
"""

import rospy
import thread,time
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse

pubCommand = False
turtle_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

def command_thread():	
	while True:
		if pubCommand:
			vel_msg = Twist()
			vel_msg.linear.x = 0.5
			vel_msg.angular.z = 0.2
			turtle_vel_pub.publish(vel_msg)
			
		time.sleep(0.1)

def commandCallback(req):
	global pubCommand
	pubCommand = bool(1-pubCommand)

	# 显示请求数据
	# Show request data
	rospy.loginfo("Publish turtle velocity command![%d]", pubCommand)

	# 反馈数据
	# return the a response 
	return TriggerResponse(1, "Change turtle command state!")

def turtle_command_server():
	# ROS节点初始化
	# Initialize ROS Node
    rospy.init_node('turtle_command_server')

	# 创建一个名为/turtle_command的server，注册回调函数commandCallback
	"""
	Create this server: /turtle_command, and register the callback 
	function "commandCallback"
	"""

    s = rospy.Service('/turtle_command', Trigger, commandCallback)

	# 循环等待回调函数
	# Wait for callback function
    print "Ready to receive turtle command."

    thread.start_new_thread(command_thread, ())
    rospy.spin()

if __name__ == "__main__":
    turtle_command_server()
```
>**For detailed explanation of the code, please visit [Writing a Simple Service and Client (Python)](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)**

>**To run the program, run the following command in terminal!**
>>**Assuming your current working directory in the terminal is the package location**
```bash
$ cd ~/[workspace-name]
$ catkin_make
$ source devel/setup.bash
$ roscore
$ rosrun turtlesim turtlesim_node
$ rosrun beginner_tutorials turtle_command_server
$ rosservice call /turtle_command "{}"
```



##### *References*
1. [Writing a Simple Service and Client (C++)](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29)
2. [Writing a Simple Service and Client (Python)](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)
3. [Examining the Simple Service and Client](http://wiki.ros.org/ROS/Tutorials/ExaminingServiceClient)