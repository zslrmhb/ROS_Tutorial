# Lesson 6 Publisher & Subscriber :vibration_mode:
>**In this lesson, we going to revisit the package that you have created in Lesson 3, simply type the following command to the terminal!**
>>**The command will direct you to the source folder of the package, and this is the place for the source files of publisher and subscriber and possibly others!**
```bash
cd ~/[workspace-name]/src/[package-name]/src
# package-name is "beginner_tutorials"
```
## ROS Publisher :outbox_tray:
>**ROS Publisher: A node that publish message to the subscriber through a rostopic**
>>**The following code and explanation is taken and modified from the this tutorial: [ros_21_tutorials](https://github.com/huchunxu/ros_21_tutorials/tree/master/docs/slides)**

**C++ Version, name it "velocity_publisher.cpp"**

```cpp
/***********************************************************************
Copyright 2020 GuYueHome (www.guyuehome.com).
***********************************************************************/

/**
 * 该例程将发布turtle1/cmd_vel话题，消息类型geometry_msgs::Twist
 */

/**
 * This example will publish this topic: turtle1/cmd_vel，
 * topic type: geometry_msgs::Twist
 */
 
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
	// ROS节点初始化
	// Initialize ROS Node
	ros::init(argc, argv, "velocity_publisher");

	// 创建节点句柄
	// Create Nodehandle
	ros::NodeHandle n;

	// 创建一个Publisher，发布名为/turtle1/cmd_vel的topic，消息类型为geometry_msgs::Twist，队列长度10
	
	/** Create a Publisher, publish the following topic: /turtle1/cmd_vel
	 * topic type: geometry_msgs::Twist
	 * queue size: 10
	 */
	ros::Publisher turtle_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

	// 设置循环的频率
	// Set up loop rate
	ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok())
	{
	    // 初始化geometry_msgs::Twist类型的消息
		// Initialize message of this type: geometry_msgs::Twist
		geometry_msgs::Twist vel_msg;
		vel_msg.linear.x = 0.5;
		vel_msg.angular.z = 0.2;

	    // 发布消息
		// Publish message
		turtle_vel_pub.publish(vel_msg);
		ROS_INFO("Publsh turtle velocity command[%0.2f m/s, %0.2f rad/s]", 
				vel_msg.linear.x, vel_msg.angular.z);

	    // 按照循环频率延时
		// loop rate delay
	    loop_rate.sleep();
	}

	return 0;
}
```
>**For detailed explanation of the code, please visit [Writing a Simple Publisher and Subscriber (C++)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29), this is the basic format of a publisher**

>**To compile the above code and run it, write the following in the CMakeLists of the package!**
>>**This will generate a executable from the above source code and automatically link the necessary libraries for you!**
```bash
add_executable(velocity_publisher src/velocity_publisher.cpp)
target_link_libraries(velocity_publisher ${catkin_LIBRARIES})
```
>**To run the program, run the following command in terminal!**
>>**Assuming your current working directory in the terminal is the package location**
```bash
$ cd ~/[workspace-name]
$ catkin_make
$ source devel/setup.bash
$ roscore
$ rosrun turtlesim turtlesim_node
$ rosrun beginner_tutorials velocity_publisher
```


**Python Version, name it "velocity_publisher.py"**
**one thing to keep in mind is that the C++ files go to the src folder and the python files go to the scripts folder of your package (create one if you don't have one)**
```Python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

########################################################################
####          Copyright 2020 GuYueHome (www.guyuehome.com).          ###
########################################################################

# 该例程将发布turtle1/cmd_vel话题，消息类型geometry_msgs::Twist

"""
 This example will publish this topic: turtle1/cmd_vel，
 topic type: geometry_msgs::Twist
"""

import rospy
from geometry_msgs.msg import Twist

def velocity_publisher():
	# ROS节点初始化
	# Initialize ROS Node
    rospy.init_node('velocity_publisher', anonymous=True)

	# 创建一个Publisher，发布名为/turtle1/cmd_vel的topic，消息类型为geometry_msgs::Twist，队列长度10

	"""Create a Publisher, publish the following topic: /turtle1/cmd_vel
	   topic type: geometry_msgs::Twist
	   queue size: 10
	"""
    turtle_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

	#设置循环的频率
	# Set up loop rate
    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
		# 初始化geometry_msgs::Twist类型的消息
		# Initialize message of this type: geometry_msgs::Twist
        vel_msg = Twist()
        vel_msg.linear.x = 0.5
        vel_msg.angular.z = 0.2

		# 发布消息
		# Publish message
        turtle_vel_pub.publish(vel_msg)
    	rospy.loginfo("Publsh turtle velocity command[%0.2f m/s, %0.2f rad/s]", 
				vel_msg.linear.x, vel_msg.angular.z)

		# 按照循环频率延时
		# loop rate delay
        rate.sleep()

if __name__ == '__main__':
    try:
        velocity_publisher()
    except rospy.ROSInterruptException:
        pass
```
>**For detailed explanation of the code, please visit [Writing a Simple Publisher and Subscriber (Python)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)**

>**To run the program, run the following command in terminal!**
>>**Assuming your current working directory in the terminal is the package location**
```bash
$ cd ~/[workspace-name]
$ catkin_make
$ source devel/setup.bash
$ roscore
$ rosrun turtlesim turtlesim_node
$ rosrun beginner_tutorials velocity_publisher
```

## ROS Subscriber :inbox_tray:
>**ROS Subscriber: A node that receive/subscribe message from the publisher through a rostopic**
>> **The following code and explanation is taken and modified from the this tutorial: [ros_21_tutorials](https://github.com/huchunxu/ros_21_tutorials/tree/master/docs/slides)**

**C++ Version, name it "pose_subscriber.cpp"**

```cpp
/***********************************************************************
Copyright 2020 GuYueHome (www.guyuehome.com).
***********************************************************************/

/**
 * 该例程将订阅/turtle1/pose话题，消息类型turtlesim::Pose
 */

/**
 * This example will subscribe this topic: /turtle1/pose，
 * topic type: turtlesim::Pose
 */
 
#include <ros/ros.h>
#include "turtlesim/Pose.h"

// 接收到订阅的消息后，会进入消息回调函数
// When subscribed message has receive, this callback function will be call
void poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    // 将接收到的消息打印出来
	// Print out the received message
    ROS_INFO("Turtle pose: x:%0.6f, y:%0.6f", msg->x, msg->y);
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
	// Initialize ROS Node
    ros::init(argc, argv, "pose_subscriber");

    // 创建节点句柄
	// Create Nodehandle
    ros::NodeHandle n;

    // 创建一个Subscriber，订阅名为/turtle1/pose的topic，注册回调函数poseCallback
	/** Create a Subscriber, subscribe the following topic: /turtle1/pose
	 * register to this callback function: poseCallback
	 **/
    ros::Subscriber pose_sub = n.subscribe("/turtle1/pose", 10, poseCallback);

    // 循环等待回调函数
	// looping and waiting for the callback function
    ros::spin();

    return 0;
}
```
>**For detailed explaination of the code, please visit [Writing a Simple Publisher and Subscriber (C++)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29), this is the basic format of a subscriber**

>**To compile the above code and run it, write the following in the CMakeLists of the package!**
>>**This will generate a executable from the above source code and automatically link the necessary libraries for you!**
```bash
add_executable(pose_subscriber src/pose_subscriber.cpp)
target_link_libraries(pose_subscriber ${catkin_LIBRARIES})
```
>**To run the program, run the following command in terminal!**
>>**Assuming your current working directory in the terminal is the package location**
```bash
$ cd ~/[workspace-name]
$ catkin_make
$ source devel/setup.bash
$ roscore
$ rosrun turtlesim turtlesim_node
$ rosrun beginner_tutorials pose_subscriber
```


**Python Version, name it "pose_subscriber.py"**
```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

########################################################################
####          Copyright 2020 GuYueHome (www.guyuehome.com).          ###
########################################################################

# 该例程将订阅/turtle1/pose话题，消息类型turtlesim::Pose

"""
 This example will subscribe this topic: /turtle1/pose，
 topic type: turtlesim::Pose
"""

import rospy
from turtlesim.msg import Pose

def poseCallback(msg):
    rospy.loginfo("Turtle pose: x:%0.6f, y:%0.6f", msg.x, msg.y)

def pose_subscriber():
	# ROS节点初始化
	# Initialize ROS Node
    rospy.init_node('pose_subscriber', anonymous=True)

	# 创建一个Subscriber，订阅名为/turtle1/pose的topic，注册回调函数poseCallback
	"""
	Create a Subscriber, subscribe the following topic: /turtle1/pose
	register to this callback function: poseCallback
	"""
    rospy.Subscriber("/turtle1/pose", Pose, poseCallback)

	# 循环等待回调函数
	# looping and waiting for the callback function
    rospy.spin()

if __name__ == '__main__':
    pose_subscriber()

```
>**For detailed explanation of the code, please visit [Writing a Simple Publisher and Subscriber (Python)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)**

>**To run the program, run the following command in terminal!**
>>**Assuming your current working directory in the terminal is the package location**
```bash
$ cd ~/[workspace-name]
$ catkin_make
$ source devel/setup.bash
$ roscore
$ rosrun turtlesim turtlesim_node
$ rosrun beginner_tutorials pose_subscriber
```


##### *References*
1. [Writing a Simple Publisher and Subscriber (C++)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)
2. [Writing a Simple Publisher and Subscriber (Python)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)
3. [Examining the Simple Publisher and Subscriber](http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber)
4. [ROS tutorial #07 Publisher & Subscriber P1](https://www.youtube.com/watch?v=FjbBGnKElLY&list=PLk51HrKSBQ8-jTgD0qgRp1vmQeVSJ5SQC&index=7&ab_channel=ShawnChen)
5. [ROS tutorial #08 Publisher & Subscriber P2](https://www.youtube.com/watch?v=jZQ-GfJ4ZIg&list=PLk51HrKSBQ8-jTgD0qgRp1vmQeVSJ5SQC&index=8&ab_channel=ShawnChen)