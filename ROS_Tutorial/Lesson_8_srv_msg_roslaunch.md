# Lesson 8 srv, msg, and roslaunch :joystick:
## The codes in this tutorial are optional for you to try, since they are similar to the last lessons. But still worth to read it!
### **The following code and explanation is taken and modified from the this tutorial: [ros_21_tutorials](https://github.com/huchunxu/ros_21_tutorials/tree/master/docs/slides)**
#### **:warning: C++ "include" : change " learning_topic" or "learning_service" to the package you are using**
#### **:warning: Python "import" : change " learning_topic" or "learning service" to the package you are using**
## msg :page_facing_up:
> **Message file will contain the data exchanged between the nodes**

**Person.msg, create in the msg folder that you will create for the package**

```msg
string name
uint8  age
uint8  sex

uint8 unknown = 0
uint8 male    = 1
uint8 female  = 2
```
> **Find the following in the package.xml file (assuming you have created new package in your workspace or you are using a pre-existing package) and uncomment it. If not found, add these lines**
>>**These are the package dependencies when your custom message files are involved**
```bash
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```

> **Now, go to the CMakeList.txt in your current package and add the following if they does not exist**

- find_package(... message_generation)
- add_message_files(FILES Person.msg)
- generate_messages(DEPENDENCIES std_msgs)
- catkin_package(... message_runtime)

> **To make the header and other files that you need to use in the code that involves this message file, use the following command!**
```bash
$ roscd [package-name]
$ cd ../..
$ catkin_make
$ cd -
```
> **As a result from the previous steps, the files distribution for 'Person.msg' should be as follows**

**package-name will be whatever you name your package**
>> **C++ header file : ~/catkin_ws/devel/include/[package-name]/**
>> **Python script: ~/catkin_ws/devel/lib/python2.7/dist-packages/[package-name]/msg**

>**person_publisher.cpp**
```cpp
/***********************************************************************
Copyright 2020 GuYueHome (www.guyuehome.com).
***********************************************************************/

/**
 * ??????????????????/person_info??????????????????????????????learning_topic::Person
 */
 
/**
 * This example will publish this topic: /person_info???
 * topic type: learning_topic::Person  
 */
 
#include <ros/ros.h>
#include "learning_topic/Person.h"

int main(int argc, char **argv)
{
    // ROS???????????????
    // Initialize ROS Node
    ros::init(argc, argv, "person_publisher");

    // ??????????????????
    // Create Nodehandle
    ros::NodeHandle n;

    // ????????????Publisher???????????????/person_info???topic??????????????????learning_topic::Person???????????????10
    /** Create a Publisher, publish the following topic: /person_info
	 * topic type: learning_topic::Person
	 * queue size: 10
	 */

    ros::Publisher person_info_pub = n.advertise<learning_topic::Person>("/person_info", 10);

    // ?????????????????????
    // Set up loop rate
    ros::Rate loop_rate(1);

    int count = 0;
    while (ros::ok())
    {
        // ?????????learning_topic::Person???????????????
        // Initialize message of this type: learning_topic::Person
    	learning_topic::Person person_msg;
		person_msg.name = "Tom";
		person_msg.age  = 18;
		person_msg.sex  = learning_topic::Person::male;

        // ????????????
        // Publish message
		person_info_pub.publish(person_msg);

       	ROS_INFO("Publish Person Info: name:%s  age:%d  sex:%d", 
				  person_msg.name.c_str(), person_msg.age, person_msg.sex);

        // ????????????????????????
        // loop rate delay
        loop_rate.sleep();
    }

    return 0;
}
```
>**person_subscriber.cpp**
```cpp
/***********************************************************************
Copyright 2020 GuYueHome (www.guyuehome.com).
***********************************************************************/

/**
 * ??????????????????/person_info??????????????????????????????learning_topic::Person
 */

/**
 * This example will subscribe this topic: /person_info ???
 * topic type: learning_topic::Person
 */

#include <ros/ros.h>
#include "learning_topic/Person.h"

// ?????????????????????????????????????????????????????????
// When subscribed message has receive, this callback function will be call
void personInfoCallback(const learning_topic::Person::ConstPtr& msg)
{
    // ?????????????????????????????????
    // Print out the received message
    ROS_INFO("Subcribe Person Info: name:%s  age:%d  sex:%d", 
			 msg->name.c_str(), msg->age, msg->sex);
}

int main(int argc, char **argv)
{
    // ?????????ROS??????
    // Initialize ROS Node
    ros::init(argc, argv, "person_subscriber");

    // ??????????????????
    // Create Nodehandle
    ros::NodeHandle n;

    // ????????????Subscriber???????????????/person_info???topic?????????????????????personInfoCallback
    /** Create a Subscriber, subscribe the following topic: /person_info
	 * register to this callback function: personInfoCallback
	 **/
    ros::Subscriber person_info_sub = n.subscribe("/person_info", 10, personInfoCallback);

    // ????????????????????????
    // looping and waiting for the callback function
    ros::spin();

    return 0;
}
```
>**To compile the above code and run it, write the following in the CMakeLists of the package!**
>>**This will generate a executable from the above source code and automatically link the necessary libraries for you!**
```bash
add_executable(person_publisher src/person_publisher.cpp)
target_link_libraries(person_publisher ${catkin_LIBRARIES})
add_dependencies(person_publisher ${PROJECT_NAME}_generate_messages_cpp)

add_executable(person_subscriber src/person_subscriber.cpp)
target_link_libraries(person_subscriber ${catkin_LIBRARIES})
add_dependencies(person_subscriber ${PROJECT_NAME}_generate_messages_cpp)
```
>**person_publisher.py**
```Python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

########################################################################
####          Copyright 2020 GuYueHome (www.guyuehome.com).          ###
########################################################################

# ??????????????????/person_info??????????????????????????????learning_topic::Person
"""
 This example will publish this topic: /person_info???
 topic type: learning_topic::Person
"""

import rospy
from learning_topic.msg import Person

def velocity_publisher():
	# ROS???????????????
    # Initialize ROS Node
    rospy.init_node('person_publisher', anonymous=True)

	# ????????????Publisher???????????????/person_info???topic??????????????????learning_topic::Person???????????????10

    """Create a Publisher, publish the following topic: /person_info
	   topic type: learning_topic::Person
	   queue size: 10
	"""
    person_info_pub = rospy.Publisher('/person_info', Person, queue_size=10)

	#?????????????????????
    # Set up loop rate
    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
		# ?????????learning_topic::Person???????????????
        # Initialize message of this type: learning_topic::Person
    	person_msg = Person()
    	person_msg.name = "Tom";
    	person_msg.age  = 18;
    	person_msg.sex  = Person.male;

		# ????????????
        # Publish message
        person_info_pub.publish(person_msg)
    	rospy.loginfo("Publsh person message[%s, %d, %d]", 
				person_msg.name, person_msg.age, person_msg.sex)

		# ????????????????????????
        # loop rate delay
        rate.sleep()

if __name__ == '__main__':
    try:
        velocity_publisher()
    except rospy.ROSInterruptException:
        pass
```
>**person_subscriber.py**
```Python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

########################################################################
####          Copyright 2020 GuYueHome (www.guyuehome.com).          ###
########################################################################

# ??????????????????/person_info??????????????????????????????learning_topic::Person

"""
 This example will subscribe this topic: /person_info???
 topic type: learning_topic::Person
"""

import rospy
from learning_topic.msg import Person

def personInfoCallback(msg):
    rospy.loginfo("Subcribe Person Info: name:%s  age:%d  sex:%d", 
			 msg.name, msg.age, msg.sex)

def person_subscriber():
	# ROS???????????????
    # Initialize ROS Node
    rospy.init_node('person_subscriber', anonymous=True)

	# ????????????Subscriber???????????????/person_info???topic?????????????????????personInfoCallback
    """
	Create a Subscriber, subscribe the following topic: /person_info
	register to this callback function: personInfoCallback
	"""
    rospy.Subscriber("/person_info", Person, personInfoCallback)

	# ????????????????????????
    # looping and waiting for the callback function
    rospy.spin()

if __name__ == '__main__':
    person_subscriber()
```
>**So far in the lesson, you should know how to compile (C++) and execute the above code, so I will not reiterate the steps**

## srv :page_with_curl:
>**Service file contains the type of request and response data between the client and the server**
```
# request
---
# response
```
> **Same drills, this "Person.srv" file will be in the srv folder of your package**
```srv
string name
uint8  age
uint8  sex

uint8 unknown = 0
uint8 male    = 1
uint8 female  = 2

--- 
string result
```
>**person_server.cpp**
```cpp
/***********************************************************************
Copyright 2020 GuYueHome (www.guyuehome.com).
***********************************************************************/

/**
 * ??????????????????/show_person???????????????????????????learning_service::Person
 */

/**
 * This example will request this service: /show_person???
 * service message type: learning_service::Person
 */

#include <ros/ros.h>
#include "learning_service/Person.h"

int main(int argc, char** argv)
{
    // ?????????ROS??????
    // Initialize ROS Node
	ros::init(argc, argv, "person_client");

    // ??????????????????
    // Create Nodehandle
	ros::NodeHandle node;

    // ??????/spawn??????????????????????????????????????????????????????/spawn???service
	ros::service::waitForService("/show_person");
	ros::ServiceClient person_client = node.serviceClient<learning_service::Person>("/show_person");

    // ?????????learning_service::Person???????????????
    // Initialize the following request: learning_service::Person
	learning_service::Person srv;
	srv.request.name = "Tom";
	srv.request.age  = 20;
	srv.request.sex  = learning_service::Person::Request::male;

    // ??????????????????
    // Request the service
	ROS_INFO("Call service to show person[name:%s, age:%d, sex:%d]", 
			 srv.request.name.c_str(), srv.request.age, srv.request.sex);

	person_client.call(srv);

	// ????????????????????????
    // Show request result
	ROS_INFO("Show person result : %s", srv.response.result.c_str());

	return 0;
}
```
>**person_client.cpp**
```cpp
/***********************************************************************
Copyright 2020 GuYueHome (www.guyuehome.com).
***********************************************************************/

/**
 * ??????????????????/show_person???????????????????????????learning_service::Person
 */

/**
 * This example will execute this service: /show_person???
 * service message type: learning_service::Person
 */

#include <ros/ros.h>
#include "learning_service/Person.h"

// service???????????????????????????req???????????????res
// service callback function, input param: req, output param: res
bool personCallback(learning_service::Person::Request  &req,
         			learning_service::Person::Response &res)
{
    // ??????????????????
    // Show request data
    ROS_INFO("Person: name:%s  age:%d  sex:%d", req.name.c_str(), req.age, req.sex);

	// ??????????????????
    // Set up the response (output param) to the callback function
	res.result = "OK";

    return true;
}

int main(int argc, char **argv)
{
    // ROS???????????????
    // Initialize ROS Node
    ros::init(argc, argv, "person_server");

    // ??????????????????
    // Create Nodehandle
    ros::NodeHandle n;

    // ??????????????????/show_person???server?????????????????????personCallback
    /**
	 * Create this server: /show_person, and register the 
	 * callback function "personCallback"
	 */
    ros::ServiceServer person_service = n.advertiseService("/show_person", personCallback);

    // ????????????????????????
    // looping and waiting for the callback function
    ROS_INFO("Ready to show person informtion.");
    ros::spin();

    return 0;
}
```
>**person_server.py**
```Python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

########################################################################
####          Copyright 2020 GuYueHome (www.guyuehome.com).          ###
########################################################################

# ??????????????????/show_person???????????????????????????learning_service::Person

"""
This example will request this service: /show_person???
service message type: learning_service::Person
"""

import sys
import rospy
from learning_service.srv import Person, PersonRequest

def person_client():
	# ROS???????????????
    # Initialize ROS Node
    rospy.init_node('person_client')

	# ??????/spawn??????????????????????????????????????????????????????/spawn???service
    """
	When discover service "/spawn", create a server and 
	connect it to the service.
	"""
    rospy.wait_for_service('/show_person')
    try:
        person_client = rospy.ServiceProxy('/show_person', Person)

		# ???????????????????????????????????????
        # Request the service
        response = person_client("Tom", 20, PersonRequest.male)
        return response.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
	# ?????????????????????????????????
    # Show request result
    print "Show person result : %s" %(person_client())
```
>**person_client.py**
```Python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

########################################################################
####          Copyright 2020 GuYueHome (www.guyuehome.com).          ###
########################################################################

# ??????????????????/show_person???????????????????????????learning_service::Person

"""
This example will execute this service: /show_person???
service message type: learning_service::Person
"""

import rospy
from learning_service.srv import Person, PersonResponse

def personCallback(req):
	# ??????????????????
    # Show request data
    rospy.loginfo("Person: name:%s  age:%d  sex:%d", req.name, req.age, req.sex)

	# ????????????
    # return the a response
    return PersonResponse("OK")

def person_server():
	# ROS???????????????
    # Initialize ROS Node
    rospy.init_node('person_server')

	# ??????????????????/show_person???server?????????????????????personCallback
    """
	Create this server: /show_person, and register the callback 
	function "personCallback"
	"""
    s = rospy.Service('/show_person', Person, personCallback)

	# ????????????????????????
    # Wait for callback function
    print "Ready to show person informtion."
    rospy.spin()

if __name__ == "__main__":
    person_server()
```

## roslaunch :rocket:
>**A XML file that enable you to set up and launch multiple nodes at a time**
```xml
<launch>
    <node pkg="package-name" name="name-at-runtime" type="node-executable"/>
    ...
    ...
    ...
    <node pkg="package-name" name="name-at-runtime" type="node-executable"/>
</launch>
```
>**You can also set up params in the param server!**
```xml
<param name="param-name" value="parameter-value">
```
> **For more detailed version, check this out! [roslaunch/XML](http://wiki.ros.org/roslaunch/XML)**

## action (optional) :golf:
>**Action messages in ROS are suitable for situation where there is a intermediate stage between the request and response**
```action
#goal

---

#result

---

#feedback
```
**For more detail, check out this post [Writing a Simple Action Server using the Execute Callback](http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29)**


# :confetti_ball: :tada: Congratulations! You finished the entire lesson. Now, if you would like to learn more about ROS, check the resources in Lesson 2! Good Luck!

##### *References*
1. [Creating a ROS msg and srv](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)
2. [Using rqt_console and roslaunch](http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch)