/// @file manus_ros2.cpp
/// @brief This file contains the main function for the manus_ros2 node, which interfaces with the Manus SDK to
/// receive animated skeleton data, and republishes the events as ROS 2 messages.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sys/time.h>

// #include "rclcpp/rclcpp.hpp"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseArray.h"
#include "SDKMinimalClient.hpp"
#include <fstream>
#include <iostream>
#include <thread>


using namespace std::chrono_literals;
using namespace std;




/// @brief ros1 publisher class for the manus_ros1 node
class ManusROS1Publisher : public ros::NodeHandle
{
public:
    ManusROS1Publisher()
    {
        manus_left_publisher_ = this->advertise<geometry_msgs::PoseArray>("manus_left", 10);
        manus_right_publisher_ = this->advertise<geometry_msgs::PoseArray>("manus_right", 10);
    }

    void publish_left(geometry_msgs::PoseArray* pose_array) {
        manus_left_publisher_.publish(*pose_array);
    }

    void publish_right(geometry_msgs::PoseArray* pose_array) {
        manus_right_publisher_.publish(*pose_array);
    }

private:
    // ros::NodeHandle& nh_;
    ros::Publisher manus_left_publisher_;
    ros::Publisher manus_right_publisher_;
};




void convertSkeletonDataToROS(std::shared_ptr<ManusROS1Publisher> publisher)
{
	ClientSkeletonCollection* csc = SDKMinimalClient::GetInstance()->CurrentSkeletons();
	if (csc != nullptr && csc->skeletons.size() != 0) {
    	for (size_t i=0; i < csc->skeletons.size(); ++i) {
      
			// Prepare a new PoseArray message for the data
			geometry_msgs::PoseArray* pose_array;
			pose_array->header.stamp = ros::Time::now();

			// Set the poses for the message
      		for (size_t j=0; j < csc->skeletons[i].info.nodesCount; ++j) {
        		const auto &joint = csc->skeletons[i].nodes[j];
				geometry_msgs::Pose pose;
				pose.position.x = joint.transform.position.x;
				pose.position.y = joint.transform.position.y;
				pose.position.z = joint.transform.position.z;
				pose.orientation.x = joint.transform.rotation.x;
				pose.orientation.y = joint.transform.rotation.y;
				pose.orientation.z = joint.transform.rotation.z;
				pose.orientation.w = joint.transform.rotation.w;
				pose_array->poses.push_back(pose);
			}

			// Which hand is this?
			if (csc->skeletons[i].info.id == SDKMinimalClient::GetInstance()->GetRightHandID()) {
				pose_array->header.frame_id = "manus_right";
				publisher->publish_right(pose_array);
			} else {
				pose_array->header.frame_id = "manus_left";
				publisher->publish_left(pose_array);
			}
		}
	}

}


// Main function - Initializes the minimal client and starts the ROS2 node
int main(int argc, char *argv[])
{
	// rclcpp::init(argc, argv);
	ros::init(argc, argv, "manus_ros_node");
	
	// ManusROS1Publisher publisher;
	

	
	auto publisher = std::make_shared<ManusROS1Publisher>();
	ROS_INFO("Starting manus_ros node");

	SDKMinimalClient t_Client(publisher);
	ClientReturnCode status = t_Client.Initialize();

	if (status != ClientReturnCode::ClientReturnCode_Success)
	{
		ROS_ERROR_STREAM("Failed to initialize the Manus SDK. Error code: " << (int)status);
		return 1;
	}
	
    ROS_INFO("Connecting to Manus SDK");
	t_Client.ConnectToHost();

	
    // Create a timer to publish the poses at 50Hz
    ros::Rate rate(50); // 50 Hz
    while (ros::ok()) {
        if (t_Client.Run()) {
            convertSkeletonDataToROS(publisher);
        }
        ros::spinOnce();
        rate.sleep();
    }

    // Shutdown the Manus client
    t_Client.ShutDown();

    // Shutdown ROS
    ros::shutdown();

	return 0;
}
