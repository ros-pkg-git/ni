#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <sstream>

int main (int argc, char **argv) {
    // Initalize the ROS node
    ros::init(argc, argv, "kinect_scan");
    ros::NodeHandle n;
    // Get the device number
    int user_device_number = 0;
    if (argc > 1) user_device_number = atoi (argv[1]);
    // Calculate the kinect node base name
    std::stringstream kinect_node_base;
    kinect_node_base << "/kinect_base_node/" << user_device_number << "/tilt";
    // Begin Process
    std_msgs::Float64 tilt_message;
    tilt_message.data = -31.0;
    ros::Publisher tiltPublisher = n.advertise<std_msgs::Float64>(kinect_node_base.str(), 1000);
    ROS_INFO("Starting in 5 seconds......");
    ros::Duration(5.0).sleep();
    ros::spinOnce();
    ROS_INFO("Go!");
    while(ros::ok()) {
        tilt_message.data = -31.0;
        tiltPublisher.publish(tilt_message);
        ros::spinOnce();
        ros::Duration(3.5).sleep();
        tilt_message.data = 31.0;
        tiltPublisher.publish(tilt_message);
        ros::spinOnce();
        ros::Duration(3.5).sleep();
    }
    ros::spin();  
    return 0;
}
