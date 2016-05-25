#include <libfreenect.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>

#include <string>
#include <sstream>

freenect_context *f_ctx;
freenect_device *f_dev;
int user_device_number = 0;

ros::Publisher imu_publisher;

void tilt_received_callback(const std_msgs::Float64::ConstPtr& tilt) {
    /* Tilt the camera */
    freenect_set_tilt_degs (f_dev, tilt->data);
    ROS_INFO("Tilting to: %lf degrees", tilt->data);
}

void led_received_callback(const std_msgs::Int32::ConstPtr& led) {
    /* Set the LED state */
    freenect_set_led(f_dev, (freenect_led_options) led->data);
    ROS_INFO("Setting LED: %d", led->data);
}

void imu_publish_data(const ros::TimerEvent& e) {
    sensor_msgs::Imu imu_msg_;
    double aX = 0.0, aY = 0.0, aZ = 0.0;

    freenect_raw_device_state *state;
    freenect_update_device_state (f_dev);
    state = freenect_get_device_state (f_dev);
    freenect_get_mks_accel (state, &aX, &aY, &aZ);

    imu_msg_.header.stamp = ros::Time::now();
    imu_msg_.linear_acceleration.x = aX;
    imu_msg_.linear_acceleration.y = aY;
    imu_msg_.linear_acceleration.z = aZ;
    imu_msg_.linear_acceleration_covariance[0] = imu_msg_.linear_acceleration_covariance[4]
        = imu_msg_.linear_acceleration_covariance[8] = 0.01; // @todo - what should these be?
    imu_msg_.angular_velocity_covariance[0] = -1; // indicates angular velocity not provided
    imu_msg_.orientation_covariance[0] = -1; // indicates orientation not provided
    imu_publisher.publish(imu_msg_);
}

int main (int argc, char **argv) {
    // Initalize the ROS node
    ros::init(argc, argv, "kinect_base_node");
    ros::NodeHandle n;
    // Initalize the Freenect Context
    if (freenect_init (&f_ctx, NULL) < 0) {
        ROS_INFO("freenect_init() failed\n");
        return 1;
    }
    freenect_set_log_level (f_ctx, FREENECT_LOG_INFO);
    // Scan for kinect devices
    int nr_devices = freenect_num_devices (f_ctx);
    ROS_INFO("Number of devices found: %d\n", nr_devices);
    // Get the device number
    if (argc > 1) user_device_number = atoi (argv[1]);
    if (nr_devices < 1) return 1;
    // Calculate the kinect node base name
    std::stringstream kinect_node_base;
    kinect_node_base << "/kinect_base_node/" << user_device_number << "/";
    // Open the base portion of the Kinect
    if (freenect_open_device (f_ctx, &f_dev, user_device_number) < 0) {
        ROS_INFO("Could not open device\n");
        return 1;
    }
    // Get the defaults 
    double  tiltDefaultPosition = 0.0;
    double  imuDefaultDuration = 1.0;
    int     ledDefaultState = LED_BLINK_GREEN;
    n.getParam(kinect_node_base.str() + "tilt", tiltDefaultPosition);
    n.getParam(kinect_node_base.str() + "led", ledDefaultState);
    n.getParam(kinect_node_base.str() + "imuDuration", imuDefaultDuration);
    // Set the default kinect state
    freenect_set_tilt_degs(f_dev, tiltDefaultPosition);
    freenect_set_led(f_dev, (freenect_led_options) ledDefaultState);
    // Create the provided services
    ros::Subscriber subTilt = n.subscribe(kinect_node_base.str() + "tilt", 1000, tilt_received_callback);
    ros::Subscriber subLED = n.subscribe(kinect_node_base.str() + "led", 1000, led_received_callback);
    imu_publisher = n.advertise<sensor_msgs::Imu>(kinect_node_base.str() + "imu", 1000);
    ros::Timer imu_publish_timer = n.createTimer(ros::Duration(imuDefaultDuration), imu_publish_data);
    ros::spin();   
    return 0;
}
