#include "ros/ros.h"
#include "std_msgs/Float32.h"

void sensor_callback(const std_msgs::Float32 &value) {
	ROS_DEBUG("Received the message: Value= %d", value);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "spot_ros_interface");
	ros::NodeHandle n;

	ros::Publisher spot_state_pub = n.advertise<std_msgs::Float32>("spot_state", 100);
	// ros::Publisher sensor_pub = n.advertise<std_msgs::Float32>("another_counter", 100);
	// ros::Subscriber sensor_pos_sub = n.subscribe("test_msg_from_python", 100, sensor_callback);
	// ros::Subscriber sensor_pos_sub = n.subscribe("test_msg_from_python", 100, sensor_callback);

	// Running at 10Hz
	ros::Rate loop_rate(10);
	int count = 0;
	ros::spinOnce();

	while (ros::ok()) {

		std_msgs::Float32 counter;
		counter.data=count;
		spot_state_pub.publish(counter);

		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

	return 0;
}