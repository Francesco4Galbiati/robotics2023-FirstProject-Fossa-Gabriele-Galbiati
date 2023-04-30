#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <sstream>
#include "nav_msgs/Odometry.h"
#include "first_project/Odom.h"
#include "first_project/reset_odom.h"
#include <time.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <string>

#define D 2.8

class pub_sub_odom{

	private:
	ros::NodeHandle n;

	//publisher of the nav message
	ros::Publisher odom_pub;

	//publisher of the custom message
	ros::Publisher custom_pub;

	//subscribes to the bag data to compute the odometry
	ros::Subscriber sub_odom;

	//subscribes to the odometry data to broadcast the tf
	ros::Subscriber tf_sub;

	//reset service node
	ros::ServiceServer service;

	//transform broadcaster
	tf::TransformBroadcaster br;

	//odometry values
	float o_x, o_y, o_th, o_th1;

	//time values
	double begin, timestamp,T_k;

	public:
	pub_sub_odom(){
		//nodes initialization
		odom_pub = n.advertise<nav_msgs::Odometry>("odometry", 1000);
		custom_pub = n.advertise<first_project::Odom>("custom_odometry", 1000);
		sub_odom = n.subscribe("/speed_steer", 1000, &pub_sub_odom::callbackSub, this);
		tf_sub = n.subscribe("/odometry", 1000, &pub_sub_odom::callbackTF, this);
		service = n.advertiseService("reset_odom", &pub_sub_odom::reset, this);

		//parameters initialization
		n.getParam("/starting_x", o_x);
		n.getParam("/starting_y", o_y);
		n.getParam("/starting_th", o_th);

		//time values initialization
		begin = ros::Time::now().toSec();
		timestamp = ros::Time::now().toSec();
	}

	//callback function for odometry computation
	void callbackSub(const geometry_msgs::Quaternion::ConstPtr &quat){

		//angular velocity
		float w;

		//input values from the bag
		float x = quat->x;
		float y = quat->y;

		//T_k = dt
		T_k = ros::Time::now().toSec() - timestamp;

		//This if filters the invalid T_k value at the start of the computation
		if(T_k > 1.0){
			T_k = 0.0;
		}

		//calculate the angular velocity with the bicycle approximation
		w = x * sin(y)/D;

		//uses Runge-Kutta for w = 0, exact integration for the other cases
		if(abs(w) < 1.0){
			o_th1 = o_th + w*T_k;
			o_x = o_x + x*T_k * cos(o_th + w*T_k/2);
			o_y = o_y + x*T_k * sin(o_th + w*T_k/2);
		} else{
			o_th1 = o_th + w*T_k;
			o_x = o_x + (x/w)*(sin(o_th1) - sin(o_th));
			o_y = o_y - (x/w)*(cos(o_th1) - cos(o_th));
		}

		timestamp = ros::Time::now().toSec() - begin;

		//update theta value
		o_th = o_th1;

		//publish values
		nav_msgs::Odometry odometry;
		first_project::Odom custom_odometry;
		ros::Time stamp(timestamp);

		//odometry message
		odometry.pose.pose.position.x = o_x;
		odometry.pose.pose.position.y = o_y;
		odometry.pose.pose.position.z = 0;
		odometry.pose.pose.orientation.x = 0;
		odometry.pose.pose.orientation.y = 0;
		odometry.pose.pose.orientation.z = o_th;
		odometry.pose.pose.orientation.w = 0;
		odometry.twist.twist.linear.x = x * sin(o_th);
		odometry.twist.twist.linear.y = x * cos(o_th);
		odometry.twist.twist.linear.z = 0;
		odometry.twist.twist.angular.x = w * sin(o_th);
		odometry.twist.twist.angular.y = w * cos(o_th);
		odometry.twist.twist.angular.z = 0;
		odometry.header.stamp = stamp;
		odometry.header.frame_id = "odom";
		odometry.child_frame_id = "odom_child";

		//custom message
		custom_odometry.x = o_x;
		custom_odometry.y = o_y;
		custom_odometry.th = o_th;
		custom_odometry.timestamp = std::__cxx11::to_string(timestamp);

		//publishing
		odom_pub.publish(odometry);
		custom_pub.publish(custom_odometry);
	}

	//callback function for tf broadcasting
	void callbackTF(const nav_msgs::Odometry::ConstPtr &odom){
		tf::Transform odom_transform;
		odom_transform.setOrigin(tf::Vector3(odom->pose.pose.position.x, odom->pose.pose.position.y, 0));
		tf::Quaternion odom_quat;
		odom_quat.setRPY(0, 0, odom->pose.pose.orientation.z);
		odom_transform.setRotation(odom_quat);
		br.sendTransform(tf::StampedTransform(odom_transform, ros::Time::now(), "world", "vehicle"));
	}

	//callback function for reset service
	bool reset(first_project::reset_odom::Request &req, first_project::reset_odom::Response &res){
		o_x = 0.0;
		o_y = 0.0;
		o_th = 0.0;
		res.resetted = true;
		return true;
	}
};

int main(int argc, char** argv){

	ros::init(argc, argv, "odom_node");

	pub_sub_odom odometry;

	ros::spin();

	return 0;
}
