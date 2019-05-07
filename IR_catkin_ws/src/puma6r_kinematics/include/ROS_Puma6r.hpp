#include "ros/ros.h"
#include <TypeDefs.hpp>
#include "Puma6r.hpp"
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

class ROS_Puma6r {
	private:
		IRlibrary::Puma6r obj6R;
		ros::Publisher fkCheckPub;
		ros::Subscriber configSub;
		std_msgs::Bool fk_check;
		tf::TransformListener listener;
	public:
		ROS_Puma6r(ros::NodeHandle n6) {
			double l1, l2, l3, l4, l5;
			n6.getParam("link_lengths/l1", l1);
			n6.getParam("link_lengths/l2", l2);
			n6.getParam("link_lengths/l3", l3);
			n6.getParam("link_lengths/l4", l4);
			n6.getParam("link_lengths/l5", l5);
			obj6R.setLinks(l1, l2, l3,l4,l5);
			configSub = n6.subscribe("/joint_states", 5, &ROS_Puma6r::configCallBack, this);
			fkCheckPub = n6.advertise <std_msgs::Bool> ("/fkCheck", 5);
		}

		void configCallBack(const sensor_msgs::JointState::ConstPtr &msg) {
			ros::Time then = msg->header.stamp;
			IRlibrary::Vec6 q;
			q << msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4], msg->position[5];
			obj6R.setConfig(q);
			auto x = obj6R.getX();
			tf::Transform transform;
			transform.setOrigin( tf::Vector3(x[0], x[1], x[2]) );
			tf::Quaternion quat;
			auto axis_angle = obj6R.getAxisAngle();
			tf::Vector3 axis_rot (axis_angle.omega[0], axis_angle.omega[1], axis_angle.omega[2]);
			double ang = axis_angle.theta;
			quat.setRotation(axis_rot, ang);
			transform.setRotation(quat);
			static tf::TransformBroadcaster br;
			tf::StampedTransform transform_endEffector;

			ros::Duration(0.01).sleep();
			try{
				listener.lookupTransform("/base_link", "/wrist",
						then, transform_endEffector);
				/* transform.setRotation(transform_endEffector.getRotation()); */
				if(abs(transform_endEffector.getOrigin().x() - x[0]) < 1e-5 && abs(transform_endEffector.getOrigin().y() - x[1]) < 1e-5 && abs(transform_endEffector.getOrigin().z() - x[2]) < 1e-5) {
					fk_check.data = true;
				}
				else
					fk_check.data = false;
			}
			catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
			}
			br.sendTransform(tf::StampedTransform(transform, then, "/base_link", "/wrist"));//fk_endEffector"));
			fkCheckPub.publish(fk_check);
		}
};
