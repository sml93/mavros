/**
 * @brief Odometry plugin
 * @file odom.cpp
 * @author James Goppert <james.goppert8@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2017 James Goppert
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <eigen_conversions/eigen_msg.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

namespace mavros {
namespace extra_plugins {
/**
 * @brief Odometry plugin
 *
 * Send odometry info
 * to FCU position and attitude estimators.
 *
 */
class OdometryPlugin : public plugin::PluginBase {
public:
	OdometryPlugin() : PluginBase(),
		_nh("~odometry"),
		_odom_sub(_nh.subscribe("odom", 10, &OdometryPlugin::odom_cb, this)),
		_tfBuffer(),
		_tfListener(_tfBuffer)
	{}

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		// tf params
		_odom_sub = _nh.subscribe("odom", 10, &OdometryPlugin::odom_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle _nh;
	ros::Subscriber _odom_sub;
 	tf2_ros::Buffer _tfBuffer;
	tf2_ros::TransformListener _tfListener;

	/* -*- callbacks -*- */

	void odom_cb(const nav_msgs::Odometry::ConstPtr &odom)
	{
		geometry_msgs::TransformStamped t_child_to_ned , t_ref_to_ned;
		try {
			t_child_to_ned = _tfBuffer.lookupTransform(
					"NED", odom->child_frame_id, ros::Time(0));
			t_ref_to_ned = _tfBuffer.lookupTransform(
					"NED", odom->header.frame_id, ros::Time(0));
		} catch (tf2::TransformException &ex) {
		  ROS_WARN("%s",ex.what());
		  ros::Duration(1.0).sleep();
		  return;
		}

		// position in ref frame
		Eigen::Vector3d pos_ref(
			odom->pose.pose.position.x,
			odom->pose.pose.position.y,
			odom->pose.pose.position.z);

		// velocity in child frame
		Eigen::Vector3d lin_vel_child(
			odom->twist.twist.linear.x,
			odom->twist.twist.linear.y,
			odom->twist.twist.linear.z);

		// angular velocity in child frame
		Eigen::Vector3d ang_vel_child(
			odom->twist.twist.angular.x,
			odom->twist.twist.angular.y,
			odom->twist.twist.angular.z);

		// quaternion in ENU
		Eigen::Quaterniond q_enu(
			odom->pose.pose.orientation.w,
			odom->pose.pose.orientation.x,
			odom->pose.pose.orientation.y,
			odom->pose.pose.orientation.z);

		// convert to NED
		Eigen::Vector3d pos_ned, lin_vel_ned, ang_vel_ned;
		Eigen::Quaterniond q_ned;
		tf2::doTransform(pos_ref, pos_ned, t_ref_to_ned);
		tf2::doTransform(lin_vel_child, lin_vel_ned, t_child_to_ned);
		tf2::doTransform(ang_vel_child, ang_vel_ned, t_child_to_ned);

		// apply frame transforms
		q_ned = ftf::transform_orientation_enu_ned(
			ftf::transform_orientation_baselink_aircraft(q_enu));

		uint64_t stamp = odom->header.stamp.toNSec() / 1e3;

		// send LOCAL_POSITION_NED_COV
		mavlink::common::msg::LOCAL_POSITION_NED_COV lpos {};

		lpos.time_usec = stamp;

		lpos.x = pos_ned.x();
		lpos.y = pos_ned.y();
		lpos.z = pos_ned.z();
		lpos.vx = lin_vel_ned.x();
		lpos.vy = lin_vel_ned.y();
		lpos.vz = lin_vel_ned.z();
		lpos.ax = 0.0;
		lpos.ay = 0.0;
		lpos.az = 0.0;
		// [[[end]]] (checksum: e8d5d7d2428935f24933f5321183cea9)

		// TODO: apply ftf::transform_frame(Covariance6d)
		size_t i = 0;
		for (int row = 0; row < 6; row++) {
			for (int col = row; col < 6; col++) {
				lpos.covariance[i] = odom->pose.covariance[row * 6 + col];
				i += 1;
			}
		}

		UAS_FCU(m_uas)->send_message_ignore_drop(lpos);

		// send ATTITUDE_QUATERNION_COV
		mavlink::common::msg::ATTITUDE_QUATERNION_COV att;

		att.time_usec = stamp;

		// [[[cog:
		// for a, b in zip("xyz", ('rollspeed', 'pitchspeed', 'yawspeed')):
		//     cog.outl("att.%s = ang_vel_ned.%s();" % (b, a))
		// ]]]
		att.rollspeed = ang_vel_ned.x();
		att.pitchspeed = ang_vel_ned.y();
		att.yawspeed = ang_vel_ned.z();
		// [[[end]]] (checksum: e100d5c18a64c243df616f342f712ca1)

		ftf::quaternion_to_mavlink(q_ned, att.q);

		// TODO: apply ftf::transform_frame(Covariance9d)
		for (size_t i = 0; i < 9; i++) {
			att.covariance[i] = odom->pose.covariance[i];
		}

		UAS_FCU(m_uas)->send_message_ignore_drop(att);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::OdometryPlugin, mavros::plugin::PluginBase)
