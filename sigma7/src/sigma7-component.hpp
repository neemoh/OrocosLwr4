#ifndef OROCOS_sigma7_COMPONENT_HPP
#define OROCOS_sigma7_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#include <std_msgs/typekit/Types.hpp>
#include <geometry_msgs/typekit/Types.hpp>
#include <sensor_msgs/typekit/Types.hpp>

#include </usr/include/dhdc.h>
#include </usr/include/drdc.h>
#include <kdl/frames.hpp>
#include <tf_conversions/tf_kdl.h>
#include <kdl/kdl.hpp>

using namespace RTT;

class sigma7: public RTT::TaskContext {
	public:
		sigma7(std::string const& name);
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
		bool initSigma();

		void getFrame( double pos_array[],  double orientation_matrix[][3], KDL::Frame& frame);
		void usePedal(bool);
		void lockOrientation();
	private:

//		char ID;

		KDL::Frame HI_Frame;
		geometry_msgs::Pose HI_pose;
		geometry_msgs::Twist HI_twist;
		std_msgs::Int8 sigma_button_state;
		std_msgs::Int8 sigma_button_previous_state;
		std_msgs::Int8 sigma_pedal_state;
		std_msgs::Int8 sigma_pedal_previous_state;
		geometry_msgs::Wrench tmp_wrench, mstr_wrench_cmd;
		bool use_pedal;
		double lock_orientation[3];
		KDL::Frame mstr_to_slv_frame,master_to_tool_orient_frame;

	protected:
		std::vector<double> trackParam_prop;

		std::vector<double> master_to_base_frame_prop, master_to_tool_orient_frame_prop;

		RTT::OutputPort<geometry_msgs::Pose> master_pose_port;
		RTT::OutputPort<geometry_msgs::Twist> master_twist_port;
		RTT::InputPort<geometry_msgs::Pose> slave_pose_port;
		RTT::InputPort<sensor_msgs::JointState> trigger;
		RTT::OutputPort<std_msgs::Int8> botton_port;
		RTT::OutputPort<std_msgs::Int8> pedal_port;
		RTT::InputPort<geometry_msgs::Wrench> force_to_master_port;

};
#endif
