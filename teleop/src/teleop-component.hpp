#ifndef OROCOS_TELEOP_COMPONENT_HPP
#define OROCOS_TELEOP_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include "MyAPI/LWR4_Kinematics.hpp"
#include "MyAPI/toolbox.hpp"
#include "MyAPI/interpolator.cpp"

#include <cmath>
//#include <math.h>

#include <kuka_lwr_fri/friComm.h>
#include <kuka_lwr_fri/typekit/Types.hpp>
#include <lwr_fri/typekit/Types.hpp>

struct filter {
	double inp1, inp2, inp3;
	double out1, out2, out3;
};

struct stateVar{
	std::vector<double> q_curr, q_init, q_last, q_dest, q_cmd, q_cmd_last;
	std::vector<double> v_curr;
	std::vector<double> q_min, q_max, v_max, a_max;
};

class Teleop : public RTT::TaskContext{

public:
	Teleop(std::string const& name);
	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

	bool goHome();
	void wtf();
	void switchForceFeedback(const bool);
	void forceFilterSwitch();
	void forceSensorCalib();
	bool startMotion();
	bool stopMotion();
	bool changeMotionMode(const int mode);
	bool setPTPCartDestination(const std::vector<double>& values);
	bool setPTPJointDestination(const std::vector<double> values);
	void orientationCoupling(const bool);
	void positionCoupling(const bool);

	bool trackingSupervisor(std::vector<double>& q_cmd, const std::vector<double> q_dest, const std::vector<double> q_curr,
			const std::vector<double> qdot_curr, const std::vector<double> qdot_max,  const std::vector<double> qdotdot_max,  double dt);
	bool P2PInterpolator(const std::vector<double> p_dest, const std::vector<double> p_init,
			std::vector<double>& p_out, const std::vector<double> v_max, const std::vector<double> a_max,
			std::vector<double>& h_cart, double& T_cart, unsigned int& interp_counter, bool& new_cart_dest ,  bool& dest_reached);

//	bool calculateFK(std::vector<double> jointValues, unsigned int & rconfiguration, double & nsparam, KDL::Frame &targetmatrix, KDL::Vector& elbow_tangent);
	bool calculateIK(KDL::Frame targetmatrix, const unsigned int rconfiguration, double nsparam, std::vector<double>& jointValues);
//	bool LWR4_IK(KDL::Frame targetmatrix, const unsigned int rconfiguration, double nsparam, double nztool, std::vector<double>& jointValues);

	bool clipToLimits(std::vector<double>& vars, std::vector<double> min_limits, std::vector<double> max_limits) ;

	void startPalpation();
	void stopPalpation();

private:

	bool motionOn;
	bool destination_reached;

	bool new_cart_dest;
	bool new_joint_dest;
	bool clutch_first_time;

//----------------------------------------- TEMP PALPATIOn
	std::vector<double> palp_cart_destination;
	bool palpation_on;
	bool palpation_first_time;
	int palp_point_status;

	//******************************************  Temporary variables
	motion_control_msgs::JointPositions tmp_joint_pos;
	geometry_msgs::Pose tmp_pose_msg;
	geometry_msgs::Vector3 tmp_vec3;
	std::vector<double> tmp_cart_vec, tmp_joint_vec;
	unsigned int mode4_counter;
	bool first_time_temp,dest_reached_temp, new_dest_temp;
	KDL::Frame targetmatrix;
	KDL::Frame initmatrix;
	double T_temp;
	vector<double> posit_init, posit_last, posit_dest, posit_cmd, h_temp, v_last, v_cmd;
	unsigned int interp_counter_temp;


	unsigned int temp_config;
	vector<double> psi_last,psi_v_cmd,psi_v_last;

	//******************************************  PTP and Interpolation variables
	unsigned int num_cart_p_var, num_cart_var;
	unsigned int num_joints;
	unsigned int num_axes;
	unsigned int motion_mode;

	stateVar slv_jnt, slv_cart;
	std::vector<double> cart_6d_last, v_cart_6d_last;


	std::vector<double> slv_jnt_home;
	std::vector<double> slv_joint_p_interpd_init, slv_joint_p_interpd;
	std::vector<double> h_joint, a1, a2 ,a3; 							// Variables of PTP interpolation
	double T_joint;

	RTT::os::TimeService::ticks time_init, time_last, time_last2;
	double dt, dt_loop_msrd, dt_computation; 											//dt is set as a property, dt_real is estimated at each cycle.
	unsigned int dt_counter;
	unsigned int interp_counter;										//Counter is used for PTP interpolator instead of dt_real

	//******************************************  Tracking and teleop variables
	bool force_feedback_on, force_bias_computed, force_filter_on, teleop_pos_coupled,teleop_ori_coupled;
	double force_scale, transl_scale;
	double first_engagement_counter_rpy, first_engagement_counter_pos;
	double rpy_avg_n, rpy_avg_n_variable;
	double pos_avg_n, pos_avg_n_variable;
	std::vector<double> mstr_rpy_avg, mstr_deltapos_avg;

	std::vector<double> mstr_pose_curr, mstr_pose_curr_filt, mstr_pose_init, mstr_dp;

	std_msgs::Int8 master_clutch, tmp_button;
	KDL::Frame mstr_frame_curr, mstr_frame_init, slv_kdl_cmd , slv_frame_curr, slv_frame_init, slv_frame_dest;
	filter pos_filt[3], force_filt[3];

	unsigned int force_bias_counter;
	std::vector<double> force_bias, force_filtered;
	geometry_msgs::Wrench tmp_wrench,  mstr_wrench_cmd;
	KDL::Frame fs_to_ee_frame, mstr_to_slv_frame, master_to_tool_orient_frame;

	//***************************** Kinematics********************************
	std::vector<std::vector<double> > dh;
	double limbs[4];
	double tool_zlength;
	unsigned int  robot_config;
	std::vector<double> ns_param;
	KDL::Frame tmp_frame;
	KDL::Frame lastMatrix;
	motion_control_msgs::JointPositions lwr_joint_msg_command;
	sensor_msgs::JointState tmp_joint_state;



protected:
	bool force_feedback_on_prop;
	bool force_filter_on_prop;
	bool teleop_pos_coupled_prop, teleop_ori_coupled_prop;
	double force_scale_prop;
	double transl_scale_prop;
	double tool_zlength_prop;
	double rpy_avg_n_prop;
	double pos_avg_n_prop;
	unsigned int motion_mode_prop;
	std::vector<double> slv_cart_q_max_prop, slv_cart_q_min_prop;					// Vector with the minimum value of each variable
	std::vector<double> slv_cart_v_max_prop, slv_cart_a_max_prop;					// Vector with the maximum acceleration of each variable
	std::vector<double> slv_jnt_q_max_prop,slv_jnt_q_min_prop;
	std::vector<double> slv_jnt_v_max_prop, slv_jnt_a_max_prop;					// Vector with the maximum acceleration of each variable
	std::vector<double> slv_jnt_home_prop;
	std::vector<double> master_to_base_frame_prop, fs_to_ee_frame_prop, master_to_tool_orient_frame_prop;
	std::vector<double> palpation_init_6dpose_prop;

	double period_prop;
	RTT::InputPort<geometry_msgs::Pose> 					cart_dest_port;		// DataPort containing the current pose
	RTT::InputPort<std_msgs::Int8> 							master_button_port;
	RTT::InputPort<sensor_msgs::JointState> 				joint_curr_port;		// DataPort containing the stamped pose
	RTT::InputPort<geometry_msgs::Wrench>					force_from_slave_port;
	RTT::OutputPort<motion_control_msgs::JointPositions>	joint_command_port;		// DataPort containing the stamped pose
	RTT::OutputPort<geometry_msgs::Vector3> 				joint_vel_port;
	RTT::OutputPort<geometry_msgs::Pose> 					cart_command_port;
	RTT::OutputPort<geometry_msgs::Pose> 					cart_FK_port;
	RTT::OutputPort<geometry_msgs::Wrench>					force_to_master_port;
//	RTT::InputPort<tFriKrlData> 							port_from_krl_master;
//	RTT::OutputPort<tFriKrlData> 							port_to_krl_master;

	RTT::InputPort<geometry_msgs::Pose> 					slave_cart_port;

};

#endif

