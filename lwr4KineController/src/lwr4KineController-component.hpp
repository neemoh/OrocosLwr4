#ifndef OROCOS_LWR4KINECONTROLLER_COMPONENT_HPP
#define OROCOS_LWR4KINECONTROLLER_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include "MyAPI/LWR4_Kinematics.hpp"
#include "MyAPI/toolbox.hpp"
#include "MyAPI/interpolator.cpp"
#include <std_msgs/Bool.h>

//#include "MyAPI/filter.h"

#include <cmath>
//#include <math.h>

#include <kuka_lwr_fri/friComm.h>
#include <kuka_lwr_fri/typekit/Types.hpp>
#include <lwr_fri/typekit/Types.hpp>
#include <active_guidance/skillProbabilities.h>

struct filter {
	double inp1, inp2, inp3;
	double out1, out2, out3;
};

struct stateVar{
	std::vector<double> q_curr,  q_last, q_dest, q_cmd, q_cmd_last;
	std::vector<double> v_curr, v_last;
	std::vector<double> q_min, q_max, v_max, a_max;
};

// Forward Declaration
class lwr4KineController;

//-------------------------------------------------------------------------------------
// CLASS: freqObserver
//-------------------------------------------------------------------------------------
class freqObserver{
public:
	freqObserver(double _dt_param, unsigned int _dt_init_steps,
			unsigned int _avg_steps);
	void check();

	// is the constant parameter
	double dt_param;
	// the measured loop time
	double dt_msrd_last;
	// the averaged measured loop time
	double dt_msrd_avg;
	//computation time
	double t_computation;
	//
	unsigned int avg_steps;
	// initialization counter
	unsigned int dt_init_counter;

	unsigned int dt_init_steps;

	// time initial variables
	RTT::os::TimeService::ticks loop_time_from, computation_time_from;

};



//-------------------------------------------------------------------------------------
// CLASS: PTPINTERPOLATOR
//-------------------------------------------------------------------------------------
// A 5th order polynomial (or 3rd order) interpolator.
// All variables arrive at the same time
class ptpInterpolator {

public:
	ptpInterpolator(
			const std::vector<double> _v_max,
			const std::vector<double> _a_max,
			const double _dt_param);

	bool setDestination(
			const std::vector<double> _p_dest,
			const std::vector<double> _p_curr);


	bool getNextCommand(
			std::vector<double> & p_interpd,
			bool & dest_reached);

private:
	// max velocity and acceleration
	std::vector<double> v_max;
	std::vector<double> a_max;

	// initial point
	std::vector<double> p_init;
	std::vector<double> p_dest;

	// interpolation variables
	std::vector<double> h;
	std::vector<double> a1;
	std::vector<double> a2;
	std::vector<double> a3;


	unsigned int num_elements;
	unsigned int degree;
	double T;
	double dt_param;
	unsigned int counter;
	bool dest_set;
	bool dest_reached;

};



//-------------------------------------------------------------------------------------
// CLASS: tele-operation
//-------------------------------------------------------------------------------------
class teleop{
public:

	//-------------------------------------------------------------------------------------
	// Constructor
	//-------------------------------------------------------------------------------------
	teleop(double _period,
			double _transl_scale,
			bool _teleop_ori_coupled,
			bool _teleop_pos_coupled,
			double _rpy_avg_n,
			double _pos_avg_n,
			double _force_scale,
			bool _force_feedback_on,
			std::vector<double> _mstr_to_slv_backup_rotation,
			std::vector<double> _fs_to_ee_frame,
			std::vector<double> _master_to_tool_orient_prop,
			std::vector<double> _mstr_to_cam_rotation_prop);


	void initializeOrientation(lwr4KineController * tel, ptpInterpolator * cart_interpolator);

	//--------------------------------------------------------------------------------------------------
	// calculate Force Bias
	//--------------------------------------------------------------------------------------------------
	// Very basic estimation of the current bias of the force sensor by averaging 100 consecutive
	// samples. Naturally assumes no external force is applied during the calibration.
	void calculateForceBias(geometry_msgs::Wrench wrench_msrd);

	//--------------------------------------------------------------------------------------------------
	// reset Force Bias
	//--------------------------------------------------------------------------------------------------
	// simply resets the Bias so that on the next run the calculateForceBias method is engaged
	void resetForceBias();

	//--------------------------------------------------------------------------------------------------
	// get Force Feedback
	//--------------------------------------------------------------------------------------------------
	// This method scales and transforms the measured force
	// TODO: add tool mass to remove it's weight based on the orientation of the end-effector
	geometry_msgs::Wrench getForceFeedback(KDL::Frame robot_pose, geometry_msgs::Wrench wrench_msrd);

	//--------------------------------------------------------------------------------------------------
	// calculate Desired Slave Pose
	//--------------------------------------------------------------------------------------------------
	// This method finds the displacement of the master from the moment the clutch is engaged. It then
	// scales, filters, and transforms the displacement to be sent to the slave.
	// Sometimes in the period that the clutch is released and engaged orientation of the master changes
	// which can create jumps in the slave trajectory. To improve this, a larger window of averaging is
	// used when the clutch is pressed, the averaging window then reduces to a small value while the
	// clutch is kept engaged.
	KDL::Frame calculateDesiredSlavePose(KDL::Frame slv_frame_curr, geometry_msgs::Pose master_msrd_pose);

	//--------------------------------------------------------------------------------------------------
	// Sigma Workspace Check
	//--------------------------------------------------------------------------------------------------
	// Checks whether the master's end-effector is (approximately) close to sigma's workspace
	bool isCloseToSigmaWorkSpaceBoundary(const KDL::Vector& _position);

	//--------------------------------------------------------------------------------------------------
	// Print Parameters
	//--------------------------------------------------------------------------------------------------
	// print some parameters for debug
	void printParameters();

	//--------------------------------------------------------------------------------------------------
	// Coupling switches
	//--------------------------------------------------------------------------------------------------
	// turn the position or orientation coupling on and off.
	void switchPositionCoupling(const bool input);
	void switchOrientationCoupling(const bool input);

	//--------------------------------------------------------------------------------------------------
	// setCam2SlaveRotation
	//--------------------------------------------------------------------------------------------------
	// set the rotation from camera to slave.
	void setCam2SlaveRotation(KDL::Rotation in);



	//--------------------------------------------------------------------------------------------------
	// setCam2SlaveRotation
	//--------------------------------------------------------------------------------------------------
	// set the rotation from camera to slave.
	void mstrToSlave(const KDL::Frame & _in_mstr, KDL::Frame & _in_slave );

	//--------------------------------------------------------------------------------------------------
	// changing the averaging window for orientation
	//--------------------------------------------------------------------------------------------------
	void setOrientationAvgWindow(unsigned int in){	rpy_avg_n = in;};

public:
	// force related variables
	bool force_bias_computed;
	bool force_feedback_on;
	unsigned int force_bias_counter;
	double force_scale;
	KDL::Vector force_bias;

	// transformations
	KDL::Rotation mstr_to_slv_rotation;
	KDL::Rotation mstr_to_slv_rotation_backup;
	KDL::Rotation fs_to_ee_rotation;
	KDL::Rotation mstr_to_tool_orient_rotation;
	KDL::Rotation mstr_to_cam_rotation;
	KDL::Rotation slv_initial_orientation;
	KDL::Frame sigma_workspace_tr;
	// teleop
	bool 	clutch_first_time;
	bool 	teleop_pos_coupled,teleop_ori_coupled;
	bool 	available_cam_pose;
	bool 	reorientation_done;
	bool 	reorientation_first_run;
	double 	transl_scale;

	KDL::Frame mstr_frame_curr, mstr_frame_init;
	KDL::Frame slv_frame_init, slv_frame_dest;

private:
	double dt_param;

	KDL::Rotation cam_to_slv_rotation;

	// engagement counter
	unsigned int first_engagement_counter_rpy, first_engagement_counter_pos;

	// averaging number of samples
	unsigned int rpy_avg_n;
	unsigned int pos_avg_n;

	// clutch engagement counter
	unsigned int rpy_avg_n_variable;
	unsigned int pos_avg_n_variable;

	std::vector<double> mstr_rpy_avg;
	std::vector<double> mstr_deltapos_avg;

};

//-------------------------------------------------------------------------------------
// CLASS: lwr4KineController OROCOS COMPONENT
//-------------------------------------------------------------------------------------
class lwr4KineController : public RTT::TaskContext{

public:
	lwr4KineController(std::string const& name);

	// orocos main methods
	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

	//-------------------------------------------------------------------------------------
	// Change motion_mode
	//-------------------------------------------------------------------------------------
	// changes the mode of the component. Available modes are:
	// mode_name = 1 : Cartesian and Joint PTP with joint trajectory. Destination set in Deployer.
	// mode_name = 2 : Cartesian PTP with Cartesian trajectory. Destination set in Deployer.
	// mode_name = 3 : Tracking. Destination coming from outside.
	// mode_name = 4 : Tele-operation. Destination coming from hapatic device.
	bool changeMotionMode(const int mode);


	//-------------------------------------------------------------------------------------
	// Cartesian point to point motion.  -  motion_mode = 1 or 2
	//-------------------------------------------------------------------------------------
	// can be used from the deployer to set a Cartesian destination. If motion_mode is 1
	// the trajectory will be generated in joint space.In contrast, if motion_mode is 2
	// the trajectory will be in Cartesian (very basic generation, must be improved...)
	// examples:
	// setPTPCartPosDestination(array(-0.45, -0.37, 0.35)) sets a Cartesian position
	// destination.and the orientation is kept constant.
	// setPTPCartPosDestination(array(-0.45, 0.17, 0.45, 170.0, 0.0, 80.0)) sets a
	// Cartesian position(x, y, z) and orientation (roll pitch yaw) destination.
	bool setPTPCartDestination(const std::vector<double>& values);


	//-------------------------------------------------------------------------------------
	// Joint point to point motion  -   motion_mode = 2
	//-------------------------------------------------------------------------------------
	// For setting a joint destination. The trajectory is polynomial velocity.
	// example: setPTPJointDestination (array(0.1, 0.7, 0.5, -1.0, 0.5, 1.0, 0.9))
	bool setPTPJointDestination(const std::vector<double> values);

	//-------------------------------------------------------------------------------------
	// helper function for conversion and setting up of the cartesian interpolator
	//-------------------------------------------------------------------------------------
	void initializeCart6dTraj(const KDL::Frame & slv_frame_dest, const KDL::Frame &  slv_frame_curr, ptpInterpolator * _cart_interpolator);

	//-------------------------------------------------------------------------------------
	// Orientation and position coupling
	//-------------------------------------------------------------------------------------
	// Switch on or off the coupling between master and slave.
	// example: switchOrientationCoupling(1)
	void switchOrientationCoupling(const bool in){to->switchOrientationCoupling(in);}
	void switchPositionCoupling(const bool in){to->switchPositionCoupling(in);};

	//-------------------------------------------------------------------------------------
	// Master workspace helper
	//-------------------------------------------------------------------------------------
	// For very novice users, this helper brings the master device to its center when
	// the user reaches the boundaries of the master workspace. This works only when
	// the user is close to the boundary AND releases the clutch.
	void switchMasterWSHelper(const bool in){this->mstr_ws_helper_on = in;};


	//-------------------------------------------------------------------------------------
	// Changing the length of the averaging
	//-------------------------------------------------------------------------------------
	void setOrientationAvgWindow(const unsigned int in){	to->setOrientationAvgWindow(in); cout<<"OK!"<<endl;};

	bool clipToLimits(std::vector<double>& vars, std::vector<double> min_limits, std::vector<double> max_limits) ;

	//-------------------------------------------------------------------------------------
	// Best arm angle calculation
	//-------------------------------------------------------------------------------------
	// Finds the set of available arm angles from the current arm angle by staying away
	// from the limits. The commanded arm angle is then derived such that a smooth motion
	// toward the desired arm angle is generated.
	double calculateBestArmAngle();

	//-------------------------------------------------------------------------------------
	// Motion observer for tele operation
	//-------------------------------------------------------------------------------------
	// In tele-operation in three cases there may be a large displacement asked from the robot
	// which will make it lock that makes the debugging difficult.
	// 1: when one of the constant transformations is wrong
	// 2: when the master's end-effector rotates between the period of un-clutching and
	// clutching.
	// 3: and when the Ik gives a far away joint position for any reason
	// In any of these cases, When a large joint displacement is asked, this method will
	// engage and interpolates in joint space to prevent the robot from lockin. This
	// naturally causes the robot to deviate from the cartesian destination which in cases
	// can be quite dangerous. So if safety is important do not use this method.
	void jointStateMotionObserver(std::vector<double> q_curr, std::vector<double> q_dest, std::vector<double>& q_cmd );


	//-------------------------------------------------------------------------------------
	// Read and set camera to slave
	//-------------------------------------------------------------------------------------
	// Reads the camera to slave transformation from the corresponding port and
	// sets the value for the teleop object..
	void updateCam2SlavePose();

	//-------------------------------------------------------------------------------------
	// goHome
	//-------------------------------------------------------------------------------------
	// Send the robot to the home position through joint space trajectory.
	bool goHome(){return(setPTPJointDestination(slv_jnt_home));	};


	void readLwr4JointsAndDoFK();

	//-------------------------------------------------------------------------------------
	// WTF
	//-------------------------------------------------------------------------------------
	// Print some information about interesting things around the world!
	void wtf();

	void switchForceFeedback(const bool);

	void switchForceFilter(){
		cout<< "Switching force filter from:" << this->force_filter_on << " to: " << !this->force_filter_on << endl;
		this->force_filter_on = !this->force_filter_on;
	}

	void forceSensorCalib(){to->resetForceBias();};

	bool startMotion();
	bool stopMotion();

public:

	// frequency observer object pointer
	freqObserver * fo;
	teleop * to;
	ptpInterpolator * joint_interpolator;
	ptpInterpolator * cart_interpolator;



	// flags
	bool motionOn;
	bool destination_reached;
	bool teleop_interpolate_done;
	bool new_cart_dest;
	bool tool_reorientation_done;
	bool mstr_workspace_alert;
	bool mstr_ws_helper_on;

	// some integers!
	unsigned int motion_mode;
	unsigned int num_cart_p_var, num_cart_vars;
	unsigned int num_joints;

	// arm angles
	std::vector<double> psi_curr, psi_last, psi_v_cmd, psi_v_last;

	// master frames
	KDL::Frame mstr_frame_curr, mstr_frame_init;

	// slave sates
	stateVar slv_jnt, slv_cart;

	// slave frames
	KDL::Frame slv_frame_curr, slv_frame_init, slv_frame_dest, slv_frame_cmd, slv_frame_cmd_last;

	// slave 6d vectors [x, y, z, r, p, y]
//	std::vector<double> slv_cart_6d_dest, slv_cart_6d_curr;

	// slave home joint configuration. [j1, j2, j3, j4, j5, j6, j7]
	std::vector<double> slv_jnt_home;

	// jacobian
	KDL::Jacobian jacob;
	// temp
	std::vector<double> tmp_cart_vec, tmp_joint_vec;


//	// force sensor to end-effector rotation
//	KDL::Rotation fs_to_ee_rotation;
//
//	// If the port of cam to slave is not connected this value is used
//	KDL::Rotation mstr_to_slv_backup_rotation;
//
//	// rotating the tool orientation to a desired one
//	KDL::Rotation master_to_tool_orient_rotation;


	bool  force_filter_on;
	double transl_scale;
	double manipT,manipA,manipR;

	// messages
	std_msgs::Int8 master_clutch;
	motion_control_msgs::JointPositions tmp_joint_pos;
	geometry_msgs::Pose tmp_pose_msg;
	geometry_msgs::Twist tmp_twist;
	geometry_msgs::Wrench tmp_wrench,  mstr_wrench_cmd;
	std_msgs::Bool bool_msg;
	active_guidance::skillProbabilities skill_prob_msg;
	// kinematics
	LWR4Kinematics* kine;
	std::vector<std::vector<double> > dh;

	unsigned int  robot_config;
	KDL::Frame tmp_frame;
	sensor_msgs::JointState tmp_joint_state;



protected:
	// properties
	bool force_feedback_on_prop;
	bool force_filter_on_prop;
	bool teleop_pos_coupled_prop, teleop_ori_coupled_prop;
	bool mstr_ws_helper_prop;
	double period_prop;
	double force_scale_prop;
	double transl_scale_prop;
	double rpy_avg_n_prop;
	double pos_avg_n_prop;
	unsigned int motion_mode_prop;
	std::vector<double> slv_cart_q_max_prop, slv_cart_q_min_prop;					// Vector with the minimum value of each variable
	std::vector<double> slv_cart_v_max_prop, slv_cart_a_max_prop;					// Vector with the maximum acceleration of each variable
	std::vector<double> slv_jnt_q_max_prop,slv_jnt_q_min_prop;
	std::vector<double> slv_jnt_v_max_prop, slv_jnt_a_max_prop;					// Vector with the maximum acceleration of each variable
	std::vector<double> slv_jnt_home_prop;
	std::vector<double> mstr_to_slv_rotation_prop, mstr_to_cam_rotation_prop, fs_to_ee_rotation_prop, mstr_to_tool_orient_prop;
	std::vector<double> palpation_init_3dpose_prop,palpation_home_3dpose_prop;
	std::vector<double> tool_to_ee_tr_prop;


	RTT::InputPort<geometry_msgs::Pose> 					master_msrd_pose_port;		// DataPort containing the current pose
	RTT::InputPort<std_msgs::Int8> 							master_clutch_port;
	RTT::InputPort<sensor_msgs::JointState> 				joint_msrd_port;		// DataPort containing the stamped pose
	RTT::InputPort<geometry_msgs::Wrench>					force_from_slave_port;
	RTT::InputPort<geometry_msgs::Pose> 					cam_to_slave_pose_port;
	RTT::OutputPort<motion_control_msgs::JointPositions>	joint_command_port;		// DataPort containing the stamped pose
	RTT::OutputPort<geometry_msgs::Twist> 					port_cart_twist;
	RTT::OutputPort<geometry_msgs::Pose> 					cart_command_port;
	RTT::OutputPort<geometry_msgs::Quaternion> 				master_to_slave_rot_port;
	RTT::OutputPort<geometry_msgs::Pose> 					cart_FK_port;
	RTT::OutputPort<geometry_msgs::Wrench>					force_to_master_port;
	RTT::OutputPort<std_msgs::Bool>							port_out_mstr_ws_alert;

//	RTT::InputPort<tFriKrlData> 							port_from_krl_master;
//	RTT::OutputPort<tFriKrlData> 							port_to_krl_master;

	RTT::InputPort<geometry_msgs::Pose> 					slave_cart_port;
	RTT::InputPort<active_guidance::skillProbabilities>		skill_probs_port;

};


#endif

