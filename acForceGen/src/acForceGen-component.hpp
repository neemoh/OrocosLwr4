#ifndef OROCOS_ACFORCEGEN_COMPONENT_HPP
#define OROCOS_ACFORCEGEN_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <kdl/frames.hpp>
#include <geometry_msgs/typekit/Types.hpp>
#include <std_msgs/typekit/Types.hpp>
#include "acMethods.hpp"

//-----------------------------------------------------------------------
// The definition of the component class
//-----------------------------------------------------------------------
class AcForceGen : public RTT::TaskContext{
public:
	// constructor
	AcForceGen(std::string const& name);

	// orocos main functions
	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook();


	// functions for orocos deployer operations
	void setACMode(int m);
	void setFMAX(double in);
	//--------------------------------------------------------------------------------------------------
	// setCam2SlaveRotation
	//--------------------------------------------------------------------------------------------------
	// set the rotation from camera to slave.
	void initializeMasterToSlaveTransformation();
	void updateMasterToSlaveTransformation();
	// a pointer of the base class type for each method
	ac * ac_p_ptr;
	ac * ac_pr_ptr;
	ac * ac_vr_ptr;
	ac * ac_e_ptr;

private:
	// some used variables
	int ac_mode;
	double FMAX;
	geometry_msgs::Wrench wrench_out;
	geometry_msgs::Pose current_pose, desired_pose;
	geometry_msgs::Twist current_twist;
	KDL::Vector current_pos, desired_pos, penet;
	KDL::Vector ac_force;
	KDL::Vector ac_force_master_ref;
	KDL::Vector current_vel;

	KDL::Rotation mstr_to_slv_rotation;
	KDL::Rotation mstr_to_cam_rotation;
	KDL::Rotation cam_to_slv_rotation;
	KDL::Rotation mstr_to_slv_rotation_backup;
protected:
	// properties
	// this frame is used if no camera rotation is provided
	std::vector<double> mstr_to_slv_rotation_prop;
	std::vector<double> mstr_to_cam_rotation_prop;
	double period_prop;

	// defining the ports
	RTT::InputPort<geometry_msgs::Pose> 		port_read_tool_pose;
	RTT::InputPort<geometry_msgs::Twist> 		port_read_twist;
	RTT::InputPort<geometry_msgs::Pose> 		port_read_desired_pose;
	RTT::InputPort<std_msgs::Int8> 				port_read_hapdev_switch;
	RTT::InputPort<geometry_msgs::Quaternion> 	mstr_to_slv_tr_port;


	RTT::OutputPort<geometry_msgs::Wrench> 		port_write_force;

};

//-----------------------------------------------------------------------
// Other functions
//-----------------------------------------------------------------------
// a conversion function
void poseMsgToPositionKDLVec( geometry_msgs::Pose pose_in, KDL::Vector &vec_out);
// a simple test elastic method
void ac_elastic_force_generation(KDL::Vector &f_out, const KDL::Vector & penet, const KDL::Vector & vel);




#endif
