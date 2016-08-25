#ifndef OROCOS_ACFORCEGEN_COMPONENT_HPP
#define OROCOS_ACFORCEGEN_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <kdl/frames.hpp>
#include <geometry_msgs/typekit/Types.hpp>
#include <std_msgs/typekit/Types.hpp>
#include "acMethods.hpp"
#include <active_guidance/skillProbabilities.h>



//-----------------------------------------------------------------------
// Isotropic viscosity class
//-----------------------------------------------------------------------
class isotropicViscosity{
public:
	isotropicViscosity(double _FMAX, double _B){
		B = _B;
		FMAX = _FMAX;
	}

	void setParam(double _FMAX, double _B){	B = _B;	FMAX = _FMAX;}
	void setFMAX(double _FMAX){	FMAX = _FMAX;}

	KDL::Vector getViscouseForce(const KDL::Vector _vel);

	// saturates the norm of _f at FMAX
	KDL::Vector saturateForce(const KDL::Vector _f);

private:
	double B;
	double FMAX;

};



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
	void setAcFMAX(double in);
	void setIsoViscParams(double _FMAX, double _B);
	void setPenetAvoidParams(double _FMAX, double _B, double _K);

	void setAllMaxForces(const double FMAX_ac, const double FMAX_PenetAvoid, const double FMAX_isovisc);
	void setParametersFromSkillMetrics(const active_guidance::skillProbabilities &);
	//--------------------------------------------------------------------------------------------------
	// assuming that the tissue is parallel to the z plane, provides an elastic force to prevent
	// penetrating too much. If such assumption cannot be made, the normal to the tissue surface at each
	// point is needed.
	KDL::Vector getPenetAvoidanceForce(const KDL::Vector & current_pos ,
			const  KDL::Vector & desired_pos, const KDL::Vector & _vel);

	//--------------------------------------------------------------------------------------------------
	// set the rotation from camera to slave.
	void initializeMasterToSlaveTransformation();
	void updateMasterToSlaveTransformation();

	// a pointer of the base class type for each method
	ac * ac_p_ptr;
	ac * ac_pr_ptr;
	ac * ac_vr_ptr;
	ac * ac_e_ptr;
	isotropicViscosity * iso_visc;
private:
	// some used variables
	int ac_mode;
	double acFMAX;

	// penet avoidance
	double FMAX_penet_avoid;
	double K_penet_avoid;
	double B_penet_avoid;


	geometry_msgs::Wrench wrench_out;
	geometry_msgs::Pose current_pose, desired_pose;
	geometry_msgs::Twist current_twist;

	KDL::Vector current_pos, desired_pos;
	KDL::Vector ac_force;
	KDL::Vector assist_force_2_visc;
	KDL::Vector assist_force_3_penet_avoid;
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
	std::vector<double> iso_visc_params;
	std::vector<double> penet_avoid_params;

	double period_prop;
	double isoViscB_param;
	double isoViscFMAX_param;
	double acFMAX_param;

	// defining the ports
	RTT::InputPort<geometry_msgs::Pose> 		port_inp_tool_pose;
	RTT::InputPort<geometry_msgs::Twist> 		port_inp_twist;
	RTT::InputPort<geometry_msgs::Pose> 		port_inp_desired_pose;
	RTT::InputPort<std_msgs::Int8> 				port_inp_hapdev_switch;
	RTT::InputPort<geometry_msgs::Quaternion> 	port_inp_mstr_to_slv_tr;
	RTT::InputPort<active_guidance::skillProbabilities> port_inp_skill_probs;

	RTT::OutputPort<geometry_msgs::Wrench> 		port_out_force;

};

//-----------------------------------------------------------------------
// Other functions
//-----------------------------------------------------------------------
// a conversion function
void poseMsgToPositionKDLVec( geometry_msgs::Pose pose_in, KDL::Vector &vec_out);
// a simple test elastic method
void ac_elastic_force_generation(KDL::Vector &f_out, const KDL::Vector & penet, const KDL::Vector & vel);




#endif
