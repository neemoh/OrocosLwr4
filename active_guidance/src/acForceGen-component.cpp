//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2016, Nearlab


    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of nearlab nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <http://nearlab.polimi.it/>
    \author    Nima Enayati
    \version   -
*/
//==============================================================================

#include "acForceGen-component.hpp"


//-----------------------------------------------------------------------
// Constructor
//-----------------------------------------------------------------------
AcForceGen::AcForceGen(std::string const& name) : TaskContext(name)	{

	// not initializing the acs here because the properties cann be used from the configureHook onwards

	// add ports
	this->addPort("tool_pose", this->port_read_tool_pose).doc("Reading the current pose of the tool");
	this->addPort("tool_twist", this->port_read_twist).doc("Readings linear and angluar velocities of the tool");
	this->addEventPort("desired_pose", this->port_read_desired_pose).doc("Readings the desired pose of the tool");
	this->addPort("hapdev_switch", this->port_read_hapdev_switch).doc("Reading the state of the haptic device switch");
	this->addPort("ac_force", this->port_write_force).doc("Writing the generated active constraint");
	this->addPort("inputMasterToSlaveTr", this->mstr_to_slv_tr_port).doc("input camera to slave pose");

	// read properties
	this->addProperty("rot_master_to_slave", 	this->mstr_to_slv_rotation_prop).doc("Rigid transformation from master to robot base. Used when master to Cam rotation is not available");
	this->addProperty("rot_master_to_cam", 		this->mstr_to_cam_rotation_prop).doc("Rigid transformation from master to camera (display).");
	this->addProperty("time_period", 			this->period_prop).doc("The constant time period used for interpolation");

	// add operations
	this->addOperation("setACMode", 				&AcForceGen::setACMode, this).doc("set the active constraint method: 0: No ac, 1: Plastic, 2:Plastic with redirection, 3: vicouse with redirection, -1 elastic test");
	this->addOperation("setFMAX", 					&AcForceGen::setFMAX, this).doc("set the active constraint method");

	// initialization
	this->ac_mode = 0;
	this->FMAX = 5;
	this->penet	  = KDL::Vector(0.0, 0.0, 0.0);


	std::cout << "AcForceGen constructed !" <<std::endl;
}

//-----------------------------------------------------------------------
// Configure hook
//-----------------------------------------------------------------------
bool AcForceGen::configureHook(){
	RTT::Logger::In in(this->getName());

	// initializing the output port
	this->port_write_force.setDataSample(this->wrench_out);

	// properties
	this->mstr_to_slv_rotation_backup	= KDL::Rotation::Quaternion(this->mstr_to_slv_rotation_prop[0], this->mstr_to_slv_rotation_prop[1], this->mstr_to_slv_rotation_prop[2], this->mstr_to_slv_rotation_prop[3]);
	this->mstr_to_cam_rotation			= KDL::Rotation::Quaternion(this->mstr_to_cam_rotation_prop[0], this->mstr_to_cam_rotation_prop[1], this->mstr_to_cam_rotation_prop[2], this->mstr_to_cam_rotation_prop[3]);


	// Viscous with redirection (f_max, elastic_lenth, boundary_thresh)
	this->ac_p_ptr = new acPlast(this->FMAX, 0.003, 0.0);

	// plastic with redirection (f_max, elastic_lenth, boundary_thresh)
	this->ac_pr_ptr = new acPlastRedirect(this->FMAX, 0.003, 0.002);

	// Viscous with redirection (f_max, b_max, boundary_thresh)
	this->ac_vr_ptr = new acViscousRedirect(this->FMAX, 60, 0.004) ;

	//elastic constraint (f_max, k, b, dt)
	this->ac_e_ptr  = new acElastic(this->FMAX/2, 200, 20, this->period_prop);

	log(RTT::Info) << "AcForceGen configured!" << RTT::endlog();
	return true;
}

//-----------------------------------------------------------------------
// startHook
//-----------------------------------------------------------------------
bool AcForceGen::startHook(){

	if(this->isRunning())
		return false;

	this->initializeMasterToSlaveTransformation();

	log(RTT::Info) << "AcForceGen started!" << RTT::endlog();
	return true;
}

//-----------------------------------------------------------------------
// startHook
//-----------------------------------------------------------------------
void AcForceGen::updateHook(){

	// Reading data. Not checking for RTT::NewData
	this->port_read_tool_pose.read(this->current_pose);
	poseMsgToPositionKDLVec(this->current_pose, this->current_pos);

	// Reading the desired tool pose
	this->port_read_desired_pose.read(this->desired_pose);
	poseMsgToPositionKDLVec(this->desired_pose, this->desired_pos);

	// Reading the tool twist
	this->port_read_twist.read(this->current_twist);
	this->current_vel[0] = this->current_twist.linear.x;
	this->current_vel[1] = this->current_twist.linear.y;
	this->current_vel[2] = this->current_twist.linear.z;

	// updating the transformation based on the camera (if topic available and new message published)
	this->updateMasterToSlaveTransformation();

	// Dealing only with the positions, the deference between the current and desired positions
	// is called penetration vector.
	this->penet = this->desired_pos -this->current_pos;

	// generating the force based on the selected method
	switch(ac_mode){
	case 0:
		// no constraint. Send zero forces.
		ac_force[0] = 0.0;
		ac_force[1] = 0.0;
		ac_force[2] = 0.0;
		break;
	case 1:
		this->ac_p_ptr->getForce(this->ac_force, this->current_pos, this->desired_pos,this->current_vel);

		break;
	case 2:
		this->ac_pr_ptr->getForce(this->ac_force, this->current_pos, this->desired_pos,this->current_vel);

		break;
	case 3:
		this->ac_vr_ptr->getForce(this->ac_force, this->current_pos, this->desired_pos,this->current_vel);
		break;
	case -1:
		this->ac_e_ptr->getForce(this->ac_force, this->current_pos, this->desired_pos,this->current_vel);
		break;

	default:
		// Value of szChEntered undefined.
		std::cout << "ac_mode " <<ac_mode <<  " not defined" << std::endl;
		break;
	}

	// writing the wrench on the port
	this->ac_force_master_ref = this->mstr_to_slv_rotation.Inverse() * this->ac_force;

	this->wrench_out.force.x = this->ac_force_master_ref[0];
	this->wrench_out.force.y = this->ac_force_master_ref[1];
	this->wrench_out.force.z = this->ac_force_master_ref[2];
	this->port_write_force.write(this->wrench_out);

}

//-----------------------------------------------------------------------
// startHook
//-----------------------------------------------------------------------
void AcForceGen::stopHook() {
	log(RTT::Info) << "AcForceGen executes stopping!" << RTT::endlog();
}

//-----------------------------------------------------------------------
// startHook
//-----------------------------------------------------------------------
void AcForceGen::cleanupHook() {

	// Deleting the variables on the free store
	delete this->ac_p_ptr;
	delete this->ac_pr_ptr;
	delete this->ac_vr_ptr;
	log(RTT::Info) << "AcForceGen cleaning up!" << RTT::endlog();

}




//-----------------------------------------------------------------------
// Other Functions
//-----------------------------------------------------------------------


//-----------------------------------------------------------------------
// A simple elastic ac for test
//-----------------------------------------------------------------------
void ac_elastic_force_generation(KDL::Vector &f_out, const KDL::Vector & penet, const KDL::Vector & vel){
	double f_, dx, dy, dz, dis;
	double v_norm = vel.Norm();

	dx = penet[0];
	dy = penet[1];
	dz = penet[2];
	dis = sqrt(dx*dx + dy*dy + dz*dz);
	f_ = 200 * dis - 50 * v_norm;
	if (f_ >2) f_ = 2;


	f_out[0]  = f_ * dx/dis;
	f_out[1]  = f_ * dy/dis;
	f_out[2]  = f_ * dz/dis;

}

//-----------------------------------------------------------------------
// converting position of pose message to a KDL vec
//-----------------------------------------------------------------------

void poseMsgToPositionKDLVec( geometry_msgs::Pose pose_in, KDL::Vector &vec_out){
	vec_out[0] = pose_in.position.x;
	vec_out[1] = pose_in.position.y;
	vec_out[2] = pose_in.position.z;
}

//-----------------------------------------------------------------------
// changing the ac method in the deployer
//-----------------------------------------------------------------------
void AcForceGen::setACMode(int m){
	// Setting the ac type
	this->ac_mode = m;
}

//-----------------------------------------------------------------------
// Setting the maximum generated force
//-----------------------------------------------------------------------
void AcForceGen::setFMAX(double in){

	if(in <0 || in>15){
		std::cout << "The max force must be between 0 and 15 N." << std::endl;
	}
	else{
		this->FMAX = in;
		// setting the FMAX of all ec objects
		ac_p_ptr->setFmax(this->FMAX);
		ac_pr_ptr->setFmax(this->FMAX);
		ac_vr_ptr->setFmax(this->FMAX);
	}
}


void AcForceGen::initializeMasterToSlaveTransformation(){

	RTT::Logger::In in(this->getName());

	geometry_msgs::Quaternion pose_in;

	// reading the cam to slave data if available
	if (this->mstr_to_slv_tr_port.connected() && this->mstr_to_slv_tr_port.read(pose_in) != RTT::NoData ){

		mstr_to_slv_rotation = KDL::Rotation::Quaternion(pose_in.x, pose_in.y, pose_in.z, pose_in.w);
		// notify the user
		log(RTT::Info)<< " Updated the camera to slave transformation." << RTT::endlog();

	}
	else{
		log(RTT::Warning)<< "Port " << this->mstr_to_slv_tr_port.getName() << " is not connected. Using backup cam to master rotation" << RTT::endlog();
		mstr_to_slv_rotation = mstr_to_slv_rotation_backup;
	}

}


void AcForceGen::updateMasterToSlaveTransformation(){

	RTT::Logger::In in(this->getName());

	geometry_msgs::Quaternion pose_in;

	// reading the cam to slave data if available
	if (this->mstr_to_slv_tr_port.connected() && this->mstr_to_slv_tr_port.read(pose_in) == RTT::NewData ){

		mstr_to_slv_rotation = KDL::Rotation::Quaternion(pose_in.x, pose_in.y, pose_in.z, pose_in.w);
		// notify the user
		log(RTT::Info)<< " Updated the master to slave transformation." << RTT::endlog();

	}


}
/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(AcForceGen)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(AcForceGen)
