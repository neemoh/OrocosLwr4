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

using namespace std;

//-----------------------------------------------------------------------
// Constructor
//-----------------------------------------------------------------------
AcForceGen::AcForceGen(std::string const& name) : TaskContext(name)	{

	// not initializing the acs here because the properties cann be used from the configureHook onwards

	// add ports
	this->addPort("inputToolPose", this->port_inp_tool_pose).doc("Reading the current pose of the tool");
	this->addPort("inputToolTwist", this->port_inp_twist).doc("Readings linear and angluar velocities of the tool");
	this->addEventPort("inputDesiredPose", this->port_inp_desired_pose).doc("Readings the desired pose of the tool");
	this->addPort("inputHapdevSwitch", this->port_inp_hapdev_switch).doc("Reading the state of the haptic device switch");
	this->addPort("inputAcForce", this->port_out_force).doc("Writing the generated active constraint");
	this->addPort("inputMasterToSlaveTr", this->port_inp_mstr_to_slv_tr).doc("input camera to slave pose");
	this->addPort("inputSkillProbs", this->port_inp_skill_probs).doc("input camera to slave pose");

	// read properties
	this->addProperty("rot_master_to_slave", 	this->mstr_to_slv_rotation_prop).doc("Rigid transformation from master to robot base. Used when master to Cam rotation is not available");
	this->addProperty("rot_master_to_cam", 		this->mstr_to_cam_rotation_prop).doc("Rigid transformation from master to camera (display).");
	this->addProperty("time_period", 			this->period_prop).doc("The constant time period used for interpolation");
	this->addProperty("acFMAX", 				this->acFMAX_param).doc("The constant time period used for interpolation");
	this->addProperty("isoViscParams",	 		this->iso_visc_params).doc("The constant time period used for interpolation");
	this->addProperty("penetAvoidParams",		this->penet_avoid_params).doc("The constant time period used for interpolation");

	// add operations
	this->addOperation("setACMode", 				&AcForceGen::setACMode, this).doc("set the active constraint method: 0: No ac, 1: Plastic, 2:Plastic with redirection, 3: vicouse with redirection, -1 elastic test");
	this->addOperation("setAcFMAX", 				&AcForceGen::setAcFMAX, this).doc("set the active constraint method");
	this->addOperation("setIsoViscosityParams", 		&AcForceGen::setIsoViscParams, this).doc("set the active constraint method");
	this->addOperation("setPenetAvoidParams", 			&AcForceGen::setPenetAvoidParams, this).doc("set the active constraint method");

	// initialization
	this->ac_mode = 0;
	this->acFMAX = 0.0;


	std::cout << "AcForceGen constructed !" <<std::endl;
}

//---------------------------------------------------------------------------------------------
// Configure hook
//--------------------------------------------------------------------------------------------
bool AcForceGen::configureHook(){
	RTT::Logger::In in(this->getName());

	if (this->iso_visc_params.size() != 2) {
		log(RTT::Error) << "Size of isoViscParams is not correct. I expect 2 elements but I got "<< this->iso_visc_params.size() << RTT::endlog();
		return false;
	}
	if (this->penet_avoid_params.size() != 3) {
		log(RTT::Error) << "Size of penetAvoidParams is not correct. I expect 3 elements but I got "<< this->penet_avoid_params.size() << RTT::endlog();
		return false;
	}

	// initializing the output port
	this->port_out_force.setDataSample(this->wrench_out);

	// properties
	this->mstr_to_slv_rotation_backup	= KDL::Rotation::Quaternion(this->mstr_to_slv_rotation_prop[0], this->mstr_to_slv_rotation_prop[1], this->mstr_to_slv_rotation_prop[2], this->mstr_to_slv_rotation_prop[3]);
	this->mstr_to_cam_rotation			= KDL::Rotation::Quaternion(this->mstr_to_cam_rotation_prop[0], this->mstr_to_cam_rotation_prop[1], this->mstr_to_cam_rotation_prop[2], this->mstr_to_cam_rotation_prop[3]);
	this->acFMAX 			= this->acFMAX_param;
	this->FMAX_penet_avoid 	= penet_avoid_params[0];
	this->K_penet_avoid 	= penet_avoid_params[1];
	this->B_penet_avoid 	= penet_avoid_params[2];

	// Viscous with redirection (f_max, elastic_lenth, boundary_thresh)
	this->ac_p_ptr = new acPlast(this->acFMAX, 0.003, 0.0);

	// plastic with redirection (f_max, elastic_lenth, boundary_thresh)
	this->ac_pr_ptr = new acPlastRedirect(this->acFMAX, 0.003, 0.002);

	// Viscous with redirection (f_max, b_max, boundary_thresh)
	this->ac_vr_ptr = new acViscousRedirect(this->acFMAX, 80, 0.002) ;

	//elastic constraint (f_max, k, b, dt)
	this->ac_e_ptr  = new acElastic(this->acFMAX/2, 200, 20, this->period_prop);


	this->iso_visc = new isotropicViscosity(this->iso_visc_params[0], this->iso_visc_params[1]);

	log(RTT::Info) << "AcForceGen configured!" << RTT::endlog();
	return true;
}

//-----------------------------------------------------------------------
// startHook
//-----------------------------------------------------------------------
bool AcForceGen::startHook(){
	RTT::Logger::In in(this->getName());

	if(this->isRunning())
		return false;

	log(RTT::Info) << "AcForceGen started!" << RTT::endlog();
	return true;
}

//-----------------------------------------------------------------------
// startHook
//-----------------------------------------------------------------------
void AcForceGen::updateHook(){
	RTT::Logger::In in(this->getName());

	// Reading data. Not checking for RTT::NewData
	this->port_inp_tool_pose.read(this->current_pose);
	poseMsgToPositionKDLVec(this->current_pose, this->current_pos);

	// Reading the desired tool pose
	this->port_inp_desired_pose.read(this->desired_pose);
	poseMsgToPositionKDLVec(this->desired_pose, this->desired_pos);

	// Reading the tool twist
	this->port_inp_twist.read(this->current_twist);
	this->current_vel =  KDL::Vector(this->current_twist.linear.x,
									 this->current_twist.linear.y, this->current_twist.linear.z);

	// reading the skill metrics if available
//	if(this->metrics_port.connected() && this->metrics_port.read(this->skill_metrics_msg) == RTT::NewData)
//		this->setParametersFromSkillMetrics(this->skill_metrics_msg);


	// updating the transformation based on the camera (if topic available and new message published)
	this->updateMasterToSlaveTransformation();

	// Dealing only with the positions
	// generating the force based on the selected method
	switch(ac_mode){

	case 0:
		// no constraint. Send zero forces.
		this->ac_force= KDL::Vector::Zero();

		break;
	case 1:
		this->ac_p_ptr->getForce(this->ac_force, this->current_pos, this->desired_pos, this->current_vel);

		break;
	case 2:
		this->ac_pr_ptr->getForce(this->ac_force, this->current_pos, this->desired_pos, this->current_vel);

		break;
	case 3:
		this->ac_vr_ptr->getForce(this->ac_force, this->current_pos, this->desired_pos, this->current_vel);
		break;
	case 4:
		this->ac_e_ptr->getForce(this->ac_force, this->current_pos, this->desired_pos, this->current_vel);
		break;

	default:
		this->ac_force= KDL::Vector::Zero();
		break;
	}

	//-----------------------------------------------------------------------------------------------------
	// HELPER METHOD 2: Isotropic viscosity
	//-----------------------------------------------------------------------------------------------------
	// To prevent novice users from reaching high velocities this method applies a simple viscous force.
	this->assist_force_2_visc = this->iso_visc->getViscouseForce(this->current_vel);


	//-----------------------------------------------------------------------------------------------------
	// HELPER METHOD 3: Penetration avoidance
	//-----------------------------------------------------------------------------------------------------
	// to push out the user when they penetrate into the tissue with a viscoelastic force.
	this->assist_force_3_penet_avoid = this->getPenetAvoidanceForce(this->current_pos, this->desired_pos, this->current_vel);

	// writing the wrench on the port
	this->ac_force_master_ref = this->mstr_to_slv_rotation.Inverse() * (this->ac_force + this->assist_force_2_visc + this->assist_force_3_penet_avoid);

	this->wrench_out.force.x = this->ac_force_master_ref[0];
	this->wrench_out.force.y = this->ac_force_master_ref[1];
	this->wrench_out.force.z = this->ac_force_master_ref[2];
	this->port_out_force.write(this->wrench_out);

}

//-----------------------------------------------------------------------
// startHook
//-----------------------------------------------------------------------
void AcForceGen::stopHook() {
	RTT::Logger::In in(this->getName());

	log(RTT::Info) << "AcForceGen executes stopping!" << RTT::endlog();
}

//-----------------------------------------------------------------------
// startHook
//-----------------------------------------------------------------------
void AcForceGen::cleanupHook() {
	RTT::Logger::In in(this->getName());

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
	if(m <0 || m>4)
		std::cout << "ac_mode " << m <<  " not defined. Allowed modes are 0, 1, 2, 3, and 4." << std::endl;
	else
		this->ac_mode = m;
}

//-----------------------------------------------------------------------
// Setting the maximum generated force
//-----------------------------------------------------------------------
void AcForceGen::setAcFMAX(double in){

	if(in <0 || in>15){
		std::cout << "The max force must be between 0 and 15 N." << std::endl;
	}
	else{
		this->acFMAX = in;
		// setting the acFMAX of all ec objects
		this->ac_p_ptr->setFmax(this->acFMAX);
		this->ac_pr_ptr->setFmax(this->acFMAX);
		this->ac_vr_ptr->setFmax(this->acFMAX);
		this->ac_e_ptr->setFmax(this->acFMAX);
		log(RTT::Info)<< "AC FMAX was set to "<< acFMAX << RTT::endlog();

	}
}



void AcForceGen::setIsoViscParams(double _FMAX, double _B){

	if( _B < 0.0  || _B > 70  || _FMAX<0.0  || _FMAX > 15.0)
		log(RTT::Error)<< "setIsoViscParam: Illegal values: _B < 0.0  || _B > 70  "
		"|| _FMAX<0.0  || _FMAX > 15.0" << RTT::endlog();
	else{
		log(RTT::Info)<< "IsoVisc parameters were set as: FMAX = "<< _FMAX << ", B = " << _B << RTT::endlog();
		this->iso_visc->setParam(_FMAX, _B);
	}
}



void AcForceGen::setPenetAvoidParams(double _FMAX, double _B, double _K){

	if( _B < 0.0  || _B > 70  || _FMAX<0.0  || _FMAX > 15.0 || _K <0.0 || _K>600.0)
		log(RTT::Error)<< "setPenetAvoid: Illegal values: _B < 0.0  || _B > 70  "
		"|| _FMAX<0.0  || _FMAX > 15.0 || _K <0.0 || _K>300.0)" << RTT::endlog();
	else{
		this->FMAX_penet_avoid 	= _FMAX;
		this->B_penet_avoid 	= _B;
		this->K_penet_avoid 	= _K;
		log(RTT::Info)<< "Penet. avoid. parameters were set as: FMAX = "<< _FMAX << ", B = " << _B << ", K = " << _K << RTT::endlog();

	}

}


void AcForceGen::setAllMaxForces(const double FMAX_ac, const double FMAX_PenetAvoid, const double FMAX_isovisc){

	if( FMAX_ac < 0.0  || FMAX_ac > 5  || FMAX_PenetAvoid<0.0  || FMAX_PenetAvoid > 5.0 || FMAX_isovisc <0.0 || FMAX_isovisc>5.0)
		log(RTT::Error)<< "setAllMaxForces: Illegal values! only forces from 0.0 N to 5.0 N are allowed" << RTT::endlog();
	else{
		// set the values
		setAcFMAX(FMAX_ac);
		this->FMAX_penet_avoid 	= FMAX_PenetAvoid;
		this->iso_visc->setFMAX(FMAX_isovisc);

		// inform the operator
		log(RTT::Info)<< "Maximum forces were set as: FMAX_ac = "<< FMAX_ac << ", FMAX_PenetAvoid = "
				<< FMAX_penet_avoid << ", FMAX_isovisc = " << FMAX_isovisc << RTT::endlog();
	}
}



void AcForceGen::setParametersFromSkillMetrics(const active_guidance::skillProbabilities & _probs){

	double FMAX_ac, FMAX_PenetAvoid, FMAX_isovisc;

	FMAX_ac = _probs.accuracy * 3.0;
	FMAX_PenetAvoid = _probs.depthperception * 2.0;
	FMAX_isovisc =  _probs.motionconsistency * 2.0;

	// if on of the other two helpers were half active set a minimum isovisc to stabilize them
	if(FMAX_isovisc < 0.5 && ( FMAX_ac > 1.0 || FMAX_PenetAvoid > 1.0) )
		FMAX_isovisc = 0.5;

	this->setAllMaxForces(FMAX_ac,FMAX_PenetAvoid, FMAX_isovisc);

}




KDL::Vector AcForceGen::getPenetAvoidanceForce(const KDL::Vector & _current_pos , const KDL::Vector & _desired_pos, const KDL::Vector & _vel){

	double penetHelpDist = -0.003; // start the helper when the tool is at this distance.
	double dz = _desired_pos.data[2] - _current_pos.data[2];

	//Generate push out force
	if(dz > penetHelpDist){
		// at the end decided to use the Viscous assistance and set the B_penet_avoid = 0.0
		double f = (dz-penetHelpDist) * this->K_penet_avoid  - _vel.Norm() * this->B_penet_avoid;

		// Saturate the force
		if(f>this->FMAX_penet_avoid)
			f = this->FMAX_penet_avoid;
		if(f<0.0)
			f = 0.0;

		return KDL::Vector(0.0, 0.0, f );
	}
	else
		return KDL::Vector::Zero();
}




void AcForceGen::initializeMasterToSlaveTransformation(){

	RTT::Logger::In in(this->getName());

	geometry_msgs::Quaternion pose_in;

	// reading the cam to slave data if available
	if (this->port_inp_mstr_to_slv_tr.connected() && this->port_inp_mstr_to_slv_tr.read(pose_in) != RTT::NoData ){

		mstr_to_slv_rotation = KDL::Rotation::Quaternion(pose_in.x, pose_in.y, pose_in.z, pose_in.w);
		// notify the user
		log(RTT::Info)<< " Updated the camera to slave transformation." << RTT::endlog();

	}
	else{
		log(RTT::Warning)<< "Port " << this->port_inp_mstr_to_slv_tr.getName() << " is not connected. Using backup cam to master rotation" << RTT::endlog();
		mstr_to_slv_rotation = mstr_to_slv_rotation_backup;
	}

}


void AcForceGen::updateMasterToSlaveTransformation(){

	RTT::Logger::In in(this->getName());

	geometry_msgs::Quaternion pose_in;

	// reading the cam to slave data if available
	if (this->port_inp_mstr_to_slv_tr.connected() && this->port_inp_mstr_to_slv_tr.read(pose_in) == RTT::NewData ){

		mstr_to_slv_rotation = KDL::Rotation::Quaternion(pose_in.x, pose_in.y, pose_in.z, pose_in.w);
		// notify the user
		log(RTT::Info)<< " Updated the master to slave transformation." << RTT::endlog();

	}

}


//-----------------------------------------------------------------------
// Isotropic viscosity class
//-----------------------------------------------------------------------

KDL::Vector isotropicViscosity::getViscouseForce(const KDL::Vector _vel){

	return saturateForce( -B * _vel);
};


KDL::Vector isotropicViscosity::saturateForce(const KDL::Vector _f){

	double f_norm = _f.Norm();

	if ( f_norm > FMAX)
		return( _f /f_norm * FMAX);
	else
		return _f;
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
