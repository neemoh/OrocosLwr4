#include "sigma7-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

using namespace RTT;
using namespace std;

sigma7::sigma7(std::string const& name) :
						TaskContext(name, PreOperational) {
	this->addPort("sigma_pose_read", this->master_pose_port).doc("Readings of the pose from Sigma");
	this->addPort("sigma_twist_read", this->master_twist_port).doc("Readings of the pose from Sigma");

	this->addPort("sigma_pose_dest", this->slave_pose_port).doc("Destination");
	this->addPort("sigma_button", this->botton_port).doc("Haptic device button");
	this->addPort("sigma_pedal", this->pedal_port).doc("Pedal port");
	this->addEventPort("trigger", this->trigger).doc("Destination");
	this->addPort("forceToMaster", this->force_to_master_port).doc("Readings of the pose from Sigma");

	this->addOperation("usePedal", &sigma7::usePedal, this);
	this->addProperty("tr_master_to_base", 	master_to_base_frame_prop).doc("Rigid transformation from master to robot base.");
	this->addProperty("tr_master_to_tool_orient", 	this->master_to_tool_orient_frame_prop).doc("Rigid transformation (quaternion) to achieve the desired orientation between the master handle and the slave tool. (Attention to x,y,z,w order).");

	//Initialize variables
	this->use_pedal = 0;

}
//######################################################################################################
//######################################################################################################

bool sigma7::configureHook() {

	RTT::Logger::In in(this->getName());

	//manually raises LogLevel to 'Info' (5)
	if (Logger::log().getLogLevel() < Logger::Info ) {
		Logger::log().setLogLevel( Logger::Info );
		Logger::log() << Logger::Info <<  "Log Level raised manually to Info."<<Logger::endl;
	}

	if (!this->initSigma())
		return false;
	// properties
	this->mstr_to_slv_frame.M	= KDL::Rotation::Quaternion(this->master_to_base_frame_prop[0], this->master_to_base_frame_prop[1], this->master_to_base_frame_prop[2], this->master_to_base_frame_prop[3]);
	this->master_to_tool_orient_frame.M				= KDL::Rotation::Quaternion(this->master_to_tool_orient_frame_prop[0], this->master_to_tool_orient_frame_prop[1], this->master_to_tool_orient_frame_prop[2], this->master_to_tool_orient_frame_prop[3]);


	log(RTT::Info) << "sigma7 configured !" << Logger::endl;
	//	dhdGetOrientationRad(&this->lock_orientation[0], &this->lock_orientation[1],&this->lock_orientation[2]);

	return true;
}
//######################################################################################################
//######################################################################################################

bool sigma7::startHook() {
	// reading the pose of the slave and setting the initial orientation of the
	// master equal to that of the slave.

	return true;
}

//######################################################################################################
//######################################################################################################

void sigma7::updateHook() {
	Logger::In in(this->getName());


	/////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////
	// Reading Sigma measurements and setting the force
	/////////////////////////////////////////////////////////////////////////////////////////////////
	double p[3],v[3];
	double orientation_matrix[3][3];

	// Read Position and Orientation from Sigma
	dhdGetPositionAndOrientationFrame(&p[0], &p[1], &p[2], orientation_matrix);
	this->getFrame(p, orientation_matrix, this->HI_Frame);
	tf::PoseKDLToMsg(this->HI_Frame, this->HI_pose);

	//Read Button state
	this->sigma_button_state.data = dhdGetButton(0);
	this->sigma_pedal_state.data = dhdGetButton(1);

	dhdGetLinearVelocity(&v[0], &v[1], &v[2]);
	HI_twist.linear.x = v[0];
	HI_twist.linear.y = v[1];
	HI_twist.linear.z = v[2];

	if(this->force_to_master_port.read(this->tmp_wrench) == RTT::NewData)
		this->mstr_wrench_cmd = this->tmp_wrench;

	//Applying force feedback if the pedal is engaged
	if((this->sigma_pedal_state.data == 1)){
		dhdEnableForce(DHD_ON);
		//Enable the gripper button
		//		dhdEmulateButton(DHD_ON);
		if (dhdSetForceAndTorqueAndGripperForce(this->mstr_wrench_cmd.force.x, this->mstr_wrench_cmd.force.y, this->mstr_wrench_cmd.force.z, 0.0, 0.0, 0.0, 0.0) < DHD_NO_ERROR) {
			log(RTT::Error) << "error: cannot set force \n" <<  dhdErrorGetLastStr() << endlog();
		}
		//		dhdGetOrientationRad(&lock_orientation[0], &lock_orientation[1],&lock_orientation[2]);
	}
	else
		this->lockOrientation();

	this->master_pose_port.write(this->HI_pose);
	this->master_twist_port.write(this->HI_twist);
	if(this->sigma_button_state.data != this->sigma_button_previous_state.data){
		this->botton_port.write(sigma_button_state);
	}
	if(this->sigma_pedal_state.data != this->sigma_pedal_previous_state.data){
		this->pedal_port.write(sigma_pedal_state);
	}
	this->sigma_button_previous_state.data = this->sigma_button_state.data;
	this->sigma_pedal_previous_state.data = this->sigma_pedal_state.data;

}

void sigma7::stopHook() {
	Logger::In in(this->getName());

	dhdStop();
	drdClose();
	if (dhdClose () < 0) {
		log(RTT::Error) << "Cannot close dhd. " <<  dhdErrorGetLastStr() << endlog();
	}
	else
		log(RTT::Info) << "dhd Closed." <<  dhdErrorGetLastStr() << endlog();

}

void sigma7::cleanupHook() {

}

bool sigma7::initSigma() {
	Logger::In in(this->getName());

	// *************************************************************************************************
	// *********************************** Initialization: Sigma  **************************************
	// *************************************************************************************************

	int major, minor, release, revision;
	dhdGetSDKVersion (&major, &minor, &release, &revision);
	log(RTT::Info) << "Force Dimension - SDK version: " << major <<"."<< minor<<"."<< release<<"."<< revision<<Logger::endl;

	// required to change asynchronous operation mode
	dhdEnableExpertMode();
	// open the first available device
	if (drdOpen () < 0) {
		//------ FOR SOME REASON THE DHD GIVES EN ERROR HERE EVEN IF IT SEEMS TO BE WORKING FINE.
		log(RTT::Error) << "Cannot open device. dhd says: " <<  dhdErrorGetLastStr() << endlog();
		dhdSleep (2.0);
		return false;
	}
	// identify device
	log(RTT::Info) <<  dhdGetSystemName() << " device detected." << Logger::endl;

	// calibration is performed externally
	if(drdIsInitialized()){
		log(RTT::Info)  <<"Device is already calibrated." << endlog();
	}
	else if(drdAutoInit()<0) {
		log(RTT::Error) << "initialization failed (%s)\n" << dhdErrorGetLastStr() << endlog();
		dhdSleep(2.0);
	}
	double nullPose[7] = { 0.0, 0.0, 0.0,  // base  (translations)
			0.0, 0.0, 0.0,  // wrist (rotations)
			0.0 };          // gripper

	drdRegulatePos  (true);
	drdRegulateRot  (true);
	drdRegulateGrip (true);
	drdStart();
	drdMoveTo(nullPose);
	// stop regulation (and leave force enabled)
	drdStop(true);
	//		drdClose(true);
	// enable force

	dhdEnableForce (DHD_ON);
	//Enable the gripper button
	dhdEmulateButton(DHD_ON);
	return true;
}
/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(sigma7)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
void sigma7::getFrame( double pos_array[], double orientation_matrix[][3], KDL::Frame& frame){
	//Add: check sizes
	frame.p[0] = pos_array[0];
	frame.p[1] = pos_array[1];
	frame.p[2] = pos_array[2];
	for (int i = 0; i < 3; i++) {
		for(int j = 0; j < 3; j++) {
			frame.M(i,j) = orientation_matrix[i][j];
		}
	}
}

void sigma7::usePedal(bool input){
	Logger::In in(this->getName());
	if(this->use_pedal == input){
		log(RTT::Info) << "No change needed." << endlog();
		return;
	}
	this->use_pedal = input;
	if(this->use_pedal)	log(RTT::Info) << "Will be using the pedal from now on!" << endlog();
	if(!this->use_pedal)	log(RTT::Info) << "Ok! Pedal will not be used." << endlog();

}

void sigma7::lockOrientation(){
	geometry_msgs::Pose temp_pose;
	KDL::Frame temp_frame;

	// read the orientation from the slave
	if(this->slave_pose_port.connected() && this->slave_pose_port.read(temp_pose)== RTT::NewData){
		tf::PoseMsgToKDL(temp_pose, temp_frame);
		temp_frame.M = this->mstr_to_slv_frame.M.Inverse() * temp_frame.M * this->master_to_tool_orient_frame.M.Inverse();
		//		temp_frame.M = this->master_to_tool_orient_frame.M.Inverse()	* temp_frame.M;
		temp_frame.M.GetRPY(this->lock_orientation[0],this->lock_orientation[1],this->lock_orientation[2]);
		//		cout << this->mstr_to_slv_frame.M.data[0]   << "  " << this->mstr_to_slv_frame.M.data[1] << "  " << this->mstr_to_slv_frame.M.data[2]<< endl;
		//		cout << this->mstr_to_slv_frame.M.data[3]   << "  " << this->mstr_to_slv_frame.M.data[4] << "  " << this->mstr_to_slv_frame.M.data[5]<< endl;
		//		cout << this->mstr_to_slv_frame.M.data[6]   << "  " << this->mstr_to_slv_frame.M.data[7] << "  " << this->mstr_to_slv_frame.M.data[8]<< endl;
		//		cout << "lock_orientation[0]" << this->lock_orientation[0] << "  " <<this->lock_orientation[1] << "  "<< 		this->lock_orientation[2] << endl;
	}
	else{ // no input orientation, so set it as zero
		this->lock_orientation[0] =
				this->lock_orientation[1] =
						this->lock_orientation[2] = 0.0;
	}
	// send the device to that orientation
	drdRegulatePos  (false);
	drdRegulateRot  (true);
	drdRegulateGrip (false);
	drdStart();
	drdMoveToRot (this->lock_orientation[0], this->lock_orientation[1],this->lock_orientation[2]);
	drdStop(true);

}
ORO_CREATE_COMPONENT(sigma7)
