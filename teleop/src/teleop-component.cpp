#include "teleop-component.hpp"

using namespace std;
using namespace RTT;
using namespace KDL;
using namespace tf;

typedef std::vector<double> vec_dbl;
#define TOLERANCE 0.00001
#define INTERPOLATION_TOLERANCE 0.001

Teleop::Teleop(std::string const& name) : TaskContext(name, PreOperational){

	this->addPort("outputJointPos", this->joint_command_port).doc("Readings of the pose from Sigma");
	this->addPort("outputCartPose", this->cart_command_port).doc("Readings of the pose from Sigma");
	this->addPort("outputCartPoseFK", this->cart_FK_port).doc("Readings of the pose from Sigma");
	this->addPort("outputCartTwist", this->port_cart_twist).doc("Readings of the pose from Sigma");
	this->addEventPort("inputJointCurrent", this->joint_curr_port).doc("Readings of the pose from PhanomOmni");
	this->addPort("inputCartDestination", this->cart_dest_port).doc("Output of the pose to Sigma");
	this->addPort("inputSlaveCart", this->slave_cart_port).doc("Output of the pose to Sigma");

	this->addPort("inputMasterClutch", this->master_clutch_port).doc("Output of the pose to Sigma");
	this->addPort("forceToMaster", this->force_to_master_port).doc("Readings of the pose from Sigma");
	this->addPort("forceFromSlave", this->force_from_slave_port).doc("Readings of the pose from Sigma");

//	this->addPort("toKRL_master", this->port_to_krl_master).doc("Commands Port to KRC");
//	this->addPort("fromKRL_master", this->port_from_krl_master).doc("Status Port from KRC");

	this->addProperty("motion_mode", motion_mode_prop).doc("type of motion: 1 for point to point, 2 for tracking");
	this->addProperty("force_feedback_on", force_feedback_on_prop).doc("Home joint positions]");
	this->addProperty("force_filter_on", force_filter_on_prop).doc("Home joint positions]");
	this->addProperty("teleop_pos_coupled", teleop_pos_coupled_prop).doc("Coupling the position of master and slave");
	this->addProperty("teleop_ori_coupled", teleop_ori_coupled_prop).doc("Coupling the orientation of master and slave");

	this->addProperty("force_scale", force_scale_prop).doc("Home joint positions]");
	this->addProperty("translation_scale", transl_scale_prop).doc("scale of translation");
	this->addProperty("tool_zlength", tool_zlength_prop).doc("Rigid transformation (quaternion) from force sensor to the endeffector. (Attention to x,y,z,w order).");
	this->addProperty("master_orientation_average_steps", 	rpy_avg_n_prop).doc("The number of averaging steps for the orientation (rpy) of the master device.");
	this->addProperty("master_position_average_steps", 		pos_avg_n_prop).doc("The number of averaging steps for the position (xyz) of the master device.");

	this->addProperty("time_period", 				this->period_prop).doc("The constant time period used for interpolation");
	this->addProperty("max_joint_vars", 			this->slv_jnt_q_max_prop).doc("Maximum limit [measurement_unit]");
	this->addProperty("min_joint_vars", 			this->slv_jnt_q_min_prop).doc("Minimum limit [measurement_unit]");
	this->addProperty("max_joint_vel", 				this->slv_jnt_v_max_prop).doc("Maximum Velocity [measurement_unit/s]");
	this->addProperty("max_joint_acc", 				this->slv_jnt_a_max_prop).doc("Maximum Acceleration [measurement_unit/s]");
	this->addProperty("max_cart_vars", 				this->slv_cart_q_max_prop).doc("Maximum limit [measurement_unit]");
	this->addProperty("min_cart_vars", 				this->slv_cart_q_min_prop).doc("Minimum limit [measurement_unit]");
	this->addProperty("max_cart_vel", 				this->slv_cart_v_max_prop).doc("Maximum Velocity [measurement_unit/s]");
	this->addProperty("max_cart_acc", 				this->slv_cart_a_max_prop).doc("Maximum Acceleration [measurement_unit/s]");
	this->addProperty("slv_home_joint", 			this->slv_jnt_home_prop).doc("Home joint positions]");
	this->addProperty("tr_master_to_base", 			this->master_to_base_frame_prop).doc("Rigid transformation from master to robot base.");
	this->addProperty("tr_fs_to_ee", 				this->fs_to_ee_frame_prop).doc("Rigid transformation (quaternion) from force sensor to the endeffector. (Attention to x,y,z,w order).");
	this->addProperty("tr_master_to_tool_orient", 	this->master_to_tool_orient_frame_prop).doc("Rigid transformation (quaternion) to achieve the desired orientation between the master handle and the slave tool. (Attention to x,y,z,w order).");

	this->addProperty("palpation_home_3dpose", 	    this->palpation_home_3dpose_prop).doc("Rigid transformation (quaternion) to achieve the desired orientation between the master handle and the slave tool. (Attention to x,y,z,w order).");
	this->addProperty("palpation_init_3dpose", 	    this->palpation_init_3dpose_prop).doc("Rigid transformation (quaternion) to achieve the desired orientation between the master handle and the slave tool. (Attention to x,y,z,w order).");

	this->addOperation("motionON", 					&Teleop::startMotion, 				this).doc("Starts the motion mode");
	this->addOperation("motionOff", 			    &Teleop::stopMotion, 				this).doc("Stops the motion");
	this->addOperation("changeMotionMode", 			&Teleop::changeMotionMode, 			this).doc("Changes the motion mode");
	this->addOperation("setPTPCartPosDestination",  &Teleop::setPTPCartDestination,  	this).doc("setPTPCartDestination");
	this->addOperation("setPTPJointDestination", 	&Teleop::setPTPJointDestination,	this).doc("setPTPCartDestination");
	this->addOperation("goHome", 					&Teleop::goHome, 					this).doc("setPTPJointDestination to saved home position");
	this->addOperation("wtf", 						&Teleop::wtf, 						this).doc("sShow current measurements and states");
	this->addOperation("switchForceFeedback", 		&Teleop::switchForceFeedback, 		this).doc("sShow current measurements and states");
	this->addOperation("calibFF", 					&Teleop::forceSensorCalib, 			this).doc("sShow current measurements and states");
	this->addOperation("forceFilterSwitch", 		&Teleop::forceFilterSwitch, 		this).doc("Switches force filter on->off and vice versa");
	this->addOperation("positionCoupling", 			&Teleop::positionCoupling, 			this).doc("Switches the master/slave coupling of position. On or Off");
	this->addOperation("orientationCoupling", 		&Teleop::orientationCoupling,		this).doc("Switches the master/slave coupling of orientation. On or Off");

	this->addOperation("startPalpation", 		&Teleop::startPalpation,		this).doc("Switches the master/slave coupling of orientation. On or Off");
	this->addOperation("stopPalpation", 		&Teleop::stopPalpation,		this).doc("Switches the master/slave coupling of orientation. On or Off");


	this->num_axes 				= 7;
	this->num_joints			= 7;
	this->num_cart_p_var 		= 3; //x,y and z
	this->num_cart_var 			= 7; //x,y,z + quaternion
	this->robot_config 			= 0;

	this->T_joint				= 0.0;

	this->interp_counter 		= 0;
	this->motion_mode			= 0;
	this->force_bias_counter	= 0;
	this->force_scale			= 1.0;
	this->transl_scale			= 1.0;
	this->tool_zlength			= 0.0;
	this->first_engagement_counter_rpy = 0;

	this->destination_reached 	= true;
	this->teleop_interpolate_done= false;
	this->motionOn 				= true;
	this->teleop_pos_coupled	= false;
	this->teleop_ori_coupled	= false;

	this->new_cart_dest 		= false;
	this->new_joint_dest 		= false;
	this->clutch_first_time 	= true;
	this->force_filter_on		= false;
	mstr_to_slv_frame.Identity();

	//---------------------------------- Temp Palpation
	this->palp_cart_destination = vec_dbl(3,0.0);
	this->palpation_first_time= 0;
	this->palp_point_status = 0 ;
	this->palpation_on = false;
	//---------------------------------- Temp Palpation


	std::cout << "Teleop constructed!" <<std::endl;
}

bool Teleop::configureHook(){
	RTT::Logger::In in(this->getName());

	if (this->isConfigured())
		return false;

	/////////////////////////////////     Checking  properties' size       ////////////////////////////////
	if (this->slv_jnt_home_prop.size() != this->num_joints) {
		log(RTT::Error) << "Size of slv_jnt_home_prop does not match: " << this->num_joints << endlog();		return false;
	}
	if (this->slv_cart_q_min_prop.size() != this->num_cart_p_var) {
		log(RTT::Error) << "Size of slv_cart_p_min_prop does not match: " << this->num_cart_var << endlog();		return false;
	}
	if (this->slv_cart_q_max_prop.size() != this->num_cart_p_var) {
		log(RTT::Error) << "Size of slv_cart_p_max_prop does not match: " << this->num_cart_var << endlog();		return false;
	}
	if (this->slv_cart_v_max_prop.size() != this->num_cart_p_var) {
		log(RTT::Error) << "Size of slv_cart_vel_max_prop does not match: " << this->num_cart_var << endlog();		return false;
	}
	if (this->slv_cart_a_max_prop.size() != this->num_cart_p_var) {
		log(RTT::Error) << "Size of slv_cart_acc_max_prop does not match: " << this->num_cart_var << endlog();		return false;
	}
	if (this->slv_jnt_q_min_prop.size() != this->num_joints) {
		log(RTT::Error) << "Size of slv_joint_p_min_prop does not match: " << this->num_joints << endlog();		return false;
	}
	if (this->slv_jnt_q_max_prop.size() != this->num_joints) {
		log(RTT::Error) << "Size of slv_jnt_q_max_prop does not match: " << this->num_joints << endlog();		return false;
	}
	if (this->slv_jnt_v_max_prop.size() != this->num_joints) {
		log(RTT::Error) << "Size of slv_jnt_v_max_prop does not match: " << this->num_joints << endlog();		return false;
	}
	if (this->slv_jnt_a_max_prop.size() != this->num_joints) {
		log(RTT::Error) << "Size of slv_joint_acc_max_prop does not match: " << this->num_joints << endlog();		return false;
	}

	/////////////////////////////////      Variable initialization     //////////////////////////////
	if (!this->changeMotionMode(this->motion_mode_prop))
		return false;

	// set the initial sleep time of the freq observer to 1 seconds
	this->fo = new freqObserver(this->period_prop, 1/this->period_prop, 10);

	// construct the velocity filter object
//	this->v_filt = new FOAWBestFit(10, 1/fo->dt_param, 0.0001);


	//////////////			Using PROPERTIES			 ///////////////

	this->motion_mode 				= this->motion_mode_prop;
	this->force_filter_on			= this->force_filter_on_prop;
	this->teleop_pos_coupled		= this->teleop_pos_coupled_prop;
	this->teleop_ori_coupled		= this->teleop_ori_coupled_prop;

	this->force_scale				= this->force_scale_prop;
	this->transl_scale				= this->transl_scale_prop;
	this->tool_zlength				= this->tool_zlength_prop;
	this->rpy_avg_n					= this->rpy_avg_n_prop;
	this->pos_avg_n					= this->pos_avg_n_prop;

	this->slv_jnt_home 				= this->slv_jnt_home_prop;
	this->slv_jnt.q_min 			= this->slv_jnt_q_min_prop;
	this->slv_jnt.q_max 			= this->slv_jnt_q_max_prop;
	this->slv_jnt.v_max 			= this->slv_jnt_v_max_prop;
	this->slv_jnt.a_max 			= this->slv_jnt_a_max_prop;
	this->slv_cart.q_min 			= this->slv_cart_q_min_prop;
	this->slv_cart.q_max 			= this->slv_cart_q_max_prop;
	this->slv_cart.v_max 			= this->slv_cart_v_max_prop;
	this->slv_cart.a_max 			= this->slv_cart_a_max_prop;

	this->kine = new LWR4Kinematics(this->tool_zlength);

	this->mstr_to_slv_frame.M			= KDL::Rotation::Quaternion(this->master_to_base_frame_prop[0], this->master_to_base_frame_prop[1], this->master_to_base_frame_prop[2], this->master_to_base_frame_prop[3]);
	this->fs_to_ee_frame.M				= KDL::Rotation::Quaternion(this->fs_to_ee_frame_prop[0], this->fs_to_ee_frame_prop[1], this->fs_to_ee_frame_prop[2], this->fs_to_ee_frame_prop[3]);
	this->master_to_tool_orient_frame.M				= KDL::Rotation::Quaternion(this->master_to_tool_orient_frame_prop[0], this->master_to_tool_orient_frame_prop[1], this->master_to_tool_orient_frame_prop[2], this->master_to_tool_orient_frame_prop[3]);

	this->slv_joint_p_interpd 		=
	this->slv_joint_p_interpd_init 	=
	this->slv_jnt.q_curr 			=
	this->slv_jnt.q_last 			=
	this->slv_jnt.q_dest 			=
	this->slv_jnt.q_cmd 			=
	this->slv_jnt.q_cmd_last 		=
	this->slv_jnt.v_curr			=
	this->slv_jnt.v_last			=
	this->tmp_joint_vec 			=
	this->h_joint					=
	this->a1 						=
	this->a2 						=
	this->a3 						= vec_dbl(this->num_joints, 0.0);

	this->slv_cart.q_curr 			=
	this->slv_cart.q_last 			=
	this->slv_cart.q_dest 			=
	this->slv_cart.q_cmd 			=
	this->slv_cart.q_cmd_last 		=
	this->tmp_cart_vec 				=

	this->mstr_pose_init			=
	this->mstr_pose_curr			=
	this->mstr_pose_curr_filt		=
	this->mstr_dp 					= vec_dbl(this->num_cart_var, 0.0);

	this->cart_6d_last   	   		=
	this->v_cart_6d_last 			= vec_dbl(6, 0.0);
	this->slv_cart.v_curr 			= vec_dbl(this->num_cart_p_var, 0.0);

	this->ns_param 					= std::vector<double>(1, 0.0);

	this->force_bias 				= KDL::Vector::Zero();
	this->force_filtered 			=  KDL::Vector::Zero();

	this->mstr_rpy_avg.resize(3, 0.0);
	this->mstr_deltapos_avg.resize(3, 0.0);

	this->mstr_wrench_cmd = conversions::createWrenchMsg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

	this->tmp_joint_state.position.resize(num_joints, 0.0);
	this->tmp_joint_pos.positions.resize(num_joints, 0.0);
	this->lwr_joint_msg_command.positions.resize(num_joints, 0.0);

	conversions::poseReset(this->tmp_pose_msg);
	conversions::vec3Reset(this->tmp_vec3);
	conversions::jointPosReset(this->tmp_joint_pos);

	//Initialize output ports
	this->joint_command_port.setDataSample(this->tmp_joint_pos);
	this->port_cart_twist.setDataSample(this->tmp_twist);
	this->force_to_master_port.setDataSample(this->mstr_wrench_cmd);
	this->cart_FK_port.setDataSample(this->tmp_pose_msg);

	//Serial link parameters
	this->limbs[0]				= 0.31; // length from robot base to shoulder
	this->limbs[1] 				= 0.4; // length from shoulder to elbow
	this->limbs[2] 				= 0.39; // length from elbow to wrist
	this->limbs[3] 				= 0.078 + this->tool_zlength; // length from wrist to flange + tool length(position of target)

	// temp stuff
	this->mode4_counter = 0;
	this->first_time_temp = true;
	this->temp_config = 0;
	this->posit_init =
	this->posit_dest =
	this-> posit_cmd=
	this->h_temp =
	this->posit_last =
    this->v_last =
    this->v_cmd =  vec_dbl(3, 0.0);
	this->psi_v_cmd = this->psi_v_last = this->psi_last= vec_dbl(1,0.0);
	this->dest_reached_temp = false;
	this->new_dest_temp = true;
	this->interp_counter_temp = 0;

	this->to = new teleopC;
	this->to->mstr_to_slv_frame = this->mstr_to_slv_frame;
	this->to->fs_to_ee_frame = this->fs_to_ee_frame;
	this->to->force_feedback_on = this->force_feedback_on_prop;


	std::cout << "Teleop configured!" <<std::endl;
	if(this->motionOn) cout<< "Motion is ON." << endl;
	else cout<< "Motion is Off." << endl;
	cout << "Tool length is:" << this->tool_zlength << endl;
	cout << "Motion mode is: "<< this->motion_mode << endl;
	return true;
}

bool Teleop::startHook(){

	if(this->isRunning())
		return false;

	return true;
}



void Teleop::updateHook(){
	double manipT,manipA,manipR;

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//														Reading data
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	double palp_area_l = 0.06;
	double palp_area_h = 0.06;
	double palp_area_d = 0.002;
	this->fo->computation_time_from = os::TimeService::Instance()->getTicks();

	///////////////////////////////////////////
	//Reading the joint positions of the robot
//	if (!this->joint_curr_port.connected() ){
//		log(RTT::Error)<< " port" << this->cart_dest_port.getName() << " Is not connected" << endlog();
//	}
	if( this->joint_curr_port.read(this->tmp_joint_state) == RTT::NewData) {

		conversions::jointStateToJointPosVector(this->tmp_joint_state, this->tmp_joint_vec );
		this->slv_jnt.q_curr = this->tmp_joint_vec;

		//Forward kinematics
		this->kine->fk(this->slv_jnt.q_curr, this->robot_config, this->ns_param.at(0), this->tmp_frame);

		conversions::KDLFrameToVector(this->tmp_frame, this->tmp_cart_vec);
		conversions::vectorToPoseMsg(this->tmp_cart_vec, this->tmp_pose_msg);
		this->slv_frame_curr = this->tmp_frame;
		this->slv_cart.q_curr = this->tmp_cart_vec;

		this->tmp_cart_vec = std::vector<double>(7, 0.0);
		this->cart_FK_port.write(this->tmp_pose_msg);

		this->tmp_joint_vec = std::vector<double>(this->num_joints, 0.0); 		//Reset temporary variables


		// manipulability indexes
		std::vector<KDL::Frame> joint_frames(this->num_joints + 1, KDL::Frame::Identity());
		KDL::Jacobian jac(7);

		this->kine->fk_all(this->slv_jnt.q_curr, this->robot_config, this->ns_param.at(0), joint_frames);
		this->kine->jacobian(joint_frames, jac);
		this->kine->getManipulabilityIdx(jac, 0, manipA);
		this->kine->getManipulabilityIdx(jac, 1, manipT);
		this->kine->getManipulabilityIdx(jac, 2, manipR);
	}

	///////////////////////////////////////////
	//Reading the haptic device's clutch state
	if(this->master_clutch_port.connected() && this->master_clutch_port.read(this->tmp_button) == RTT::NewData){
		this->master_clutch = this->tmp_button;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//													Generating motion commands
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	if(this->motionOn){


		switch (this->motion_mode) {
		case 0:
			//Just chillin'
			break;

		case 1:
			/////////////////////////////// Cartesian or joint PTP with Joint trajectory ////////////////////////////////////
			/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Joint trajectory with 3rd order polynomial velocity profile current for either Cartesian or joint
			// point to point motion.
			// The destination is set from the deployer through either setPTPCartDestination or setPTPJointDestination
			// operations.
			/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			if (this->new_cart_dest ){

				///////////////////////////////////////////
				//Inverse Kinematics
				if(conversions::vectorToKDLFrame(this->slv_cart.q_dest, this->tmp_frame))
				{
					if (this->kine->ik(this->tmp_frame, this->robot_config,  this->ns_param.at(0), this->tmp_joint_vec)) {
						this->setPTPJointDestination(this->tmp_joint_vec);
					}
					else {
						log(RTT::Warning) << "No good pose of the robot; no solution of the inverse kinematics: due to bad target or bad configuration or bad nullspace parameter" << endlog();
						this->tmp_joint_vec = std::vector<double>(this->num_joints, 0.0); 		//Reset temporary variables
					}
				}
				this->new_cart_dest = false;
			}

			///////////////////////////////////////////
			//Joint Interpolation
			if (!this->destination_reached){
				if(!this->P2PInterpolator(this->slv_jnt.q_dest, this->slv_joint_p_interpd_init, this->slv_joint_p_interpd, this->slv_jnt.v_max, this->slv_jnt.a_max, this->h_joint,  this->T_joint, this->interp_counter, this->new_joint_dest, this->destination_reached)){
					log(RTT::Error)<<  "Error P2P interpolator. Stopping the interpolator" <<endlog();
					this->motionOn = false;
				}
				this->slv_jnt.q_cmd = this->slv_joint_p_interpd;
			}

			break;


		case 2:
			/////////////////////////////// Cartesian PTP with Cartesian trajectory ////////////////////////////////////////////////
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Cartesian semi-straight line connecting the current position to the Destination with
			// Trapezoidal velocity profile.
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			///////////////////// TEMP PALPATION
			//-------------------------------------------------------------------------------------------------------------------
			if(this->palpation_on){
				if(this->palpation_first_time){
					this->palp_cart_destination = this->palpation_init_3dpose_prop;
					setPTPCartDestination(this->palp_cart_destination);
					this->palpation_first_time = false;
				}

				if (this->destination_reached){
					if (this->palp_cart_destination[1]< (this->palpation_init_3dpose_prop[1] + palp_area_h)){
						switch (this->palp_point_status){

						case 0 :

							this->palp_cart_destination[0] += palp_area_l;
							this->palp_point_status++;
							break;
						case 1:

							this->palp_cart_destination[1] += palp_area_d;
							this->palp_point_status++;
							break;

						case 2:

							this->palp_cart_destination[0] -= palp_area_l;
							this->palp_point_status++;
							break;
						case 3:

							this->palp_cart_destination[1] += palp_area_d;
							this->palp_point_status = 0;
							break;


						default:
							break;
						}
//						cout << "this->palp_cart_destination: " << this->palp_cart_destination << endl;
						setPTPCartDestination(this->palp_cart_destination);

					}

				}


			}




			//-------------------------------------------------------------------------------------------------------------------
			///////////////////// TEMP PALPATION


			if (!this->destination_reached){

				///////////////////////////////////////////
				vec_dbl cart_6d_dest  = vec_dbl(6,0.0);
				vec_dbl cart_6d_cmd   = vec_dbl(6,0.0);
				vec_dbl	v_cart_6d_now = vec_dbl(6,0.0);
				vec_dbl cart_6d_min   = vec_dbl(6,-2.0);
				vec_dbl cart_6d_max   = vec_dbl(6,2.0);
				vec_dbl cart_6d_v_max   = vec_dbl(6,0.2);
				vec_dbl cart_6d_a_max   = vec_dbl(6,0.1);

				cart_6d_a_max[0] = this->slv_cart.a_max[0];
				cart_6d_a_max[1] = this->slv_cart.a_max[1];
				cart_6d_a_max[2] = this->slv_cart.a_max[2];
				cart_6d_v_max[0] = this->slv_cart.v_max[0];
				cart_6d_v_max[1] = this->slv_cart.v_max[1];
				cart_6d_v_max[2] = this->slv_cart.v_max[2];

				if(this->new_cart_dest){
					this->cart_6d_last[0] = this->slv_cart.q_curr.at(0);
					this->cart_6d_last[1] = this->slv_cart.q_curr.at(1);
					this->cart_6d_last[2] = this->slv_cart.q_curr.at(2);
					slv_frame_curr.M.GetRPY(cart_6d_last[3], cart_6d_last[4], cart_6d_last[5]);
					this->psi_last[0] = this->ns_param.at(0);
					this->new_cart_dest = false;
				}


				//Saving the position values in a variable for the interpolator
				cart_6d_dest[0] = this->slv_cart.q_dest.at(0);
				cart_6d_dest[1] = this->slv_cart.q_dest.at(1);
				cart_6d_dest[2] = this->slv_cart.q_dest.at(2);
				// orientation
				KDL::Frame temp_frame;
				conversions::vectorToKDLFrame(this->slv_cart.q_dest, temp_frame);
				temp_frame.M.GetRPY(cart_6d_dest[3], cart_6d_dest[4], cart_6d_dest[5]);

				variableInterpolator(cart_6d_dest, this->cart_6d_last, this->v_cart_6d_last, cart_6d_cmd,
						v_cart_6d_now,  cart_6d_min, cart_6d_max, cart_6d_v_max, cart_6d_a_max, this->destination_reached, this->fo->dt_param) ;

//				cout << "cart_6d_dest " << cart_6d_dest<< endl;
//				cout << "cart_6d_cmd " << cart_6d_dest<< endl;

				temp_frame.M = KDL::Rotation::RPY(cart_6d_cmd[3],cart_6d_cmd[4],cart_6d_cmd[5]);
				temp_frame.p.data[0] = cart_6d_cmd[0];
				temp_frame.p.data[1] = cart_6d_cmd[1];
				temp_frame.p.data[2] = cart_6d_cmd[2];

//				cout << "cart_6d_dest " << cart_6d_dest << endl;
//				cout << "cart_6d_cmd " << cart_6d_cmd << endl;
				conversions::KDLFrameToVector(temp_frame, this->slv_cart.q_cmd);

				this->cart_6d_last = cart_6d_cmd;
				this->v_cart_6d_last = v_cart_6d_now;

				this->new_cart_dest = false;

			}
			else{
				this->cart_6d_last[0] = this->slv_cart.q_curr.at(0);
				this->cart_6d_last[1] = this->slv_cart.q_curr.at(1);
				this->cart_6d_last[2] = this->slv_cart.q_curr.at(2);
			}
			break;


		case 3:
			/////////////////////////////// TRACKING MOTION MODE /////////////////////////////////////////////////////////////////
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Pose data is read from the port, IK is performed. No check is done.
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			if (!this->cart_dest_port.connected()) {
				log(Error) << this->cart_dest_port.getName() << " not ready" << endlog();
				break;
			}
			if (this->cart_dest_port.read(this->tmp_pose_msg) != RTT::NoData){
				conversions::poseMsgToVector(this->tmp_pose_msg, this->tmp_cart_vec);
				this->mstr_pose_curr = this->tmp_cart_vec;
				conversions::poseReset(this->tmp_pose_msg);

				///////////////////////////////////////////
				//Inverse Kinematics
				if(conversions::vectorToKDLFrame(this->mstr_pose_curr, this->tmp_frame))
				{
					if (this->kine->ik(this->tmp_frame, this->robot_config,  this->ns_param.at(0), this->tmp_joint_vec)) {
						this->slv_jnt.q_dest = tmp_joint_vec;
						//							this->trackingSupervisor(this->slv_jnt.q_cmd, this->slv_jnt.q_dest, this->slv_jnt.q_last , this->slv_jnt.v_max, this->dt);
						//							this->trackingSupervisor(this->slv_jnt.q_cmd, this->slv_jnt.q_dest, this->slv_jnt.q_cmd_last ,this->slv_jnt.v_curr, this->slv_jnt.v_max, this->slv_jnt.a_max, this->dt);
						this->slv_jnt.q_cmd = this->slv_jnt.q_dest;
						this->destination_reached = false;
					}
					else {
						log(RTT::Warning) << "No good pose of the robot; no solution of the inverse kinematics: due to bad target or bad configuration or bad nullspace parameter" << endlog();
					}
				}
			}
			break;

		case 4:

			//////////////////////////////////////////////// TELEOP MOTION MODE ///////////////////////////////////////////////////
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// reads pose from the haptic device port.
			// Applies the clutching and transformations
			// Force is read from the force sensor, it is filtered, scaled and sent to the haptic device
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			if (!this->cart_dest_port.connected()) {
				log(Error) << this->cart_dest_port.getName() << " not ready" << endlog();
				break;
			}

			//-------------------------------------------------------------------------------------
			// Force bias
			//Estimating current bias of the force sensor by averaging 100 consecutive samples
			if(this->to->force_feedback_on && this->force_from_slave_port.read(this->tmp_wrench) == RTT::NewData){
				this->to->calculateForceBias(this->tmp_wrench);
			}



			//When the clutch is engaged slave and master are coupled.
			//			if (this->master_button.data == 1 && force_bias_computed){
			if (this->master_clutch.data == 1 ){

				//			if (force_bias_computed){
				if (this->cart_dest_port.read(this->tmp_pose_msg) != RTT::NoData){
					conversions::poseMsgToVector(this->tmp_pose_msg, this->tmp_cart_vec);
					this->mstr_pose_curr = this->tmp_cart_vec;
					conversions::poseReset(this->tmp_pose_msg);
					conversions::vectorToKDLFrame(this->mstr_pose_curr, this->mstr_frame_curr);


					if(this->clutch_first_time){
						// Saving the pose of the master and slave when the clutch is engaged for the first time for incremental pos
						this->mstr_pose_init = this->mstr_pose_curr;
						this->slv_cart.q_init = this->slv_cart.q_curr;
						conversions::vectorToKDLFrame(this->mstr_pose_init, this->mstr_frame_init);
						conversions::vectorToKDLFrame(this->slv_cart.q_init, this->slv_frame_init);

						this->first_engagement_counter_rpy = 0.5 * 1/this->fo->dt_param;// seconds first engagement integration;
						this->first_engagement_counter_pos = 0.5 * 1/this->fo->dt_param;// seconds first engagement integration;

						this->slv_jnt.q_cmd_last = this->slv_jnt.q_curr; // Used in the tracking supervisor
						this->psi_last[0] = this->ns_param.at(0); // for the redundancy handler

						// Setting the initial value for the averaging of the pos and orientation as that of the slave taken in
						// master's ref frame since the master may move a bit when the clutch is not engaged, it is safer to start
						// the averaging with the pos and ori of the slave itself.
						KDL::Rotation temp_slv_rot =  this->mstr_to_slv_frame.M.Inverse() * this->slv_frame_curr.M * this->master_to_tool_orient_frame.M.Inverse();
						temp_slv_rot.GetRPY(this->mstr_rpy_avg[0],this->mstr_rpy_avg[1],this->mstr_rpy_avg[2]);

						this->mstr_deltapos_avg = vec_dbl(3,0.0);

						this->clutch_first_time = false;
					}

					//------------------------Filtering the orientation of master by a simple moving average that has a larger window
					// when the clutch is pressed and then reduces to a constant window set in the properties
					if(this->first_engagement_counter_rpy >= this->rpy_avg_n){
						this->rpy_avg_n_variable = this->first_engagement_counter_rpy;
						this->first_engagement_counter_rpy--;
					}

					vec_dbl	rpy_temp = vec_dbl(3,0.0);
					this->mstr_frame_curr.M.GetRPY(rpy_temp[0],rpy_temp[1],rpy_temp[2]);

					for(int i= 0; i<3 ; i++){
						this->mstr_rpy_avg[i] -= this->mstr_rpy_avg[i] / this->rpy_avg_n_variable;
						this->mstr_rpy_avg[i] += rpy_temp[i] / this->rpy_avg_n_variable;
					}
					KDL::Rotation avg_rot = KDL::Rotation::RPY(this->mstr_rpy_avg[0],this->mstr_rpy_avg[1],this->mstr_rpy_avg[2]);
					this->tmp_frame.M = avg_rot;

					//------------------------Filtering the position of master by a simple moving average that has a larger window
					if(this->first_engagement_counter_pos >= this->pos_avg_n){
						this->pos_avg_n_variable = this->first_engagement_counter_pos;
						this->first_engagement_counter_pos--;
					}
					for(int i= 0; i<3 ; i++){
						this->mstr_deltapos_avg[i] -= this->mstr_deltapos_avg[i] / this->pos_avg_n_variable;
						this->mstr_deltapos_avg[i] += (this->mstr_frame_curr.p[i]-this->mstr_frame_init.p[i]) / this->pos_avg_n_variable;
						this->tmp_frame.p[i] = this->mstr_deltapos_avg[i];
						if(int(this->pos_avg_n_variable) % 50 ==0)	cout << "pos_avg_n_variable " << this->pos_avg_n_variable << endl;
					}

					//------------------------  REFERENCE FRAME TRANSFORMATIONS
					//DELTA POSITION Reference frame transformation from master to slave
					this->tmp_frame.p = this->mstr_to_slv_frame.M * this->tmp_frame.p ;

					//ORIENTATION Reference frame transformation from master to slave and the desired tool orientation
					// with respect to the haptic device handle
					//					this->tmp_frame.M = this->mstr_to_slv_frame.M.Inverse() * this->tmp_frame.M * this->mstr_to_slv_frame.M;
					this->tmp_frame.M = this->mstr_to_slv_frame.M * this->tmp_frame.M *this->master_to_tool_orient_frame.M;

					//------------------------ Position is incremental
					if(this->teleop_pos_coupled) this->slv_frame_dest.p = this->slv_frame_init.p + this->tmp_frame.p;
					else 						 this->slv_frame_dest.p = this->slv_frame_init.p;

					//------------------------ Orientation is absolute
					if(this->teleop_ori_coupled) this->slv_frame_dest.M	= this->tmp_frame.M;
					else	  					 this->slv_frame_dest.M	= this->slv_frame_init.M;

					conversions::KDLFrameToVector(this->slv_frame_dest, this->slv_cart.q_dest);

					//------------------------ Supervising the desired motion
					//this->trackingSupervisor(this->slv_cart.q_cmd, this->slv_cart.q_dest, this->slv_cart.q_curr , this->slv_cart.v_max, this->dt);
					this->slv_cart.q_cmd = this->slv_cart.q_dest; // not supervising

					//------------------------ Writing the commanded Cartesian pose (for analysis purposes)
					conversions::vectorToPoseMsg(this->slv_cart.q_cmd, this->tmp_pose_msg);
					this->cart_command_port.write(this->tmp_pose_msg);

					//-------------------------------------------------------------------------------------
					// Force measurement, scaling and transformation.
					if (this->to->force_feedback_on)//Measured force
						this->force_from_slave_port.read(this->tmp_wrench);

					this->mstr_wrench_cmd = this->to->getForceFeedback(this->slv_frame_curr, this->mstr_wrench_cmd);
					// writing force on the port
					this->force_to_master_port.write(this->mstr_wrench_cmd);

				}
				else
					log(Error) << " No new readings from: " << this->cart_dest_port.getName()  << endlog();
			}
			else { //If the clutch is released
				this->clutch_first_time = true;
				this->destination_reached = true;
			}

			break;

//		case 5:
//			/////////////////////////////// TEEEEEEEEEEEEEST MODE ///////////////////////////////
//			//////////////////////////////////////////////////////////////////////////////////////
//		{
//			KDL::Vector elbowtemp;
////			RTT::os::TimeService::ticks ticks_temp;
//
////			vector<double> psi_range(2,0.0);
////			psi_range[0] = -M_PI;
////			psi_range[1] =  M_PI;
//////			vector<double> my_psi;
////			double psi_step = M_PI/180;
//
//			vector<double> psi_chosen(1,0.0);
//			vector<double> psi_cmd(1,0.0);
//
//			//double psi_cmd_step = 0.001;
//			// Filling in the Psi vector
////			my_psi.push_back(psi_range[0]);
////			while(my_psi.back() < psi_range[1]){
////				my_psi.push_back(my_psi.back()+psi_step);
////			}
////			//Removeing the last 2 elements
////			my_psi.pop_back();
////			my_psi.pop_back();
////
////			vector<vector<double> > six_joints(my_psi.size(), vector<double>(6));
//
////			double joint_four = 0.0;
//////			cout << "my_psi starts from  " << my_psi[0] << " to  " << my_psi.back()  <<endl;
////			double kun_elapset = 0.0;
////			double my_elapset = 0.0;
////			double fk_elapset = 0.0;
////			double redCircle_elapset= 0.0;
////			double n_traj = 500;
//
//
//
//			static const double arr[] = {0.2, 0.8, 0.5, -1.0, 0.5, 1.0, 0.9};
//			std::vector<double> temp_joints (arr, arr + sizeof(arr) / sizeof(arr[0]) );
//			if(this->first_time_temp){
//				LWR4_Kinematics::FK(temp_joints, this->temp_config, this->psi_last[0], this->tool_zlength, this->initmatrix);
//
//				this->targetmatrix  = this->slv_frame_curr;
//				this->posit_dest[0] = this->initmatrix.p[0] + 0.09;
//				this->posit_dest[1] = this->initmatrix.p[1] + 0.4;
//				this->posit_dest[2] = this->initmatrix.p[2] - 0.1;
//
////				this->posit_init[0] = this->initmatrix.p[0];
////				this->posit_init[1] = this->initmatrix.p[1];
////				this->posit_init[2] = this->initmatrix.p[2];
//				this->posit_last[0] = this->slv_cart.q_curr[0];
//				this->posit_last[1] = this->slv_cart.q_curr[1];
//				this->posit_last[2] = this->slv_cart.q_curr[2];
//
////				if( this->slave_cart_port.read(this->tmp_pose_msg) == RTT::NewData) {
////					conversions::poseMsgToVector(this->tmp_pose_msg, this->tmp_cart_vec);
////				}
//				printVector(this->posit_dest, " this->posit_dest " );
//				cout << " d " << endl;
//				cout << this->posit_dest << endl;
//
//			}

//
//			//			this->setPTPJointDestination(temp_joints);
//
//			vec_dbl v_max_temp(3, 0.1);
//			vec_dbl a_max_temp(3, 0.1);
//
//
//			//			for(int iiter = 0; iiter<n_traj; iiter++){
//			if(!this->dest_reached_temp){
////				this->P2PInterpolator(this->posit_dest, this->posit_init, this->posit_cmd, v_max_temp, a_max_temp, this->h_temp, this->T_temp,   this->interp_counter_temp,  this->new_dest_temp ,  this->dest_reached_temp) ;
//				variableInterpolator(this->posit_dest, this->posit_last, this->v_last,this->posit_cmd,
//						this->v_cmd,  this->slv_cart.q_min, this->slv_cart.q_max,this->slv_cart.v_max, this->slv_cart.a_max, this->dest_reached_temp, this->dt) ;
//
//				if(this->mode4_counter < 10){
//					cout << "  vx = " << (this->posit_cmd[0] - this->posit_last[0])/this->dt <<
//							"  vy = " << (this->posit_cmd[1] - this->posit_last[1])/this->dt <<
//							"  vz = " << (this->posit_cmd[2] - this->posit_last[2])/this->dt <<endl;
//					cout << " ax = " << (this->v_cmd[0] - this->v_last[0])/this->dt <<
//							" ay = " << (this->v_cmd[1] - this->v_last[1])/this->dt <<
//							" az = " << (this->v_cmd[2] - this->v_last[2])/this->dt <<endl;
////					printVector(this->posit_last, " this->posit_last " );
////					printVector(this->posit_cmd, " this->posit_cmd " );
//
//				}
////				cout << "dest_reached_temp " << dest_reached_temp <<endl;
//				this->posit_last = this->posit_cmd;
//				this->v_last = this->v_cmd ;
//
//				this->mode4_counter +=1;
//
//				this->targetmatrix.p[0] = this->posit_cmd[0];
//				this->targetmatrix.p[1] = this->posit_cmd[1];
//				this->targetmatrix.p[2] = this->posit_cmd[2];
////
////				cout << "this->targetmatrix.p[0] : " << this->targetmatrix.p[0]<< endl;
////				cout << "this->targetmatrix.p[1] : " << this->targetmatrix.p[1]<< endl;
////				cout << "this->targetmatrix.p[2] : " << this->targetmatrix.p[2]<< endl;
//
//				vector<double> valid_psis;
//				vector<double> valid_psis2;
//
//				vector<vector<double> > valid_six_joints;
//				vector<vector<double> > valid_six_joints2;
//				std::vector<double> psi_arc(2,0.0);
////				double current_psi = 0.0;
//
//				//		ticks_temp= os::TimeService::Instance()->getTicks();
//
//				LWR4_Kinematics::validJointsForCurrentArc(this->targetmatrix, this->temp_config, this->psi_last[0], this->tool_zlength,
//						psi_arc);
//				//		redCircle_elapset = os::TimeService::Instance()->secondsSince(ticks_temp);
//				//		std::cout << "validJoints2 (uS) : " << redCircle_elapset*1000000<< std::endl;
//
//				//		ticks_temp= os::TimeService::Instance()->getTicks();
//				//		LWR4_Kinematics::validJointsForPsiVector( this->targetmatrix, temp_config, my_psi, this->tool_zlength, joint_four, valid_six_joints, valid_psis);
//				//		redCircle_elapset = os::TimeService::Instance()->secondsSince(ticks_temp);
//				//		std::cout << "validJoints1 (uS) : " << redCircle_elapset*1000000<< std::endl;
//
//
//				psi_chosen[0] = psi_arc[0] + (psi_arc[1]- psi_arc[0])/2;
//				psi_cmd = psi_chosen;
//				vector<double> psi_v_max (1, 0.05);
//				vector<double> psi_a_max (1, 0.05);
//				vector<double> psi_q_min (1, 10.0);
//				vector<double> psi_q_max (1, 10.0);
//
//				bool psi_flag_temp = 0;
////				if(psi_cmd- this->psi_last[0] >  psi_cmd_step)	psi_cmd = this->psi_last[0] + psi_cmd_step;
////				if(psi_cmd- this->psi_last[0] < -psi_cmd_step)	psi_cmd = this->psi_last[0] - psi_cmd_step;
//				variableInterpolator(psi_chosen, psi_last, this->psi_v_last, psi_cmd,
//						this->psi_v_cmd,  psi_q_min, psi_q_max ,psi_v_max, psi_a_max, psi_flag_temp, this->dt) ;
//
////				cout << "psi_chosen " <<  psi_chosen <<" psi_last " << this->psi_last << "  cmd " << psi_cmd << endl;
//
//				this->psi_last = psi_cmd;
//				this->psi_v_last = this->psi_v_cmd;
//
//				//		ticks_temp= os::TimeService::Instance()->getTicks();
//				if(LWR4_Kinematics::IK(this->targetmatrix, this->temp_config, psi_cmd[0], this->tool_zlength,  this->tmp_joint_vec)){
//					//		my_elapset = os::TimeService::Instance()->secondsSince(ticks_temp);
//					//		std::cout << "elapset My(uS) :  " << my_elapset*1000000<< std::endl;
//					this->slv_jnt.q_dest = tmp_joint_vec;
//					this->slv_jnt.q_cmd = this->slv_jnt.q_dest;
//					this->destination_reached = false;
//
//				}
//				else {
//					log(RTT::Warning) << "No good pose of the robot; no solution of the inverse kinematics: due to bad target or bad configuration or bad nullspace parameter" << endlog();
//				}
//
//				this->first_time_temp = false;
//			}
//
//		}
//
//			break;
		default:
			log(RTT::Error) << "Wrong motion mode selected " << this->motion_mode << ", switching to 0 instead" << endlog();
			this->changeMotionMode(0);
			break;
		}

		///////////////////////////////////////////////////////////////////////////////////
		//Redundancy handling and inverse Kinematics for modes 2 and 4.
		////////////////////////////////////////////////////////////////////////////////////
		if((this->motion_mode==2 || this->motion_mode==4) && (!this->destination_reached || this->master_clutch.data == 1) ){
			KDL::Vector elbowtemp;
			vector<double> psi_chosen(1,0.0);
			vector<double> psi_cmd(1,0.0);
			std::vector<double> psi_arc(2,0.0);

			this->targetmatrix  = this->slv_frame_curr;
			this->targetmatrix.p[0] = this->slv_cart.q_cmd.at(0);
			this->targetmatrix.p[1] = this->slv_cart.q_cmd.at(1);
			this->targetmatrix.p[2] = this->slv_cart.q_cmd.at(2);

			//		vector<double> valid_psis;
			//		vector<double> valid_psis2;
			//
			//		vector<vector<double> > valid_six_joints;
			//		vector<vector<double> > valid_six_joints2;
			//		ticks_temp= os::TimeService::Instance()->getTicks();
			this->kine->validJointsForCurrentArc(this->targetmatrix, this->robot_config, this->psi_last[0],
					psi_arc);


			//		redCircle_elapset = os::TimeService::Instance()->secondsSince(ticks_temp);
			//		std::cout << "validJoints2 (uS) : " << redCircle_elapset*1000000<< std::endl;

			//		ticks_temp= os::TimeService::Instance()->getTicks();
			//		LWR4_Kinematics::validJointsForPsiVector( this->targetmatrix, temp_config, my_psi, this->tool_zlength, joint_four, valid_six_joints, valid_psis);
			//		redCircle_elapset = os::TimeService::Instance()->secondsSince(ticks_temp);
			//		std::cout << "validJoints1 (uS) : " << redCircle_elapset*1000000<< std::endl;

			//			// //manipulability
			//			std::vector<KDL::Frame> joint_frames2(this->num_joints + 1, KDL::Frame::Identity());
			//			KDL::Jacobian jac2(7);
			//			double manip_p, manip_n, temp_double;
			//			unsigned int temp_int;
			//			this->kine->ik(this->targetmatrix, this->robot_config,  this->psi_last[0]+M_PI/180, this->tmp_joint_vec);
			//			this->kine->fk_all(this->tmp_joint_vec, temp_int, temp_double, joint_frames2);
			//			this->kine->jacobian(joint_frames2, jac2);
			//			this->kine->getManipulabilityIdx(jac2,0,manip_p);
			//
			//			this->kine->ik(this->targetmatrix, this->robot_config,  this->psi_last[0]-M_PI/180, this->tmp_joint_vec);
			//			this->kine->fk_all(this->tmp_joint_vec, temp_int, temp_double, joint_frames2);
			//			this->kine->jacobian(joint_frames2, jac2);
			//			this->kine->getManipulabilityIdx(jac2,0,manip_n);
			//
			//			if((manip_p-manip_n)>0)
			//				psi_chosen[0] = this->psi_last[0]+M_PI/180;
			//			else
			//				psi_chosen[0] = this->psi_last[0]-M_PI/180;
			//
			//			if (psi_chosen[0]<psi_arc[0])
			//				psi_chosen[0] = psi_arc[0] ;
			//			else if((psi_chosen[0]>psi_arc[1]))
			//				psi_chosen[0] = psi_arc[1] ;




			psi_chosen[0] = psi_arc[0] + (psi_arc[1]- psi_arc[0])/2;
			psi_cmd = psi_chosen;
			vector<double> psi_v_max (1, 0.3);
			vector<double> psi_a_max (1, 0.1);
			vector<double> psi_q_min (1, -10.0);
			vector<double> psi_q_max (1, 10.0);

			bool psi_flag_temp = 0;
			//				if(psi_cmd- this->psi_last[0] >  psi_cmd_step)	psi_cmd = this->psi_last[0] + psi_cmd_step;
			//				if(psi_cmd- this->psi_last[0] < -psi_cmd_step)	psi_cmd = this->psi_last[0] - psi_cmd_step;
			variableInterpolator(psi_chosen, this->psi_last, this->psi_v_last, psi_cmd,
					this->psi_v_cmd,  psi_q_min, psi_q_max ,psi_v_max, psi_a_max, psi_flag_temp, this->fo->dt_param) ;

			//				cout << "psi_chosen " <<  psi_chosen <<" psi_last " << this->psi_last << "  cmd " << psi_cmd << endl;

//			cout << "psi_last" << psi_last << endl;
//			cout << "this->ns_param.at(0)" << this->ns_param.at(0) << endl;
//			cout << "psi_chosen" << psi_chosen << endl;
//			cout << "psi_cmd" << psi_cmd << endl;
			this->psi_last = psi_cmd;
			this->psi_v_last = this->psi_v_cmd;


			//////////////////////////////////////////////////////////////////////////////////////////////////////
			//Inverse Kinematics
			if(conversions::vectorToKDLFrame(this->slv_cart.q_cmd, this->tmp_frame)) {
				if (this->kine->ik(this->tmp_frame, this->robot_config,  psi_cmd[0], this->tmp_joint_vec)) {
					this->slv_jnt.q_dest = this->tmp_joint_vec;

//					this->destination_reached = false;
				}
				else {
					log(RTT::Warning) << "No good pose of the robot; no solution of the inverse kinematics: due to bad target or bad configuration or bad nullspace parameter" << endlog();
					this->tmp_joint_vec = std::vector<double>(this->num_joints, 0.0); 		//Reset temporary variables
				}
			}

			// if in the tele-operation case the commanded joint is too far, something is wrong.
			// to prevent the robot from locking interpolate IN JOINTS towards the destination.
			// Mainly for debug.
			if(this->motion_mode==4){
				if(this->teleop_interpolate_done){
					this->slv_jnt.q_cmd =this->slv_jnt.q_dest;

					// find the maximum joint displacement
					for (unsigned int iter=0; iter < this->num_joints; iter++){
						this->tmp_joint_vec.at(iter) = (this->slv_jnt.q_cmd.at(iter) - this->slv_jnt.q_curr.at(iter));
					}
					double max_diff = *( max_element(this->tmp_joint_vec.begin() , this->tmp_joint_vec.end()) );

					// if the largest asked displacement is bigger than 3 degrees something is wrtong, so interpolate.
					if (max_diff> 3*M_PI/180){
						this->teleop_interpolate_done = false;

					}
			}
				// no else. Check again
				if(!this->teleop_interpolate_done){
					cout << "Hold the clutch and don't move the master until this message stops!!" << endl;
					variableInterpolator(
							this->slv_jnt.q_dest, this->slv_jnt.q_cmd_last, this->slv_jnt.v_last,
							this->slv_jnt.q_cmd, this->slv_jnt.v_curr,  this->slv_jnt.q_min,
							this->slv_jnt.q_max, this->slv_jnt.v_max, this->slv_jnt.a_max,
							this->teleop_interpolate_done , this->fo->dt_param) ;
				}


			}
			else // if motion_mode is 2
				this->slv_jnt.q_cmd =this->slv_jnt.q_dest;


		}
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Writing joint position on the port
		if(this->motion_mode!=0 && (!this->destination_reached || this->master_clutch.data == 1) ){
			if( conversions::vectorToJointPos(this->slv_jnt.q_cmd, this->tmp_joint_pos))
				this->joint_command_port.write(this->tmp_joint_pos);
		}
		// 		//this is not a good idea!
		//		this->clipToLimits(this->slv_cart.q_cmd, this->slv_cart.q_min, this->slv_cart.q_max);
		//		this->clipToLimits(this->slv_jnt.q_cmd, this->slv_jnt.q_min, this->slv_jnt.q_max);


		//-------------------------------------------------------------------------------------
		// calculating time related stuff
		//-------------------------------------------------------------------------------------
		// Observe the loop frequency
		this->fo->check();

		//-------------------------------------------------------------------------------------
		// Estimate Cartesian velocity of the robot
		//-------------------------------------------------------------------------------------
		// Simple first order method turns out to be good enough. I compared it with FOAW and
		// saw no improvement so I switched back to first order.
		// Note that I am calculating the velocity based on the commanded position not current one
		for (unsigned int iter=0; iter < this->num_cart_p_var; iter++){
			this->slv_cart.v_curr.at(iter) = (this->slv_cart.q_cmd.at(iter) - this->slv_cart.q_cmd_last.at(iter)) / this->fo->dt_param;
		}

		//		// FOAW
		//		double v_foaw;
		//		int ws;
		//		this->v_filt->filter(this->slv_cart.q_cmd[0], v_foaw, ws);

		conversions::vectorToTwist(this->slv_cart.v_curr, this->tmp_twist);
		// testing:::: writing the manip indexes
		this->tmp_twist.angular.x = manipA;
		this->tmp_twist.angular.y = manipT;
		this->tmp_twist.angular.z = manipR;

		this->port_cart_twist.write(this->tmp_twist);

		this->slv_jnt.v_last = this->slv_jnt.v_curr;

		//joint vel
		for (unsigned int iter=0; iter < this->num_joints; iter++){
			this->slv_jnt.v_curr.at(iter) = (this->slv_jnt.q_cmd.at(iter) - this->slv_jnt.q_cmd_last.at(iter)) / this->fo->dt_param;
		}


		////////////////////////////////////////////////////////////////////////////////////////
		//Set last values
		this->slv_cart.q_cmd_last = this->slv_cart.q_cmd;
		this->slv_cart.q_last = this->slv_cart.q_curr;

		this->slv_jnt.q_last = this->slv_jnt.q_curr;
		this->slv_jnt.q_cmd_last = this->slv_jnt.q_cmd;


		// Measuring the computation time
		this->fo->t_computation = os::TimeService::Instance()->secondsSince(this->fo->computation_time_from);

	}
}



void Teleop::stopHook() {
	std::cout << "Teleop executes stopping !" <<std::endl;

}

void Teleop::cleanupHook() {
	delete this->fo;
	delete this->kine;
	std::cout << "Teleop cleaning up !" <<std::endl;
}




//-------------------------------------------------------------------------------------
// freqObserver constructor
//-------------------------------------------------------------------------------------
freqObserver::freqObserver(double _dt_param, unsigned int _dt_init_steps, unsigned int _avg_steps){

	dt_param=_dt_param;
	dt_init_steps= _dt_init_steps;
	avg_steps = _avg_steps;
	//initializing the measured values with the nominal one
	dt_msrd_avg=_dt_param;
	dt_msrd_last=_dt_param;
	t_computation=0;

	dt_init_counter=0;
	computation_time_from = 0;
	loop_time_from = 0;

}

//-------------------------------------------------------------------------------------
// freqObserver check the frequency
//-------------------------------------------------------------------------------------
void freqObserver::check(){

	if(dt_init_counter <=  dt_init_steps){
		dt_init_counter++;
		loop_time_from = os::TimeService::Instance()->getTicks();
	}
	else{

		dt_msrd_last = os::TimeService::Instance()->secondsSince(loop_time_from);
		loop_time_from = os::TimeService::Instance()->getTicks();

		dt_msrd_avg-= dt_msrd_avg/avg_steps;
		dt_msrd_avg+= dt_msrd_last/avg_steps;

		// Warn if the average measured loop time is more than %10 different than the expected
		if(fabs(dt_msrd_avg - dt_param)> dt_param/10)
			log(RTT::Warning) << "ATTENTION! Frequency inconsistency!! My average loop time is:"<< dt_msrd_avg << " While I expect: "<< dt_param << endlog();
	}


}
/*
/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
////// ************************************** Teleop FUNCTIONS *************************************************\\\\\\\\\
/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
 */
///------------------------------------------------------------------------------------------------------------------------------------


void Teleop::startPalpation(){
	this->palpation_on = true;
	this->changeMotionMode(2);
	this->palpation_first_time = true;
}

void Teleop::stopPalpation(){
	this->palpation_on = false;
	this->palpation_first_time = false;
	this->palp_point_status = 0;
	this->destination_reached = true;
	this->changeMotionMode(1);
	this->setPTPCartDestination(this->palpation_home_3dpose_prop);
}


bool Teleop::startMotion() {
	Logger::In in(this->getName());

	if (!this->isRunning()) {
		log(RTT::Error) << "Component not running. Motion mode not started" << endlog();
		return false;
	}
	if (this->motionOn) {
		log(RTT::Warning) << "Motion is already on!" << endlog();
		return false;
	}

	this->motionOn = true;
	log(RTT::Warning)<<  "Motion mode is:  " << this->motion_mode <<  endlog();

	return true;
	//	log(RTT::Info)<<  "pi_cart_dest.at(0)  " << this->pi_cart_dest.at(0)<<  "pi_cart_init.at(0)  " << this->slv_cart.q_curr.at(0)<<  "slv_cart.q_curr.at(0)  " << this->slv_cart.q_cmd.at(0)<<  endlog();

}

///------------------------------------------------------------------------------------------------------------------------------------
bool Teleop::changeMotionMode(const int mode) {
	Logger::In in(this->getName());

	if (mode < 0 || mode > 4) {
		log(RTT::Error) << "Mode " << mode << " Mode not acceptable. Legal modes are 1 (for PTP) or 2 (for Tracking)" << endlog();
		return false;
	}
	string mode_name;
	switch(mode){
	case 0: mode_name = "Idle";													break;
	case 1:	mode_name = "Cartesian and Joint PTP with joint trajectory";		break;
	case 2:	mode_name = "Cartesian PTP with Cartesian trajectory";				break;
	case 3:	mode_name = "Tracking";												break;
	case 4:	mode_name = "Tele-operation";										break;
	}
	this->motion_mode = mode;
	this->motion_mode_prop = mode;
	log(RTT::Info) << "Changed motion mode to " << mode << " = " << mode_name << endlog();
	return true;
}

///------------------------------------------------------------------------------------------------------------------------------------
bool Teleop::stopMotion() {

	// Action to be performed to stop the movement;
	Logger::In in(this->getName());

	if (!this->isRunning()) {
		log(RTT::Error) << "Component not running; not stopping the interpolation" << endlog();
		return false;
	}

	if (!this->motionOn) {
		log(RTT::Warning) << "The interpolator is not running." << endlog();
		return false;
	}

	if (!this->destination_reached) {
		log(RTT::Warning) << "The target was not reached. Stopping at the last sent joint command." << endlog();
		this->destination_reached = true;
		//this->setPTPJointDestination(this->slv_jnt.q_cmd_last);
	}

	this->motionOn = false;
	this->to->force_bias_computed = false;

	log(RTT::Info) << "Motion mode stopped!" << endlog();
	return true;

}

///------------------------------------------------------------------------------------------------------------------------------------
void Teleop::positionCoupling(const bool input){
	Logger::In in(this->getName());
	if(this->teleop_pos_coupled == input){
		log(RTT::Info) << "No change needed." << endlog();
		return;
	}
	this->teleop_pos_coupled = input;
	if(this->teleop_pos_coupled)	log(RTT::Info) << "The positions of master and slave are coupled now." << endlog();
	if(!this->teleop_pos_coupled)	log(RTT::Info) << "Uncoupled the positions of master and slave" << endlog();
}

///------------------------------------------------------------------------------------------------------------------------------------
void Teleop::orientationCoupling(const bool input){
	Logger::In in(this->getName());
	if(this->teleop_ori_coupled == input){
		log(RTT::Info) << "No change needed." << endlog();
		return;
	}
	this->teleop_ori_coupled = input;
	if(this->teleop_ori_coupled)	log(RTT::Info) << "The orientations of master and slave are coupled now." << endlog();
	if(!this->teleop_ori_coupled)	log(RTT::Info) << "Uncoupled the orientations of master and slave" << endlog();
}
///-------------------------------------------------------------------------------------------------------------------------------------
bool Teleop::setPTPCartDestination(const std::vector<double>& vars) {
	log(RTT::Debug) << "Setting Cartesian destination" << endlog();
	if (this->destination_reached) {

		if (vars.size() == 3) {
			//position is input and orientation is the current EE orientation
			this->slv_cart.q_dest.at(0) = vars.at(0);
			this->slv_cart.q_dest.at(1) = vars.at(1);
			this->slv_cart.q_dest.at(2) = vars.at(2);
			this->slv_cart.q_dest.at(3) = this->slv_cart.q_curr.at(3);
			this->slv_cart.q_dest.at(4) = this->slv_cart.q_curr.at(4);
			this->slv_cart.q_dest.at(5) = this->slv_cart.q_curr.at(5);
			this->slv_cart.q_dest.at(6) = this->slv_cart.q_curr.at(6);
		}
		else if(vars.size() == 6){
			KDL::Frame temp_frame;
			temp_frame.p.data[0] = vars.at(0);
			temp_frame.p.data[1] = vars.at(1);
			temp_frame.p.data[2] = vars.at(2);
			temp_frame.M = KDL::Rotation::RPY(vars.at(3)/180*M_PI,vars.at(4)/180*M_PI,vars.at(5)/180*M_PI);
			conversions::KDLFrameToVector(temp_frame, this->slv_cart.q_dest);
		}
		else{
			log(RTT::Info) << "Wrong input dimension. Destination must be either a Cartesian position, i.e. x,y,z or Pose i.e. x,y,z,r,p,y." << endlog();
			return false;
		}

		//Set the flags
		if(this->motion_mode!=1) this->destination_reached = false; // WHen motion mode ==1 the flag will be set later in setPTPJointDestination
		this->new_cart_dest = true;
		return true;
	}
	else{
		log(RTT::Info) << "Cannot set a new destination. I have not reached the previous destination yet. Go have a coffee or something!" << endlog();
		return false;
	}
}


///------------------------------------------------------------------------------------------------------------------------------------
bool Teleop::setPTPJointDestination(std::vector<double> vars) {
	Logger::In in(this->getName());
	log(RTT::Debug) << "Setting joints destination" << endlog();

	//Check for mismatches
	if (vars.size() != this->num_joints) {
		log(RTT::Error) << "Joint destination with wrong dimension" << endlog();
		return false;
	}

	if(!this->clipToLimits(vars, this->slv_jnt.q_min, this->slv_jnt.q_max)){
		log(RTT::Error)<< "Not able to clip joints to the limits" << endlog();
		return false;
	}
	if(areVectorsEqual(vars, this->slv_jnt.q_curr, INTERPOLATION_TOLERANCE)){
		log(RTT::Warning)<< "Commanded joint positions are equal to current positions." << endlog();
		return false;
	}

	if (this->motionOn) {
		if (this->destination_reached){
			//Save the initial values and time
			this->slv_joint_p_interpd_init = this->slv_jnt.q_curr;
//			this->time_init = os::TimeService::Instance()->getTicks();
			this->interp_counter = 1;

			//Set the destination and the flags
			this->slv_jnt.q_dest = vars;
			this->new_joint_dest = true;
			this->destination_reached= false;
			return true;
		}
		else
		{
			log(RTT::Warning) << "setPTPJointDestination: Cannot set a new destination while the previous one is not reached. What's with the rush!!" << endlog();
			return false;
		}
	}
	else{
		log(RTT::Error) << "setPTPJointDestination: Cannot set a destination while motion is off." << endlog();
		return false;
	}
}
///------------------------------------------------------------------------------------------------------------------------------------
void Teleop::switchForceFeedback(bool input){
	Logger::In in(this->getName());
	if(this->to->force_feedback_on == input){
		log(RTT::Info) << "No change needed." << endlog();
		return;
	}
	this->to->force_feedback_on = input;
	if(this->to->force_feedback_on)	log(RTT::Info) << "Force feedback is turned On." << endlog();
	if(!this->to->force_feedback_on)	log(RTT::Info) << "Force feedback is turned Off." << endlog();

}

///------------------------------------------------------------------------------------------------------------------------------------
void Teleop::forceFilterSwitch() {
	cout<< "Switching force filter from:" << this->force_filter_on << " to: " << !this->force_filter_on << endl;
	this->force_filter_on = !this->force_filter_on;
}

///------------------------------------------------------------------------------------------------------------------------------------
void Teleop::forceSensorCalib() {
	this->force_bias = KDL::Vector::Zero();
	this->to->force_bias_computed = false;
}

///------------------------------------------------------------------------------------------------------------------------------------
bool Teleop::goHome() {
	return(this->setPTPJointDestination(this->slv_jnt_home));
}

///------------------------------------------------------------------------------------------------------------------------------------
void Teleop::wtf(){

	if(this->motionOn) 	cout << "Motion is ON." << endl;
	else 				cout << "Motion is Off." << endl;
	cout << "Motion mode is: "<< this->motion_mode << endl;
	cout << "		Note: 0 = Idle, 1=PTP in joint, 2=PTP in Cartesian, 3=Tracking, 4=Tele-operation" << endl;
	cout << "Tool length is:" << this->tool_zlength << endl;
	cout << "Number of orientation samples averaged in teleop: " << this->rpy_avg_n << endl;
	cout << "Master/slave position coupling			 : " << this->teleop_pos_coupled << endl;
	cout << "Master/slave orientation coupling		 : " << this->teleop_ori_coupled <<"\n"<<endl;

	cout << "Current config param is                 : " << this->robot_config << endl;
	cout << "Current arm angle is                    : " << this->ns_param.at(0) <<"\n"<< endl;

	cout << "Current joints read from the FRI(rad)   : " << this->slv_jnt.q_curr << endl;
	cout << "Current joints read from the FRI(deg)   : " << conversions::radTodeg(this->slv_jnt.q_curr) << endl;
	cout << "Last commanded joints(deg)              : " << conversions::radTodeg(this->slv_jnt.q_cmd_last) <<"\n"<< endl;

		//	Reading the Cartesian pose of the robot
	if( this->slave_cart_port.read(this->tmp_pose_msg) != RTT::NoData) {
		conversions::poseMsgToVector(this->tmp_pose_msg, this->tmp_cart_vec);
	}
	cout << "Current Cartesian pose read from the FRI: " << this->tmp_cart_vec << endl;
	cout << "Current Cartesian pose (FK from joints) : " << this->slv_cart.q_curr << endl;
	cout << "Last commanded Cartesian                : " << this->slv_cart.q_cmd_last <<"\n"<< endl;

	this->tmp_cart_vec = std::vector<double>(7, 0.0);
	double r,p,y;
	this->slv_frame_curr.M.GetRPY(r,p,y);
	cout << "Current Orientation FK (r, p, y) in degrees        : "<< r*180/M_PI <<"  "<< p*180/M_PI << "  " << y*180/M_PI << endl;
	KDL::Frame temp_frame;
	conversions::vectorToKDLFrame(this->slv_cart.q_cmd_last,temp_frame);
	temp_frame.M.GetRPY(r,p,y);
	cout << "Last commanded Orientation FK (r, p, y) in degrees	: "<< r*180/M_PI <<"  "<< p*180/M_PI << "  " << y*180/M_PI << endl;

}

///------------------------------------------------------------------------------------------------------------------------------------
bool Teleop::clipToLimits(vec_dbl& vars, vec_dbl min_limits, vec_dbl max_limits) {

	if(vars.size() != max_limits.size() || vars.size() != min_limits.size())
		return false;

	for (unsigned int iter = 0; iter < vars.size(); iter++) {
		if (vars.at(iter) < min_limits.at(iter)) {
			log(RTT::Warning) << "Clipping variable " << iter + 1 << "from"<< vars.at(iter) << " to " << min_limits.at(iter) << endlog();
			vars.at(iter) = min_limits.at(iter);
		}
		else if (vars.at(iter) > max_limits.at(iter)) {
			log(RTT::Warning) << "Clipping variable " << iter + 1 << "from"<< vars.at(iter) << " to " << max_limits.at(iter) << endlog();
			vars.at(iter) = max_limits.at(iter);

		}
	}
	return true;

}

///------------------------------------------------------------------------------------------------------------------------------------
bool Teleop::trackingSupervisor(vec_dbl& q_cmd, const vec_dbl q_dest, const vec_dbl q_curr, const vec_dbl qdot_curr, const vec_dbl qdot_max,  const vec_dbl qdotdot_max,  double dt) {
	if(q_curr.size() != q_cmd.size() || q_curr.size() != qdot_max.size()){
		log(RTT::Error) << "Dimension mismatch in trackingSupervisor inputs" << endlog();
		return false;
	}


	double dq_max, h, v_max;

	for (unsigned int iter=0; iter < q_dest.size() ; iter++){

		h = q_dest.at(iter) - q_curr.at(iter);
		// finding the maximum step that can be reached from current velocity using max acceleration
		v_max = qdot_curr.at(iter) +  copysign(qdotdot_max.at(iter), h)* dt;

		if (std::fabs(v_max) > std::fabs(qdot_max.at(iter))){
			v_max = copysign(qdot_max.at(iter), v_max);
		}
		dq_max = v_max * dt ;

		if (std::fabs(h) > std::fabs(dq_max)){
				q_cmd.at(iter) = q_curr.at(iter) + dq_max;

//						log(RTT::Warning) << "trackingSupervisor: at joint #" << iter+1 << " h =  " << fabs(h)<< " is bigger than dq_max: " << dq_max <<endlog();
			log(RTT::Warning) << "trackingSupervisor: joint #" << iter+1 << " h =  " << fabs(h)<< " > dq_max: " << dq_max << " -- Current " << q_curr.at(iter)<< " -- setting " << q_cmd.at(iter)<<" instead of " << q_dest.at(iter) <<endlog();
			log(RTT::Warning) << "v: " << v_max << " qdot_curr " << qdot_curr.at(iter) <<endlog();

		}
		else{
			q_cmd.at(iter) = q_dest.at(iter);
		}
	}

	return true;
}

///------------------------------------------------------------------------------------------------------------------------------------
bool Teleop::P2PInterpolator(const vec_dbl p_dest, const vec_dbl p_init, vec_dbl& p_out, const vec_dbl v_max, const vec_dbl a_max, vec_dbl& h, double& T,   unsigned int& counter, bool& new_dest , bool& dest_reached) {
	Logger::In in(this->getName());

	//The approach is to calculate minimum time for each joint and then to synchronize all the joints
	//at the largest value of the minimum times so that they reach the target at the same time.
	//
	//
	//Given the trajectory q(t), defined between points qi and qf with a travel time of T= tf - ti,
	//its expression in normalized form is as follows:
	//			q(t) = qi + h*s(taw )
	//with:
	//			h = qf - qi
	//			0 â‰¤ Ïƒ(Ï„) â‰¤ 1,
	//			Ï„= (t âˆ’ ti) /T
	//For a polynomial trajectory of degree 3 (Ïƒ(Ï„) = a1*s + a2*s^2 + a3*s^3) with zero initial velocity
	//we have:
	//			Ïƒ(Ï„)  = 3Ï„^2 âˆ’ 2Ï„^3
	//			Ïƒ'(Ï„) = 6Ï„ âˆ’ 6Ï„^2
	//			Ïƒ''(Ï„)= 6 âˆ’ 12Ï„
	//Therefore the maximum velocity and acceleration can be found as:
	//			q'Max = 3*h/(2*T)
	//			q''Max = 6*h/(T^2)
	//If vInitial is not zero, that is if a new target is given while the previous one has not yet been
	//reached, then T would be different. However, since adding vInitial to the computation of T adds
	//too much computations, it seems easier for the moment to assume vInit=0 and find T.
	//After finding the maximum T
	//	Vin = vInit*T/h;  //remember to normalize vInit
	//	a1=Vin;
	//	a2=3-2*Vin;
	//	a3=Vin-2;

	//log(RTT::Debug) << "Running P2PInterpolator " << endlog();

	unsigned int n_var;
	n_var = p_dest.size();

	if(new_dest){

		vec_dbl t_max_vel(n_var, 0.0);
		vec_dbl t_max_acc(n_var, 0.0);

		double t_max_vel_tmp = 0.0;
		double t_max_acc_tmp = 0.0;

		for (unsigned int iter=0; iter < n_var; iter++){

			h.at(iter) = p_dest.at(iter) -p_init.at(iter);
//			cout << h.at(iter) << endl;
			//	3rd degree polynomial
//			t_max_vel_tmp = 3*h.at(iter) / (2*v_max.at(iter));
//			t_max_acc_tmp = 6*h.at(iter) / a_max.at(iter);

			//	5th degree polynomial
			t_max_vel_tmp = 15*h.at(iter) / (8*v_max.at(iter));
			t_max_acc_tmp = 10*std::sqrt(3)*h.at(iter) / (3*a_max.at(iter));

			t_max_vel.at(iter) =std::fabs(t_max_vel_tmp);
			t_max_acc.at(iter) = std::sqrt(std::fabs(t_max_acc_tmp));
		}
		// find the joint that will take the longest
		t_max_vel_tmp = *( max_element(t_max_vel.begin() , t_max_vel.end()) );
		t_max_acc_tmp = *( max_element(t_max_acc.begin() , t_max_acc.end()) );
		T = (t_max_acc_tmp >= t_max_vel_tmp ? t_max_acc_tmp : t_max_vel_tmp);

//		std::cout << " t_max_vel_tmp: " << t_max_vel_tmp << " t_max_acc_tmp: " << t_max_acc_tmp << " T: " << T << " v_max.at(0) " << v_max.at(0)<< " a_max.at(0) " << a_max.at(0) << std::endl;

		double h_tot = 0.0;
		for (unsigned int iter=0; iter < n_var; iter++){
			// Find the total displacement of the variables
			h_tot += fabs(h.at(iter));
			// Check if the destination is changed for this joint. (Prevent division by zero due to h=0)
			if (fabs(h.at(iter)) > INTERPOLATION_TOLERANCE ){

				//If initial velocity is considered
				//				this->a1.at(iter) =v_init.at(iter) * T/h.at(iter);
				//				this->a2.at(iter) =3 - 2 * (v_init.at(iter) * T/h.at(iter));
				//				this->a3.at(iter) =(v_init.at(iter) * T/h.at(iter)) - 2;

				//Assuming Zero initial velocity
				//	3rd degree polynomial
				this->a1.at(iter) = 0.0;
				this->a2.at(iter) = 3.0;
				this->a3.at(iter) = -2.0;
				//	5th degree polynomial
				this->a1.at(iter) = 10.0;
				this->a2.at(iter) = -15.0;
				this->a3.at(iter) = 6.0;

			}
			else{
				this->a1.at(iter) = 0.0;
				this->a2.at(iter) = 0.0;
				this->a3.at(iter) = 0.0;
			}
		}
		if(fabs(h_tot) < INTERPOLATION_TOLERANCE ){
			log(RTT::Warning) << "The commanded destination is the same as current." << endlog();
			dest_reached= true;
			return true;
		}

		new_dest = false;
	}


	double taw = 0.0;
	//taw = (os::TimeService::Instance()->secondsSince(time_init)) / T;  //This turned to be a bad choice. Constant dt
	taw = counter * this->fo->dt_param / T;
	counter++;

	if (taw<=1.0)
	{
		for (unsigned int iter=0; iter < n_var; iter++){

			// Check if the destination is changed for this joint.
			if (fabs(h.at(iter)) > INTERPOLATION_TOLERANCE ){
				//p_out.at(iter) = p_init.at(iter) + h.at(iter) * ( 3*pow(taw,2) -2*pow(taw,3) );

				//	3rd degree polynomial
//				p_out.at(iter) = p_init.at(iter) + h.at(iter) * ( this->a1.at(iter)*taw 	   + this->a2.at(iter)*pow(taw,2) + this->a3.at(iter)*pow(taw,3) );
				//	5th degree polynomial
				p_out.at(iter) = p_init.at(iter) + h.at(iter) * ( this->a1.at(iter)*pow(taw,3) + this->a2.at(iter)*pow(taw,4) + this->a3.at(iter)*pow(taw,5));
			}
			else {
				p_out.at(iter) = p_init.at(iter);
			}
		}
	}
	//else if(taw > 1.0 &&  taw <= 1.0 + 2*this->dt)
	else
	{
		p_out = p_dest;
		dest_reached= true;
		log(RTT::Info) << "Teleop::P2PInterpolator: Reached destination:" << endlog();
	}

	return true;

}

teleopC::teleopC(){

	force_bias_computed = false;
	force_feedback_on = false;
	force_bias_counter = 0;
	force_scale = 1;

}

void teleopC::calculateForceBias(geometry_msgs::Wrench wrench_msrd){

	//Estimating current bias of the force sensor by averaging 100 consecutive samples
	if(!force_bias_computed){
		if (this->force_bias_counter == 0){
			this->force_bias = KDL::Vector::Zero();
			this->force_bias_counter += 1;
		}
		else if (this->force_bias_counter < 100) {
			this->force_bias[0] += wrench_msrd.force.x / 100;
			this->force_bias[1] += wrench_msrd.force.y / 100;
			this->force_bias[2] += wrench_msrd.force.z / 100;
			this->force_bias_counter += 1;
		}
		else{
			this->force_bias_computed = true;
			this->force_bias_counter = 0;
			log(Info) << " Force sensor bias computed:" << " x=" <<this->force_bias[0] <<  " y="<< this->force_bias[1] << " z=" << this->force_bias[2] << endlog();
		}
	}

}

geometry_msgs::Wrench teleopC::getForceFeedback(KDL::Frame robot_pose, geometry_msgs::Wrench wrench_msrd){

	/////////////////////////////////////// Calculating force
	KDL::Vector force_msrd;
	KDL::Vector force_scaled;
	KDL::Vector force_out;

	if (force_feedback_on){
		// save the measurements in a kdl vector
		force_msrd = KDL::Vector(wrench_msrd.force.x, wrench_msrd.force.y, wrench_msrd.force.z);
		//			// remove the bias and scale the force
		//			if(force_filter_on) // have removed the filtering where force_filtered was calculated
		//				force_scaled = force_scale  * (this->force_filtered - this->force_bias);
		//			else ////No FILTERING
		force_scaled = force_scale  * (force_msrd - force_bias);

		//Taking the forces to the master's reference frame
		force_out = this->mstr_to_slv_frame.M.Inverse() * robot_pose.M * fs_to_ee_frame.M.Inverse() * force_scaled ;
		//
	}
	else//If the force is disabled
		force_out = KDL::Vector::Zero();

	return( conversions::createWrenchMsg(force_out[0], force_out[1], force_out[2], 0.0,0.0,0.0));


}

ORO_CREATE_COMPONENT(Teleop)
