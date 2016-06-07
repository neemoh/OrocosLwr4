#include "hapticACC-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <limits>
#include <math.h>
#include <iostream>
#include <fstream>

using namespace RTT;

HapticACC::HapticACC(std::string const& name) :
				TaskContext(name, PreOperational) {

	this->addPort("sigma_pose_read", this->pose_read_port).doc("Readings of the pose from Sigma");
	this->addPort("sigma_button", this->botton_port).doc("Destination");
	this->addPort("sigma_force", this->force_port).doc("Force");

	this->addOperation("setKB", &HapticACC::setKB, this, OwnThread).doc("set  coeffs").arg("coeffs", "setK");
	this->addOperation("setMethodAndShape", &HapticACC::setMS, this, OwnThread).doc("set  coeffs").arg("coeffs", "setK");
	this->addOperation("setFMAX", &HapticACC::setFMAX, this, OwnThread).doc("set  coeffs").arg("coeffs", "setK");

	this->addProperty("dynamic_tumor_motion_range", this->prop_motion_range_tumor).doc("The range of the periodic motion [m]");
	this->addProperty("dynamic_tumor_motion_freq", this->prop_sample_per_cycle_tumor).doc("Number of samples per cycle of periodic motion.");
	this->addProperty("max_force", this->prop_F_MAX).doc("Number of samples per cycle of periodic motion.");
	this->addProperty("dominant_hand", this->prop_dominant_hand).doc("Dominant hand of the user. 0 for Left. 1 for Right.");
	this->addProperty("udp_send_pre_sample", this->prop_udp_send_pre_sample).doc("Dominant hand of the user. 0 for Left. 1 for Right.");
	this->addProperty("udp_port", this->prop_local_port).doc("UDP socket");
	this->addProperty("remote_address", this->prop_remote_addr).doc("remote IP address");

	this->first_clutch[0]		= true;
	this->first_clutch[1]		= true;
	this->K 				= 200; //2000;
	this->B 				= 80;
	this->SCALE				= 3.0;
	this->time_init 		=
	this->sigma_state.dom_fx 	=
	this->sigma_state.dom_fy 	=
	this->sigma_state.dom_fz 	= 0.0;
	this->local_port =
	this->m_sock_addr_len 	=
	this->m_socket 			=
	this->ds 				=
	this->sent 				= 0;
	this->temp 				= 0.3;
	this->F_MAX				= 0.0;
	this->user_options.method	 = 0;
	this->user_options.shape 	 = 1;
	this->sample_per_cycle_tumor = 0;
	this->motion_range_tumor=
	this->dx_tumor			=
	this->dy_tumor			=
	this->dz_tumor			=
	this->C 				=
	this->THETA 			=
	this->V_TOOL_LAST 		= 0.0;
	this->v_tool_dir_last	= KDL::Vector(1.0,0.0,0.0);
	this->f_vc_dir_last		= KDL::Vector(1.0,0.0,0.0);
	this->n_2_last			= KDL::Vector(1.0,0.0,0.0);
	this->q					= KDL::Vector(0.0,0.0,0.0);
	this->p_tool[0]			= KDL::Vector(0.001,0.001,0.001);
	this->p_tool[1]			= KDL::Vector(0.001,0.001,0.001);

	this->counter_tumor 	= 0;
}

bool HapticACC::configureHook() {

	RTT::Logger::In in(this->getName());

	//manually raises LogLevel to 'Info' (5)
	if ( Logger::log().getLogLevel() < Logger::Info ) {
		Logger::log().setLogLevel( Logger::Info );
		Logger::log() << Logger::Info <<  "Log Level raised manually to Info."<<Logger::endl;
	}

	///// Reading the properties
	///////////////////////////

	// UDP Properties
	if(this->prop_local_port == 0){
		log(Warning) << "UDP socket not configured !" <<endlog();
		return false;
	}else{
		this->local_port = this->prop_local_port;
	}
	this->ip_address = "0.0.0.0";

	if(this->prop_remote_addr.empty()){
		log(Warning) << "Remote IP address not configured !" <<endlog();
		return false;
	}else
		this->ip_address = this->prop_remote_addr;
	if(this->prop_udp_send_pre_sample == 0){
		log(Warning) << "udp_send_pre_sample is not configured !" <<endlog();
		return false;
	}
	else
		this->udp_send_pre_sample = this->prop_udp_send_pre_sample;


	// Dominant haptic device ID
	this->dominant_hand = this->prop_dominant_hand;

	// Maximum force for all guidance methods
	if(this->prop_F_MAX == 0){
		log(Warning) << "The maximum force not configured! Setting 3N." <<endlog();
		this->F_MAX  = 3;
	}
	else
		this->F_MAX= this->prop_F_MAX;

	// Dynamic tumor properties
	if(this->prop_motion_range_tumor == 0)
		this->motion_range_tumor = 0.005;
	else
		this->motion_range_tumor = this->prop_motion_range_tumor;

	if(this->prop_sample_per_cycle_tumor == 0)
		this->sample_per_cycle_tumor  = 5000;
	else
		this->sample_per_cycle_tumor = this->prop_sample_per_cycle_tumor;




	/////// Initialize the haptic device
	if (!this->initSigma())
		return false;

	log(RTT::Info) << "HapticACC configured !" << Logger::endl;
	return true;
}

bool HapticACC::startHook() {
	Logger::In in(this->getName());

	// create the UDP socket
	if (HapticACC::udp_create_socket() != 0)
		return false;

	log(Info) << "HapticACC started !" <<endlog();
	return true;
}

void HapticACC::updateHook() {
	Logger::In in(this->getName());

	///////////   Saving the initial time for time analysis
	this->time_init = os::TimeService::Instance()->getTicks();


	/////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////             Measurement          ///////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////


	////////////    GETTING ORIENTATION IN MATRIX REPRESENTATION TO CONVERT IT TO QUATERNION
	////            AND GET A POSE MSG FOR ROS COMMUNICATION


	for (int dev=0; dev<2; dev++) {

		double orientationMatrix[3][3];
		//	 Read Position and Orientation from Sigma and apply transformation 90 deg rotation around z
		double temp_x, temp_y,temp_z;
		// Device 0 is left and 1 is right.


		dhdGetPositionAndOrientationFrame(&temp_x, &temp_y, &temp_z, orientationMatrix, dev);
		this->p_msrd[dev][0] =  temp_y;
		this->p_msrd[dev][1]  = -temp_x;
		this->p_msrd[dev][2]  =  temp_z;

		//Read Button state
		this->dev_Button[dev].data = dhdGetButton(0,dev);

		//////////////////////      CLUTCH
		if( this->dev_Button[dev].data == 1 ){
			if(this->first_clutch[dev]){
				this->p_msrd_init[dev] = this->p_msrd[dev];
				this->p_init[dev]	  = this->p_tool[dev];
				this->first_clutch[dev] = false;
			}
			this->p_tool[dev] = this->p_msrd[dev] - this->p_msrd_init[dev] + this->p_init[dev];
			//Clutching orientation
			//		ori_msrd_init_x = this->sigma_state.ox;
			//		ori_msrd_init_y = this->sigma_state.oy;
			//		ori_msrd_init_z = this->sigma_state.oz;
			//		ori_msrd_init_w = this->sigma_state.ow;

		}
		else {
			this->first_clutch[dev] = true;
			//		this->sigma_state.ox = this->ori_msrd_init_x;
			//		this->sigma_state.oy = this->ori_msrd_init_y;
			//		this->sigma_state.oz = this->ori_msrd_init_z;
			//		this->sigma_state.ow = this->ori_msrd_init_w;

		}

		// Creating a KDL frame to be converted to Pose message for ROS
		KDL::Frame dev_pose_frame;
		dev_pose_frame.p[0] = this->p_tool[dev].x();
		dev_pose_frame.p[1] = this->p_tool[dev].y();
		dev_pose_frame.p[2] = this->p_tool[dev].z();
		for (int i = 0; i < 3; i++) {
			for(int j = 0; j < 3; j++) {
				dev_pose_frame.M(i,j) = orientationMatrix[i][j];
			}
		}
		tf::PoseKDLToMsg(dev_pose_frame, this->dev_pose[dev]);

		if (dev == this->dominant_hand){
			// Saving the pose in the state variable to be sent over the UDP to the visualization computer
			this->sigma_state.dom_px = this->p_tool[dev][0]/this->SCALE;
			this->sigma_state.dom_py = this->p_tool[dev][1]/this->SCALE;
			this->sigma_state.dom_pz = this->p_tool[dev][2]/this->SCALE;
			this->sigma_state.dom_ox = this->dev_pose[dev].orientation.x;
			this->sigma_state.dom_oy = this->dev_pose[dev].orientation.y;
			this->sigma_state.dom_oz = this->dev_pose[dev].orientation.z;
			this->sigma_state.dom_ow = this->dev_pose[dev].orientation.w;

		}
		else{
			this->sigma_state.secondary_px = this->p_tool[dev][0]/this->SCALE;
			this->sigma_state.secondary_py = this->p_tool[dev][1]/this->SCALE;
			this->sigma_state.secondary_pz = this->p_tool[dev][2]/this->SCALE;
			this->sigma_state.secondary_ox = this->dev_pose[dev].orientation.x;
			this->sigma_state.secondary_oy = this->dev_pose[dev].orientation.y;
			this->sigma_state.secondary_oz = this->dev_pose[dev].orientation.z;
			this->sigma_state.secondary_ow = this->dev_pose[dev].orientation.w;
			this->sigma_state.secondary_buton = this->dev_Button[dev].data;

		}



	}





	/////////////////////
//Read velocity
	double  vx, vy, vz;
	dhdGetLinearVelocity(&vx, &vy, &vz,this->dominant_hand);
	// applying the transformation to velocity
	this->v_msrd = KDL::Vector(vy, -vx, vz);






	/////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////        CONSTRAINT GEOMETRY GENERATION      /////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////

	// define a shorter dominant_hand variable to make the code more readable!
	int dh = this->dominant_hand;

	if ( this->user_options.shape == 0 ) {

		// y-z plane
		this->cp = this->p_tool[dh];
		this->cp[0] = 0.0;


	} else if ( this->user_options.shape == 1 ) {
		// Circle
		double cx = 0.0;
		double cy = 0.0;
		double radius = 0.2;

		cp2circle(this->p_tool[dh][0],this->p_tool[dh][2],cx,cy,radius,this->cp[0],this->cp[2]);
		this->cp[1] = 0.0;

	} else if ( this->user_options.shape == 2 ) {

		this->cp[1] = 0.0;
		cp2square( this->p_tool[dh][0], this->p_tool[dh][2], -0.2, -0.2, 0.4, 0.4, this->cp[0], this->cp[2]);
		//log(Info) << this->cp[0] << " #:" << this->cp[2] << endlog();

	} else if ( this->user_options.shape == 3 ) {

		// A closed 2D curve representing the contour of a tumor that will be generated in 3D in
		// the visualization computer
		// The closestPoint algorithm finds the closest point on the curve in a simple recursive manner
		int NSEARCH     = 3;
		int NGUESS      = 6;
		double SEARCH_STEP = 0.02;
		tumor(this->p_tool[dh], this->cp, SEARCH_STEP, NSEARCH, NGUESS);


	} else if ( this->user_options.shape == 4 ) {


		// Dynamic case of the same closed curve.
//		double motion_range_tumor = 0.01;
//		int samples_per_cycle = 4000;

		int NSEARCH     = 3;
		int NGUESS      = 6;
		double SEARCH_STEP = 0.02;

		if (this->counter_tumor < this->sample_per_cycle_tumor){
				this->dx_tumor = motion_range_tumor * sin(2*M_PI* this->counter_tumor/this->sample_per_cycle_tumor);
				this->dz_tumor = -(motion_range_tumor * sin(2*M_PI* this->counter_tumor/this->sample_per_cycle_tumor));

				this->counter_tumor += 1;
			}
			else
				this->counter_tumor =0;

		KDL::Vector dynamic_p = KDL::Vector(this->p_tool[dh][0] - this->dx_tumor, this->p_tool[dh][1] - this->dy_tumor, this->p_tool[dh][2] - this->dz_tumor);

		tumor(dynamic_p, this->cp, SEARCH_STEP, NSEARCH, NGUESS);

		this->cp[0] += this->dx_tumor;
		this->cp[1] += this->dy_tumor;
		this->cp[2] += this->dz_tumor;



	} else {
		this->cp = KDL::Vector(0.0,0.0,0.0);
	}

	// Scaling and saving the force to be sent over the UDP
	this->sigma_state.dx_tumor 	= this->dx_tumor/this->SCALE;
	this->sigma_state.dy_tumor 	= this->dy_tumor/this->SCALE;
	this->sigma_state.dz_tumor 	= this->dz_tumor/this->SCALE;

	this->sigma_state.dom_proxx 	= this->cp[0] / this->SCALE;
	this->sigma_state.dom_proxy 	= this->cp[1]  / this->SCALE;
	this->sigma_state.dom_proxz 	= this->cp[2]  / this->SCALE;
	this->sigma_state.VCmethod 	= this->user_options.method;


	/////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////             FORCE GENERATION          ///////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////

	if ( this->user_options.method == 0 ) {

		this->f_vc = KDL::Vector(0.0,0.0,0.0);
//		double wall = -0.01;
//		double f_els, f_vis;
//
//		if (this->dev_Button.data == 1 && this->p_tool[dh][0] < wall){
//			f_els = -this->K * (this->p_tool[dh][0] - wall);
//			f_vis = -this->B * vx;
//		}
//		else{
//			f_els = 0.0;
//			f_vis = 0.0;
//		}
//
//		this->f_vc[0] = saturate(-F_MAX, f_els + f_vis , F_MAX );

	} else if ( this->user_options.method == 2 ) { // Bowyer 2014

    	double sig2 = 2.5;
    	double fc = this->F_MAX;
    	double theta = 0.4;
    	double sig0 = fc/0.003;
    	double zcss = fc/sig0;
    	KDL::Vector d = this->p_tool[dh] - this->p_last;
    	KDL::Vector pt = this->p_tool[dh] - this->cp;

    	// boundary condition
    	double ptrans = 0.002;
    	if ( norm(pt) < ptrans ) theta = theta * (norm(pt)/ptrans);

    	this->z = this->z + d;

    	KDL::Vector ptn = pt;
    	this->normalizeSafe(ptn, KDL::Vector(0.0,0.0,0.0));
    	KDL::Vector zn = this->z;
		this->normalizeSafe(zn, KDL::Vector(0.0,0.0,0.0));
		double a = atan2(norm(ptn*zn), dot(zn, ptn));

		KDL::Vector n = pt*this->z;
		KDL::Vector nn = n;
		this->normalizeSafe(nn, KDL::Vector(0.0,0.0,0.0));

		KDL::Vector y = cos(theta)*pt + sin(theta)*(nn*pt) + (1-cos(theta))*dot(nn,pt)*nn;
		KDL::Vector yn = y;
		this->normalizeSafe(yn, KDL::Vector(0.0,0.0,0.0));

		if ( a <= theta && norm(this->z) <= zcss )
			this->z = this->z * 1;
		else if ( a <= theta && norm(this->z) > zcss )
			this->z = zcss*zn;
		else
		{
			if ( dot(this->z,yn) <= 0.0 ) this->z = KDL::Vector(0.0,0.0,0.0);
			else if ( (0.0 < dot(this->z,yn)) && (dot(this->z,yn) < zcss) ) this->z = dot(this->z,yn)*yn;
			else this->z = zcss * yn;
		}

		this->f_vc = - (sig0*this->z + sig2*this->v_msrd);

	} else if ( this->user_options.method == 1 ) { //Kikuuwe

    	double F = 0.5;
    	double R = this->F_MAX;
    	int Kk = R/0.003;
    	double Bk = 1.5;
    	double T = 0.001;

    	KDL::Vector q_last = this->q;
    	double cte = Kk + (Bk/T);
    	KDL::Vector ps = this->p_tool[dh] + (Bk * (this->q - this->p_last) / (Kk * T + Bk));
    	KDL::Vector ki = ps - this->cp;
    	KDL::Vector n = ki;
		this->normalizeSafe(n, KDL::Vector(0.0,0.0,0.0));

		double a1 = saturate(0.0, dot(n,ki), (R/cte));
		double a2 = saturate(0.0, -dot(n,ki), (R/cte));

		KDL::Vector e = q_last - ps;
		double a3 = saturate(-a1, dot(n,e), a2);

		//I-n*n'
		KDL::Vector In1 = KDL::Vector(1 - n[0]*n[0], 0 - n[0]*n[1], 0 - n[0]*n[2]);
		KDL::Vector In2 = KDL::Vector(0 - n[1]*n[0], 1 - n[1]*n[1], 0 - n[1]*n[2]);
		KDL::Vector In3 = KDL::Vector(0 - n[2]*n[0], 0 - n[2]*n[1], 1 - n[2]*n[2]);
		//(I-n*n')*e
		KDL::Vector mat = KDL::Vector(dot(In1,e), dot(In2,e), dot(In3,e));
		double m = max(1.0,( ( cte/F ) * sqrt( ( norm(e)*norm(e) ) - ( dot(n,e)*dot(n,e) ) ) ) );

		this->q = ps + n * a3 + mat/m;

		KDL::Vector a4 = sat(e, (F/cte));
		KDL::Vector qf = ps + a4;
		double qfql = norm(qf - q_last);
		double qql = norm(this->q - q_last);

		if ( qfql <= qql ) this->q = qf;

		this->f_vc = Kk * (this->q - this->p_tool[dh]) + Bk * ( ( this->q - q_last ) - ( this->p_tool[dh] - this->p_last ) ) / T;

	} else if ( this->user_options.method == 3 ) {

		// UPPERCASE NAMES = SCALARS
		// LOWERCASE NAMES = VECTORS

		double F_VC, F_VC_SAT, V_TOOL;
//		double PHI 		 	= M_PI*2/3;
//		double V_THRESH  	= 0.1;			 // [m/s]
		double PENET_THRESH	= 0.004;		 // [m]
		double B_M			= this->B;
//		double V_THRESH2 	= 0.05;			 // [m/s]
//		double W_THRESH	 	= 2*M_PI;		 // [rad/s]
//		double W		 	= 0.0;			 // [rad/s]

		KDL::Vector penet 			= this->cp - this->p_tool[dh];
		KDL::Vector penet_dir 		= penet;
		KDL::Vector v_tool_dir 		= this->v_msrd;

//		penet[1] 	= 0.0;
		this->normalizeSafe(v_tool_dir, this->v_tool_dir_last);
		this->normalizeSafe(penet_dir, KDL::Vector(0.0,0.0,0.0));

		double V_PENET_DOTP = dot(v_tool_dir, penet_dir);
		double PENET =norm(penet);

		if (PENET< PENET_THRESH){
//			PENET = 0;
			B_M =  this->B * PENET/ PENET_THRESH;
		}
		else
			B_M = this->B;

		V_TOOL = norm( this->v_msrd );
		F_VC = B_M * sqrt( ( 1 - V_PENET_DOTP ) / 2 ) * V_TOOL;

		KDL::Vector n = v_tool_dir * penet_dir;
		KDL::Vector nn = n;
		this->normalizeSafe(nn, penet_dir);

		//		this->C = this->C + (V_TOOL - this->V_TOOL_LAST)/V_THRESH;
		//		this->C = saturate(0.0,this->C,1.0);

		//		if (PENET <= PENET_MIN)
		//			this->C = PENET/PENET_MIN * this->C;

		//				this->THETA = (M_PI/2) + this->C*PHI;
		//		this->THETA = (M_PI/2);
		this->THETA = (M_PI/2) * (1 + V_PENET_DOTP);

		if ( V_PENET_DOTP < 0.0 )
			this->f_vc_dir_des = penet_dir;
		else
			//if(this->rotateVector(v_tool_dir, this->f_vc_dir_des, nn, THETA) < 0)
			if(this->rotateVector(v_tool_dir, this->f_vc_dir_des, nn, this->THETA) < 0)
				log(RTT::Error) << "Null in rotateVector." <<"  norm(v_tool_dir) = "<< norm(v_tool_dir)<<
				"  , norm(nn) = "<< norm(nn)<< endlog();

//		W = saturate(0.0, V_TOOL /V_THRESH2 *W_THRESH, W_THRESH);
		KDL::Vector n_2 = this->f_vc_dir_last * this->f_vc_dir_des;

		this->normalizeSafe(n_2, this->n_2_last);
		//
//		if(dot(this->f_vc_dir_last, this->f_vc_dir_des) < 0.99) {
//			if(this->rotateVector(this->f_vc_dir_last, this->f_vc_dir, n_2, W*dt)<0)
//				log(RTT::Error) << "Null in rotateVector." <<"  norm(this->f_vc_dir_last) = "<< norm(this->f_vc_dir_last)<<
//				"  , norm(n_2) = "<< norm(n_2)<< endlog();
//		}
//		else{
		this->f_vc_dir = this->f_vc_dir_des;
//
//		}


//		if ( V_PENET_DOTP < 0.0 )
//			this->f_vc_dir = penet_dir;
//		else
//			this->rotateVector(v_tool_dir, f_vc_dir,nn,THETA);
//		f_vc_dir = f_vc_dir_des;

		F_VC_SAT = saturate(-this->F_MAX, F_VC , this->F_MAX );
		this->f_vc = F_VC_SAT * this->f_vc_dir;


//		if (dot(this->f_vc, this->v_msrd) > 0.1)
//		if(PENET == 0.0)
//			this->f_vc = KDL::Vector(0.0,0.0,0.0);

		this->V_TOOL_LAST= V_TOOL;
		this->v_tool_dir_last = v_tool_dir;
		this->f_vc_dir_last = this->f_vc_dir;
		this->n_2_last = n_2;

	} else if ( this->user_options.method == 99 ) {

		this->f_vc = -this->K * (this->p_tool[dh] - this->cp);

	} else {

		this->f_vc = KDL::Vector(0.0,0.0,0.0);

	}

	this->p_last = this->p_tool[dh];


	// Adding interaction force in form of an elastic constraint on xz plane (shifted for 1mm inside the tissue)
	double f_interaction = 0.0;
	if (this->p_tool[dh][1] > 0 ){
		f_interaction = -400 * (this->p_tool[dh][1] - 0.001);
	}

	// Transforming and saving force
	this->sigma_state.dom_fx = -(this->f_vc[1]+f_interaction);
	this->sigma_state.dom_fy =  this->f_vc[0];
	this->sigma_state.dom_fz =  this->f_vc[2];


// Subordinate hand force generation is only a rigid wall on y = 0;
	double f_sub = 0.0;
	if (this->p_tool[1-dh][1] > 0 ){
		f_sub = -400 * p_tool[1-dh][1];
		if (f_sub > 5.0)
			f_sub = 5.0;
	}

	this->sigma_state.secondary_fx = -f_sub;
	this->sigma_state.secondary_fy = 0.0;
	this->sigma_state.secondary_fz = 0.0;

	/////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////             DATA TRANSMISSION         ///////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////  TO THE SIGMA apply force

	if( this->dev_Button[dh].data == 1 ){
		if (dhdSetForceAndTorqueAndGripperForce(this->sigma_state.dom_fx , this->sigma_state.dom_fy , this->sigma_state.dom_fz , 0.0, 0.0, 0.0, 0.0, dh) < DHD_NO_ERROR)
			log(RTT::Error) << "error: cannot set force \n" <<  dhdErrorGetLastStr() << endlog();
	}
	else{
		//Apply zero force
		if (dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, dh) < DHD_NO_ERROR)
			log(RTT::Error) << "error: cannot set force \n" <<  dhdErrorGetLastStr() << endlog();
	}

	if( this->dev_Button[1-dh].data == 1 ){
		if (dhdSetForceAndTorqueAndGripperForce(this->sigma_state.secondary_fx , this->sigma_state.secondary_fy , this->sigma_state.secondary_fz , 0.0, 0.0, 0.0, 0.0, 1-dh) < DHD_NO_ERROR)
			log(RTT::Error) << "error: cannot set force \n" <<  dhdErrorGetLastStr() << endlog();
	}
	else{
		//Apply zero force
		if (dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1-dh) < DHD_NO_ERROR)
			log(RTT::Error) << "error: cannot set force \n" <<  dhdErrorGetLastStr() << endlog();
	}
	//////////////////////////////////////////////////////  TO UDP
	this->ds += 1;
	if( this->ds == this->udp_send_pre_sample) {
		//this->sigma_state.fx = os::TimeService::Instance()->getTicks();
		if(this->udp_send() != 0)
			log(Warning) << "error sending on the UDP !" <<endlog();
		else this->sent+=1;
		this->ds = 0;
	}

//	if( (this->udp_recv())  != 0)
//		log(Warning) << "error receiving data on the UDP !" <<endlog();

//	log(Warning) << "Received: " << this->user_options.method << " " << this->user_options.shape  <<endlog();


	/////////////////////////////////////////////////////  TO PORTS
	// measureing elapsed time sine time_init
	double dt = os::TimeService::Instance()->secondsSince(this->time_init);

	this->mstr_wrench_cmd.force.x = this->f_vc[0];
	this->mstr_wrench_cmd.force.y = this->f_vc[1];
	this->mstr_wrench_cmd.force.z = this->f_vc[2];
	//using torque messages to view other data during the development
	this->mstr_wrench_cmd.torque.x = this->v_msrd[0];
	this->mstr_wrench_cmd.torque.y = this->v_msrd[1];
	this->mstr_wrench_cmd.torque.z = this->v_msrd[2];

	this->dev_pose_stamped.pose.position.x = this->sigma_state.dom_px;
	this->dev_pose_stamped.pose.position.y = this->sigma_state.dom_py;
	this->dev_pose_stamped.pose.position.z = this->sigma_state.dom_pz;
	//using orientation messages to send proxy position
	this->dev_pose_stamped.pose.orientation.x = this->sigma_state.dom_proxx;
	this->dev_pose_stamped.pose.orientation.y = this->sigma_state.dom_proxy;
	this->dev_pose_stamped.pose.orientation.z = this->sigma_state.dom_proxz;
	this->dev_pose_stamped.pose.orientation.w = 0.0;

	this->dev_pose_stamped.header.stamp.nsec = os::TimeService::ticks2nsecs(time_init);
//	this->dev_pose_stamped.header.frame_id = '0';
	this->pose_read_port.write(this->dev_pose_stamped);

	this->force_port.write(this->mstr_wrench_cmd);

	if(this->dev_Button[dh].data != this->dev_Button_Last.data){
		this->botton_port.write(dev_Button[dh]);
	}
	this->dev_Button_Last.data = this->dev_Button[dh].data;

}

void HapticACC::stopHook() {
	Logger::In in(this->getName());

	dhdStop();
	drdClose();
	if (dhdClose () < 0) {
		log(RTT::Error) << "Cannot close dhd. " <<  dhdErrorGetLastStr() << endlog();
	}
	else
		log(RTT::Info) << "dhd Closed." <<  dhdErrorGetLastStr() << endlog();

}

void HapticACC::cleanupHook() {
	close(m_socket);
}

bool HapticACC::initSigma() {

	// *************************************************************************************************
			// *********************************** Initialization: Sigma  **************************************
			// *************************************************************************************************

			int major, minor, release, revision;
			dhdGetSDKVersion (&major, &minor, &release, &revision);
		//	printf ("Force Dimension - SDK version: %d.%d.%d.%d\n", major, minor, release, revision);
			log(RTT::Info) << "Force Dimension - SDK version: " << major <<"."<< minor<<"."<< release<<"."<< revision<<Logger::endl;




			for (int dev=0; dev<2; dev++) {


					// open device
					if (drdOpenID (dev) < 0) {
						log(RTT::Error) << "Error: not enough devices found. " <<  dhdErrorGetLastStr() << endlog();

						dhdSleep (2.0);
						for (int j=0; j<=dev; j++) drdClose (j);
						return -1;
					}

					//TODO: Calibration procedure is not working. For the moment it is supposed that the
					// calibration is performed externally
					//Calibrate the device if it is not already calibrated;
					if(drdIsInitialized(dev)){
						ROS_INFO("Device is already calibrated.\n");
					}
					else if(drdAutoInit(dev)<0) {
						ROS_ERROR("error: initialization failed (%s)\n", dhdErrorGetLastStr ());
						dhdSleep(2.0);
					}

					log(RTT::Info) <<  dhdGetSystemName(dev) << " device detected." << Logger::endl;

					//std::cout << "drdIsInitialized()" << drdIsInitialized() <<std::endl;
					//std::cout << "drdAutoInit()" << drdAutoInit() <<std::endl;
					//std::cout << "drdCheckInit ()" << drdCheckInit () <<std::endl;
					//ROS_ERROR("error: initialization failed (%s)\n", dhdErrorGetLastStr ());


					// stop regulation (and leave force enabled)
					drdStop(true,dev);
					// enable force
					dhdEnableForce (DHD_ON, dev);
					//Enable the gripper button
					dhdEmulateButton(DHD_ON, dev);
				}

			return true;


}
/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(HapticACC)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
void HapticACC::setFMAX(const double F){

	this->F_MAX = F;
}

void HapticACC::setKB(double K, double B){

	this->K = K;
	this->B = B;
}

void HapticACC::setMS(int M, int S){

	this->user_options.method = M;
	this->user_options.shape = S;
}

int HapticACC::udp_create_socket() {
	if (this->m_socket != 0)
		close(this->m_socket);
	this->m_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);


	int flags = fcntl(this->m_socket, F_GETFL);
	flags |= O_NONBLOCK;
	fcntl(this->m_socket, F_SETFL, flags);

//	setsockopt(this->m_socket, SOL_SOCKET, SO_REUSEADDR, 0, 0);

	this->m_sock_addr_len		    = sizeof(struct sockaddr_in);
	//	}
	this->m_remote_addr.sin_family	    = AF_INET;
//	this->m_remote_addr.sin_addr.s_addr = inet_addr(this->ip_address.c_str());
	this->m_remote_addr.sin_port        = htons(this->local_port);

	//TODO  I got an error when I used this->ip_address.c_str() so I put the ip directly. fix later
	if (inet_aton("192.168.30.40", &m_remote_addr.sin_addr)==0) {
		log(Error) << "inet_aton() failed\n" << stderr << endlog();
	}
//	if (inet_aton(this->ip_address.c_str(), &m_remote_addr.sin_addr)==0) {
//		log(Error) << "inet_aton() failed\n" << stderr << endlog();
//	}

	struct sockaddr_in local_addr;
	bzero((char *) &local_addr, sizeof(local_addr));
	local_addr.sin_family = AF_INET;
	local_addr.sin_addr.s_addr = INADDR_ANY;
	local_addr.sin_port = htons(this->local_port);

	log(Info) << "Binding port #: " << (int)(local_addr.sin_port) << " #:" << this->local_port << " on ip " << this->ip_address << endlog();

//	if (bind(this->m_socket, (sockaddr*) &local_addr, sizeof(sockaddr_in)) < 0) {
//		log(Error) << "Binding of port failed with errno " << errno << endlog();
//		return -1;
//	}

	return 0;
}

int HapticACC::udp_recv() {
	// receive data, this is a blocking call
	if (int n = recvfrom(this->m_socket, (void*) &this->user_options, sizeof(this->user_options), 0,
			(sockaddr*) &m_remote_addr, &m_sock_addr_len) == -1){
//		diep("recvfrom()");
		return -1;
	}
//	if (sizeof(this->temp) != n) {
//		log(Error) << "bad packet length: " << n << ", expected: "
//				<< sizeof(double) << endlog();
//		return -1;
//	}
	return 0;
}

//int HapticACC::udp_send() {
//	if (sendto(this->m_socket, (void*) &this->sigma_state.px, sizeof(this->sigma_state.px), 0,
//			(sockaddr*) &m_remote_addr, m_sock_addr_len) ==-1) {
//		log(Error) << "Sending datagram failed." << endlog();
//		diep("sendto()");
//		return -1;
//	}
//	return 0;
//}


int HapticACC::udp_send() {
	if (sendto(this->m_socket, (void*) &this->sigma_state, sizeof(this->sigma_state), 0,
			(sockaddr*) &m_remote_addr, m_sock_addr_len) ==-1) {
		log(Error) << "Sending datagram failed." << endlog();
		diep("sendto()");
		return -1;
	}
	return 0;
}

int HapticACC::rotateVector(const KDL::Vector vector_in, KDL::Vector &vector_out,  const KDL::Vector n, const double THETA){
	if (norm(vector_in) ==0.0 || norm(n) ==0.0){
		log(Error) << "Null vector given as the input of rotation function." << endlog();
		return -1;
	}
	else{
		vector_out = vector_in + sin(THETA)*(n*vector_in)+(1.0-cos(THETA))*(n*(n*vector_in));
		return 0;
	}
}
int HapticACC::normalizeSafe(KDL::Vector & vec_in, const KDL::Vector vec_safe) {
	// Checks the norm of the vec_in. If it is non zero the function normalizes it.
	// If the norm is zero the function uses vec_safe as the output

	double l = norm(vec_in);
	if( abs(l) >= std::numeric_limits<double>::epsilon() ) {
		vec_in[0] /= l;
		vec_in[1] /= l;
		vec_in[2] /= l;
		return 0;
    } else {
    	vec_in = vec_safe;
    	return -1;
    }
}
double saturate (double a, double x, double b){

	if (x < a){
		return a;
	}
	else if (x >b){
		return b;
	}
	else
		return x;

}

KDL::Vector HapticACC::sat(KDL::Vector x, double a){

	double l = norm(x);
	KDL::Vector xn = x;
	normalizeSafe(xn, KDL::Vector(0.0,0.0,0.0));

	if (l <= a) return x;
	else return a * xn;

}

double norm(KDL::Vector x) {
	return sqrt( x[0]*x[0] + x[1]*x[1] + x[2]*x[2] );
}

double dist( double p1x, double p1y, double p2x, double p2y ) {
	return sqrt( pow((p1x-p2x),2) + pow((p1y-p2y),2));
}

void min(std::vector<double>  table, int size, double &value, int &index)
{
   value = table[0]; // initialize
   index=0;

   for(int i=1;i<size;i++)
   {
      if(value > table[i])
      {
	  value=table[i];
	  index=i;
      }
   }
}

void diep(char *s) {
	perror(s);
}

void cp2circle( double x, double y, double xc, double yc, double r, double &cpx, double &cpy ) {

    double vX = x - xc;
    double vY = y - yc;
    double magV = sqrt(vX*vX + vY*vY);
    cpx =  xc + vX / magV * r;
    cpy = yc + vY / magV * r;

}

void cp2square( double px, double py, double x, double y, double width, double height, double &cpx, double &cpy) {
// x: px < x ? x : px > x + w ? x + w : px, y: py < y ? y : py > y + h ? y + h : py
// px, py coord of the point; x, y origin vertice of the square;

	if (px < x) cpx = x;
	else if (px > (x + width)) cpx = x + width;

	if (py < y) cpy = y;
	else if (py > (y + height)) cpy = y + height;

	//assuming center at 0,0
	if ( py > px ) {
		if ( py > -px ) {
			cpx = px;
			cpy = cpy = y + height;
		} else {
			cpy = py;
			cpx = x;
		}
	} else {
		if ( py > -px ) {
			cpy = py;
			cpx = x + width;
		} else {
			cpx = px;
			cpy = y;
		}
	}
};

void tumor(KDL::Vector p, KDL::Vector & cp, double SEARCH_STEP, int NSEARCH, int NGUESS){

	if (p[2] > 0.0) {
		if (p[0] < 0.0) {

			double X_LEFT_LIM = -0.3;
			double X_RIGHT_LIM = 0.0;
			cp = closestPoint(1, p, SEARCH_STEP, NSEARCH, NGUESS, X_LEFT_LIM, X_RIGHT_LIM);

		} else {

			double X_LEFT_LIM = 0.0;
			double X_RIGHT_LIM = 0.3;
			cp = closestPoint(2, p, SEARCH_STEP, NSEARCH, NGUESS, X_LEFT_LIM, X_RIGHT_LIM);

		}
	} else {
		if (p[0] > 0.0){

			double X_LEFT_LIM = 0.0;
			double X_RIGHT_LIM = 0.3;
			cp = closestPoint(3, p, SEARCH_STEP, NSEARCH, NGUESS, X_LEFT_LIM, X_RIGHT_LIM);

		}else {

			double X_LEFT_LIM = -0.3;
			double X_RIGHT_LIM = 0.0;
			cp = closestPoint(4, p, SEARCH_STEP, NSEARCH, NGUESS, X_LEFT_LIM, X_RIGHT_LIM);

		}
	}
	cp[1] = 0.0;

}

double lines( double x, int quad) {
	double scale = 2;
	if ( quad == 1 ) return (0.2983 + 0.9948*x + 6.6122*pow(x,2) + 22.2222*pow(x,3))/scale;
	else if ( quad==2 ) return (0.3000 + 3.8500*x - 56.1667*pow(x,2) + 473.3333*pow(x,3) - 1933.3333*pow(x,4) + 2666.6667*pow(x,5)) / scale;
	else if ( quad==3 ) return (-0.3018 - 3.7631*x +46.9167*pow(x,2) - 173.3333*pow(x,3) + 233.3333*pow(x,4)) / scale;
	else if ( quad==4 ) return (-0.2991 - 3.7188*x - 42.4490*pow(x,2) - 111.1111*pow(x,3)) / scale;
	else return 0.0;

}

KDL::Vector closestPoint( int quad, KDL::Vector point, double search_step, int nsearch, int nguess, double X_LEFT_LIM, double X_RIGHT_LIM ) {

	std::vector<double> candid(2, 0.0);
	candid[0] = point[0];
	candid[1] = point[2];

//	log(Info) << "point a.  candid[0]: " << candid[0] << " candid[1]: " << candid[1]<< endlog();
	for ( int j = 0; j < nsearch; j++ ) {

		divideAndConquer(quad, point, candid, search_step, nguess, X_LEFT_LIM, X_RIGHT_LIM);
		search_step = search_step/nguess*2;
	}
//	log(Info) << "point b.  candid[0]: " << candid[0] << " candid[1]: " << candid[1]<< endlog();
	return KDL::Vector(candid[0], 0.0, candid[1]);

}


void divideAndConquer(int quad, KDL::Vector point, std::vector<double> &init, double search_step, int nguess, double X_LEFT_LIM, double X_RIGHT_LIM) {

//	std::vector<std::vector<double>>  guesses[nguess+1][2] = std::vector<double>(2, 0.0);
	std::vector<double>  guesses_x (nguess+1,0.0);
	std::vector<double>  guesses_y (nguess+1,0.0);
	std::vector<double>  distlist(nguess+1,0.0);
	std::vector<double> point_temp(2, 0.0);
//	point_temp[0] = point[0];
//	point_temp[1] = point[2];

	double init_guess = init[0] -nguess/2*search_step;

	if (init_guess < X_LEFT_LIM) init_guess = X_LEFT_LIM;
	else if ((init_guess + search_step*nguess) > X_RIGHT_LIM) init_guess = X_RIGHT_LIM - search_step*nguess;

	for (int j=0; j<nguess+1; j++) {
		guesses_x[j] = init_guess + (j)*search_step;
		guesses_y[j] = lines(guesses_x[j], quad);
		distlist[j] = dist(point[0], point[2], guesses_x[j], guesses_y[j]);
	}



	double temp;
	int jj;

	min(distlist, nguess+1, temp, jj);

	init[0] = guesses_x[jj];
	init[1] = guesses_y[jj];//guesses[jj];
}

ORO_CREATE_COMPONENT(HapticACC)
