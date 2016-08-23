#include "taskPerformanceEval-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

using namespace std;

taskPerformanceEval::taskPerformanceEval(std::string const& name) : TaskContext(name){

	this->addEventPort("inputCurrPoseInSlaveFrame", this->port_inp_curr_pose_in_slv).doc(" ");
	this->addPort("inputDesPoseInSlaveFrame", 		this->port_inp_des_pose_in_slv).doc(" ");
	this->addPort("inputTwistInSlaveFrame", 		this->port_inp_twist_in_slv).doc(" ");
	this->addPort("inputMasterPose", 				this->port_inp_master_pose);
	this->addPort("inputTaskToSlaveTr", 			this->port_inp_task_to_slv_tr);
	this->addPort("inputClutch", 					this->port_inp_clutch);
	this->addPort("inputACForce", 					this->port_inp_wrench);
	this->addPort("inputAcPathLength",				this->port_inp_ac_length);

	this->addPort("outputCurrPoseInSlvRfDownSmpl", 	this->port_out_curr_pose_in_slvrf_downsmpl);
	this->addPort("outputDestPoseInSlvRfDownSmpl", 	this->port_out_des_pose_in_slvrf_downsmpl);
	this->addPort("outputCuttingPoseInTaskRfDownSmpl", this->port_out_cutting_pose_in_taskrf_downsmpl);
//	this->addPort("outputTwist", this->port_out_twist_in_slv_downsmpl);
	this->addPort("outputClutchStamped", this->port_out_clutch_stamped);
	this->addPort("outputMasterDownSmplStamped", this->port_out_mstr_pose_downsmpl_stamped);
	this->addPort("outputMetrics1", this->port_out_metrics1_downsmpl);
	this->addPort("outputMetrics2", this->port_out_metrics2_downsmpl);
	this->addPort("outputMetricsAll", this->port_out_metrics_all);

	this->addPort("outputEvents", this->port_out_events).doc("Sending events: s: acquisition started, e: acquisition ended");


	// initialize
	this->vision_downsmpl_counter		=
	this->recording_downsmpl_counter 	= 0;

	this->addProperty("visionDownsmplRatio", 	this->vision_downsmpl_ratio_param);
	this->addProperty("recordingDownsmplRatio", 	this->recording_downsmpl_ratio_param);


	this->addOperation("startAcquisiion", 	&taskPerformanceEval::startAcquisition, 	this).doc("Start evaluating and writing on ports");
	this->addOperation("endAcquisiion", 	&taskPerformanceEval::endAcquisition, 	this).doc("Stop evaluating and writing on ports");



	this->task_running = false;

	//Initializing variables
	this->ws_total_samples 		= 0;
	this->ws_boundary_samples 	= 0;
	this->num_clutchings 		= 0;
	this->mstr_tot_displacement = 0.0;
	this->elapsed_time 			= 0.0;
	this->ticks_from_acq_start 	= 0;

	this->error_sqrd_sum    	= 0.0;
	this->rmse_counter 			= 0;
	this->max_error				= 0.0;
	this->max_vel				= 0.0;
	this->cut_segments			= 0;
	this->lost_contact_with_tissue = true;
	this->ac_length				= 0.0;

	this->ws_alert 				=
	this->ws_alert_last 		= false;
	this->clutch_engaged		= false;
	this->new_clutch_msg		= false;

	sigma_workspace_tr.M = KDL::Rotation::RotZ(M_PI/4);
	sigma_workspace_tr.p = KDL::Vector(0.038, 0.0, -0.006);

	std::cout << "taskPerformanceEval constructed !" <<std::endl;
}

bool taskPerformanceEval::configureHook(){
	RTT::Logger::In in(this->getName());

	// initializing the output ports
	this->port_out_curr_pose_in_slvrf_downsmpl.setDataSample(this->pose_msg);
	this->port_out_des_pose_in_slvrf_downsmpl.setDataSample(this->pose_msg);
	this->port_out_cutting_pose_in_taskrf_downsmpl.setDataSample(this->pose_stamped_msg);
	this->port_out_mstr_pose_downsmpl_stamped.setDataSample(this->pose_stamped_msg);
	this->port_out_clutch_stamped.setDataSample(this->point_stamped_msg);
	this->port_out_metrics1_downsmpl.setDataSample(this->vec3_stamped_msg);
	this->port_out_metrics2_downsmpl.setDataSample(this->vec3_stamped_msg);
	this->port_out_metrics_all.setDataSample(this->pose_stamped_msg);
	this->port_out_events.setDataSample(this->char_msg);


	std::cout << "taskPerformanceEval configured !" <<std::endl;
	return true;

}



//---------------------------------------------------------------------------------------------
// Start hook
//---------------------------------------------------------------------------------------------
bool taskPerformanceEval::startHook(){
	RTT::Logger::In in(this->getName());

	std::cout << "taskPerformanceEval started !" <<std::endl;
	return true;
}


//---------------------------------------------------------------------------------------------
// Update hook
//---------------------------------------------------------------------------------------------
void taskPerformanceEval::updateHook(){
	RTT::Logger::In in(this->getName());

	// read the current and desired poses in slave frame and transform them to task frame
	this->readCurrAndDesPosesAndTransformToTask(this->curr_pose_in_taskrf, this->des_pose_in_taskrf);

	// down-sample the current and desired tool poses in slave frame to be published for the
	// vision node
	this->downsampleAndSendCurrAndDesPoses(this->curr_pose_in_taskrf, this->des_pose_in_taskrf);

	// read path length from port
	if (this->port_inp_ac_length.connected() && this->port_inp_ac_length.read(this->float_msg) == RTT::NewData )
		this->ac_length = this->float_msg.data;

	// read master pose
	if(this->port_inp_master_pose.read(this->pose_msg)!=RTT::NoData){
		this->mstr_position_last = this->mstr_position;
		this->mstr_position = KDL::Vector(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);
	}

	// read the clutch port. If new data available stamp it and send it.
	if(this->port_inp_clutch.read(this->int8_msg) == RTT::NewData){
		this->new_clutch_msg = true;
		this->clutch_engaged = this->int8_msg.data;
	}

	// check if the user is close to sigma's ws limit
	this->ws_alert = this->isCloseToSigmaWorkSpaceBoundary(this->mstr_position);
	// if workspace alert changes send a message ------------------------
	if(this->ws_alert != this->ws_alert_last ){

		if(this->ws_alert && this->clutch_engaged) // only when clutch is engaged alert the user
			this->char_msg.data = 'w';
		else
			this->char_msg.data = 'q';

		this->port_out_events.write(this->char_msg);
	}
	this->ws_alert_last = this->ws_alert;  // ------------------------------------------



	if(this->task_running){

		//record time time
		this->elapsed_time = RTT::os::TimeService::Instance()->secondsSince(this->ticks_from_acq_start);
		this->recording_downsmpl_counter++;

		if(this->recording_downsmpl_counter == this->recording_downsmpl_ratio_param){
			this->recording_downsmpl_counter = 0;

			// penetration check. I care about the error only when
			// the tool is in contact with (or very close to) the board. Naturally a more
			// sophisticated check will be needed if the task is not on a board
			if((this->des_pose_in_taskrf.p[2] - this->curr_pose_in_taskrf.p[2]) > -0.005){

				//---------------------------------------------------------------------------------------------
				//			METRICS 1
				//---------------------------------------------------------------------------------------------
				// calculate the error as the distance from current position to the desired one
				double error = (this->des_pose_in_taskrf.p - this->curr_pose_in_taskrf.p).Norm();

				// compute the error sum and add to the sample counter to  find the rmse at the end of the acq.
				this->error_sqrd_sum   += error*error;
				this->rmse_counter += 1;

				// maximum error
				if(error > this->max_error) this->max_error = error;

				//---------------------------------------------------------------------------------------------
				// read velocity
				if(this->port_inp_twist_in_slv.read(this->twist_msg) == RTT::NewData)
					tf::TwistMsgToKDL(this->twist_msg, this->twist_in_slave);

				// find its norm
				double vel = this->twist_in_slave.vel.Norm();

				// maximum velocity
				if(vel > this->max_vel) this->max_vel = vel;

				// count cut segments (how many times the user lost and regained contact with the surface)
				if(this->lost_contact_with_tissue){
					this->cut_segments += 1;
					this->lost_contact_with_tissue = false;
				}

				//---------------------------------------------------------------------------------------------
				// read force
				if(this->port_inp_wrench.read(this->wrench_msg) == RTT::NewData)
					kdl_vec = KDL::Vector(this->wrench_msg.force.x, this->wrench_msg.force.y, this->wrench_msg.force.z);

				double force = kdl_vec.Norm();

				//---------------------------------------------------------------------------------------------
				// saving in a message and write on the port
				this->writeTimeStamp(this->elapsed_time, this->vec3_stamped_msg);
				this->vec3_stamped_msg.vector.x = error;
				this->vec3_stamped_msg.vector.y = vel;
				this->vec3_stamped_msg.vector.z = force;

				this->port_out_metrics1_downsmpl.write(this->vec3_stamped_msg);



				//---------------------------------------------------------------------------------------------
				// Writing the cutting pose on the port
				tf::PoseKDLToMsg(this->curr_pose_in_taskrf,this->pose_stamped_msg.pose);
				this->writeTimeStamp(this->elapsed_time, this->pose_stamped_msg);
				this->port_out_cutting_pose_in_taskrf_downsmpl.write(this->pose_stamped_msg);

			}// end if penetrating
			else
				this->lost_contact_with_tissue = true;


			//---------------------------------------------------------------------------------------------
			// 				MASTER METRICS
			//---------------------------------------------------------------------------------------------

			//---------------------------------------------------------------------------------------------
			// Workspace check.
			// here we want to check how often the user gets close to the boundaries of the Sigma's workspace
			this->ws_total_samples++;
			if(this->ws_alert)
				this->ws_boundary_samples++;


			//---------------------------------------------------------------------------------------------
			// Calculate total master displacement
			// Simply find the norm of the Cartesian displacement from the last sample and
			this->mstr_tot_displacement += (this->mstr_position - this->mstr_position_last).Norm();



			//---------------------------------------------------------------------------------------------
			// sending the down-sampled and stamped master position
			if(this->port_inp_master_pose.read(this->pose_msg)!= RTT::NoData){
				this->pose_stamped_msg.pose = this->pose_msg;
				this->writeTimeStamp(this->elapsed_time, this->pose_stamped_msg);
				this->port_out_mstr_pose_downsmpl_stamped.write(this->pose_stamped_msg);
			}

		}// --------------------------- end if for down-sampling


		if(this->new_clutch_msg){

			this->new_clutch_msg = false;
			//Clutch counter
			if(this->clutch_engaged==1)
				this->num_clutchings++;

			// couldn't find any good stamped message type that's why I'm using pointStamped!!
			this->point_stamped_msg.point.x = this->clutch_engaged;
			//			this->point_stamped_msg.header.stamp.Time(this->elapsed_time);
			this->port_out_clutch_stamped.write(this->point_stamped_msg);
		}


	} // ------------------------end if task running




	//---------------------------------------------------------------------------------------------
	// port_in_pose_1

}



void taskPerformanceEval::stopHook() {
  std::cout << "taskPerformanceEval executes stopping !" <<std::endl;
}



void taskPerformanceEval::cleanupHook() {
  std::cout << "taskPerformanceEval cleaning up !" <<std::endl;
}



//---------------------------------------------------------------------------------------------
// updateTaskToSlavePose
//---------------------------------------------------------------------------------------------
void taskPerformanceEval::updateTaskToSlavePose(){

	geometry_msgs::Pose pose_in;

	// read from port
	if (this->port_inp_task_to_slv_tr.connected() && this->port_inp_task_to_slv_tr.read(pose_in) != RTT::NoData ){

		KDL::Frame slv_to_task_frame;
		tf::PoseMsgToKDL(pose_in, slv_to_task_frame);
		this->slv_to_task_frame = slv_to_task_frame.Inverse();
	}

}



//---------------------------------------------------------------------------------------------
// read Curr And Des Poses And Trans form To Task
//---------------------------------------------------------------------------------------------
void taskPerformanceEval::readCurrAndDesPosesAndTransformToTask(KDL::Frame & _curr_pose_in_task, KDL::Frame & _des_pose_in_task){

	geometry_msgs::Pose pose_in;

	// read the current pose
	if (this->port_inp_curr_pose_in_slv.connected() && this->port_inp_curr_pose_in_slv.read(pose_in) == RTT::NewData ){
		this->curr_pose_msg_in_slvrf = pose_in;
		tf::PoseMsgToKDL(pose_in, _curr_pose_in_task);

		// transform to task space
		_curr_pose_in_task = this->slv_to_task_frame * _curr_pose_in_task;
	}

	// read the desired pose
	if (this->port_inp_des_pose_in_slv.connected() && this->port_inp_des_pose_in_slv.read(pose_in) == RTT::NewData ){
		this->des_pose_msg_in_slvrf = pose_in;
		tf::PoseMsgToKDL(pose_in, _des_pose_in_task);

		// transform to task space
		_des_pose_in_task = this->slv_to_task_frame * _des_pose_in_task;
	}

}


void taskPerformanceEval::downsampleAndSendCurrAndDesPoses(const KDL::Frame & _curr_pose_in_task, const KDL::Frame & _des_pose_in_task){

	this->vision_downsmpl_counter++;

	if(this->vision_downsmpl_counter == this->vision_downsmpl_ratio_param){
		this->vision_downsmpl_counter = 0;

		// write the down-sampled current pose

		this->port_out_curr_pose_in_slvrf_downsmpl.write(this->curr_pose_msg_in_slvrf);

		// write the down-sampled desired pose
		this->port_out_des_pose_in_slvrf_downsmpl.write(this->des_pose_msg_in_slvrf);
	}


}




void taskPerformanceEval::startAcquisition(){

	if (this->task_running == true)
		RTT::log(RTT::Warning) << "Task is already running." << RTT::endlog();
	else{
		// setup the flag
		this->task_running = true;

		// save the time
		this->ticks_from_acq_start = RTT::os::TimeService::Instance()->getTicks();

		// send the event out
		this->char_msg.data = 's';
		this->port_out_events.write(this->char_msg);

		// reset the counters
		this->ws_total_samples = 0;
		this->ws_boundary_samples = 0;

		// reset other metrics
		this->num_clutchings = 0;
		this->mstr_tot_displacement = 0.0;
		this->error_sqrd_sum    	= 0.0;
		this->rmse_counter 			= 0;
		this->max_error				= 0.0;
		this->max_vel				= 0.0;
		this->cut_segments			= 0;

		this->lost_contact_with_tissue = true;

		// Initialize mstr_position_last
		if(this->port_inp_master_pose.read(this->pose_msg)!=RTT::NoData){
			this->mstr_position_last = KDL::Vector(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);
		}
		else
			RTT::log(RTT::Error) << "No pose data from the master ." << RTT::endlog();
	}

}



void taskPerformanceEval::endAcquisition(){

	if (this->task_running == false)
		RTT::log(RTT::Warning) << "Task is already finished." << RTT::endlog();
	else{
		// setup the flag
		this->task_running = false;

		// send the event out
		this->char_msg.data = 'e';
		this->port_out_events.write(this->char_msg);

		// saving the accumulative metrics the message and write on the port
		double elapsed_time = RTT::os::TimeService::Instance()->secondsSince(this->ticks_from_acq_start);
		this->writeTimeStamp(elapsed_time, this->vec3_stamped_msg);

		// calculate the rmse
		double rmse = sqrt( this->error_sqrd_sum / double(this->rmse_counter) );


		// find the ratio of outside of ws samples to total number of samples
		double ws_boundary_ratio = double(this->ws_boundary_samples) / double(this->ws_total_samples);
		this->vec3_stamped_msg.vector.x = ws_boundary_ratio;
		// total displacement per minute
		//	this->vec3_stamped_msg.vector.y = this->mstr_tot_displacement / (elapsed_time/60);
		this->vec3_stamped_msg.vector.y = this->mstr_tot_displacement;

		// number of clutches per minute
		//	this->vec3_stamped_msg.vector.z = double(this->num_clutchings)/ (elapsed_time/60);
		this->vec3_stamped_msg.vector.z = double(this->num_clutchings);

		// write on port
		this->port_out_metrics2_downsmpl.write(this->vec3_stamped_msg);

		// out of laziness using a pose message to publish the metrics
		// last three metrics are scaled based on the length of the path.
		this->writeTimeStamp(this->ac_length, this->pose_stamped_msg);
		this->pose_stamped_msg.pose.position.x 		= rmse;
		this->pose_stamped_msg.pose.position.y 		= this->max_error;
		this->pose_stamped_msg.pose.position.z 		= this->max_vel;
		this->pose_stamped_msg.pose.orientation.x 	= ws_boundary_ratio;
		this->pose_stamped_msg.pose.orientation.y	= this->mstr_tot_displacement  ;
		this->pose_stamped_msg.pose.orientation.z 	= this->cut_segments  ;
		this->pose_stamped_msg.pose.orientation.w 	= elapsed_time  ;


		// write on port
		this->port_out_metrics_all.write(this->pose_stamped_msg);

	}
}


//---------------------------------------------------------------------------------------------
// closeToSigmaWorkSpaceBoundary
//---------------------------------------------------------------------------------------------
bool taskPerformanceEval::isCloseToSigmaWorkSpaceBoundary(const KDL::Vector& _position){

	// This transformation makes the comparison easier by aligning the workspace with the
	// coordinate frame axes and shifting it to the center,
	KDL::Vector position_transformed = this->sigma_workspace_tr * _position;

	// The workspace can be approximated by a half sphere with radius 0.11m.
	if (position_transformed[0]>0 && position_transformed.Norm() < 0.11)
		return false;
	else
		return true;

}



template <typename type> void taskPerformanceEval::writeTimeStamp(double seconds, type & b){

	double fractpart, intpart;
	fractpart = modf (seconds , &intpart);
	b.header.stamp.sec = uint32_t(intpart);
	b.header.stamp.nsec = uint32_t(fractpart*1000000000);

}
/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(taskPerformanceEval)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(taskPerformanceEval)
