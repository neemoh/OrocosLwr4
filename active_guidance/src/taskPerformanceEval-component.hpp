#ifndef OROCOS_DECIMATOR_COMPONENT_HPP
#define OROCOS_DECIMATOR_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <geometry_msgs/typekit/Types.hpp>
#include <std_msgs/typekit/Types.hpp>
#include <kdl/frames.hpp>
#include <tf_conversions/tf_kdl.h>
#include <active_guidance/perfmetrics.h>
//#include <rtt/Time.hpp>

class taskPerformanceEval : public RTT::TaskContext{
  public:
    taskPerformanceEval(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    // read the task space to slave pose
    void updateTaskToSlavePose();

    // reads the current and desired poses and transforms them to task space
    void readCurrAndDesPosesAndTransformToTask(KDL::Frame & curr_pose_in_task, KDL::Frame & des_pose_in_task);

    // reads the current and desired poses and transforms them to task space
    void downsampleAndSendCurrAndDesPoses(const KDL::Frame & curr_pose_in_task, const KDL::Frame & des_pose_in_task);

    // operation function for starting the acwuisition
    void startAcquisition();

    void endAcquisition();

    bool isCloseToSigmaWorkSpaceBoundary(const KDL::Vector&_position);

    template <typename type> void writeTimeStamp(double seconds, type & b);


  private:

    bool task_running;
    unsigned int vision_downsmpl_counter;
    unsigned int recording_downsmpl_counter;

    KDL::Frame slv_to_task_frame;
    KDL::Frame curr_pose_in_taskrf;
	KDL::Frame des_pose_in_taskrf;
    geometry_msgs::Pose curr_pose_msg_in_slvrf;
    geometry_msgs::Pose des_pose_msg_in_slvrf;
	KDL::Vector kdl_vec;

	// error metrics
	double error_sqrd_sum;
	unsigned int rmse_counter;
	double max_error;

	double max_vel;
	unsigned int cut_segments;
	bool lost_contact_with_tissue;

	double ac_length;
	std_msgs::Float64 float_msg;

	// clutching metric
	unsigned long int num_clutchings;;

	//Master metrics
	bool ws_alert;
	bool ws_alert_last;
	unsigned long int ws_total_samples;
	unsigned long int ws_boundary_samples;
	KDL::Frame sigma_workspace_tr;
	double mstr_tot_displacement;
	KDL::Vector mstr_position;
	KDL::Vector mstr_position_last;
	bool clutch_engaged;
	bool new_clutch_msg;

	// time related variables
	RTT::os::TimeService::ticks ticks_from_acq_start;
	double elapsed_time;

	// messages
	KDL::Twist twist_in_slave;
    geometry_msgs::Pose pose_msg;
    geometry_msgs::PoseStamped pose_stamped_msg;
    geometry_msgs::Twist twist_msg;
    geometry_msgs::Wrench wrench_msg;
    std_msgs::Int8 int8_msg;
    geometry_msgs::PointStamped point_stamped_msg;
	active_guidance::perfmetrics perf_metrics_msg;

    geometry_msgs::Vector3Stamped vec3_stamped_msg;
	std_msgs::Char char_msg;

  protected:
    unsigned int vision_downsmpl_ratio_param;
    unsigned int recording_downsmpl_ratio_param;

	RTT::InputPort<geometry_msgs::Pose> 		port_inp_curr_pose_in_slv;
	RTT::InputPort<geometry_msgs::Pose> 		port_inp_des_pose_in_slv;
	RTT::InputPort<geometry_msgs::Pose> 		port_inp_task_to_slv_tr;
	RTT::InputPort<geometry_msgs::Pose> 		port_inp_master_pose;
	RTT::InputPort<geometry_msgs::Twist> 		port_inp_twist_in_slv;
	RTT::InputPort<std_msgs::Int8> 				port_inp_clutch;
	RTT::InputPort<geometry_msgs::Wrench> 		port_inp_wrench;
	RTT::InputPort<std_msgs::Float64>			port_inp_ac_length;



	// these two are down-sampled current and desired poses of the tool sent to the
	// vision node
	RTT::OutputPort<geometry_msgs::Pose> 		port_out_curr_pose_in_slvrf_downsmpl;
	RTT::OutputPort<geometry_msgs::Pose> 		port_out_des_pose_in_slvrf_downsmpl;
	//Down sampled cutting pose for evaluation
	RTT::OutputPort<geometry_msgs::PoseStamped> 		port_out_cutting_pose_in_taskrf_downsmpl;
	// Down sampled velocity for performance evaluation
	RTT::OutputPort<geometry_msgs::Twist> 		port_out_twist_in_slv_downsmpl;
	// number of clutches for evaluation
	RTT::OutputPort<geometry_msgs::PointStamped> port_out_clutch_stamped;
	// Down-sampled ac force for performance evaluation
	RTT::OutputPort<geometry_msgs::PoseStamped> port_out_mstr_pose_downsmpl_stamped;
	// Sending the norms of error, velocity and force
	RTT::OutputPort<geometry_msgs::Vector3Stamped> 	port_out_metrics1_downsmpl;
	RTT::OutputPort<geometry_msgs::Vector3Stamped> 	port_out_metrics2_downsmpl;
	RTT::OutputPort<active_guidance::perfmetrics> 	port_out_metrics_all;
	// events
	RTT::OutputPort<std_msgs::Char> 			port_out_events;
};
#endif
