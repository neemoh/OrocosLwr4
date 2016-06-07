#ifndef OROCOS_HAPTICACC_COMPONENT_HPP
#define OROCOS_HAPTICACC_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#include <std_msgs/typekit/Types.hpp>
#include <geometry_msgs/typekit/Types.hpp>
#include <sensor_msgs/typekit/Types.hpp>

#include </usr/include/dhdc.h>
#include </usr/include/drdc.h>
#include <kdl/frames.hpp>
#include <tf_conversions/tf_kdl.h>
#include <kdl/kdl.hpp>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <fcntl.h>

using namespace RTT;
using namespace std;
double saturate (double a, double x, double b);
KDL::Vector sat(KDL::Vector x, double a);
double norm(KDL::Vector x);
void diep(char *s);
void cp2circle( double x, double y, double xc, double yc, double r, double &cpx, double &cpy );
void cp2square( double px, double py, double xc, double yc, double width, double height, double &cpx, double &cpy);
double dist( double p1x, double p1y, double p2x, double p2y ) ;
void min(double * table, int size, double &value, int &index);
double lines( double x, int quad);
KDL::Vector closestPoint( int quad, KDL::Vector point, double search_step, int nsearch, int nguess, double X_LEFT_LIM, double X_RIGHT_LIM );
void  divideAndConquer(int quad, KDL::Vector point, std::vector<double> &init, double search_step, int nguess, double X_LEFT_LIM, double X_RIGHT_LIM);
void tumor(KDL::Vector p, KDL::Vector & cp, double SEARCH_STEP, int NSEARCH, int NGUESS);

struct HapticState{

	double dom_px;
	double dom_py;
	double dom_pz;

	double dom_ox;
	double dom_oy;
	double dom_oz;
	double dom_ow;

	double dom_proxx;
	double dom_proxy;
	double dom_proxz;

	double dom_fx;
	double dom_fy;
	double dom_fz;

	double secondary_px;
	double secondary_py;
	double secondary_pz;

	double secondary_ox;
	double secondary_oy;
	double secondary_oz;
	double secondary_ow;

	double secondary_fx;
	double secondary_fy;
	double secondary_fz;

	double secondary_buton;


	double dx_tumor;
	double dy_tumor;
	double dz_tumor;

	double VCmethod;

};

struct UserOptions{
	int method;
	int shape;
};

class HapticACC : public RTT::TaskContext{
public:
	HapticACC(std::string const& name);
	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

	bool initSigma();
	void setKB(double K, double B);
	void setMS(int M, int S);
	void setFMAX(double);
	int  rotateVector(const KDL::Vector vector_in, KDL::Vector &vector_out,  const KDL::Vector n, const double THETA);
	int  normalizeSafe(KDL::Vector & vec_in, const KDL::Vector vec_safe);
	int udp_create_socket();
	int udp_recv();
	int udp_send();

	KDL::Vector sat(KDL::Vector x, double a);
private:

	HapticState sigma_state;
	UserOptions user_options;
	int dominant_hand;


	// Virtual constraint
	int ds, sent;
	bool first_clutch[2];
	double K, B;
	double temp, C, THETA, V_TOOL_LAST, F_MAX, SCALE,  ori_msrd_init_x,  ori_msrd_init_y,  ori_msrd_init_z,  ori_msrd_init_w;
	double 	dx_tumor, dy_tumor, dz_tumor, motion_range_tumor;
	int counter_tumor, sample_per_cycle_tumor;
	RTT::os::TimeService::ticks time_init;
	KDL::Vector z, q, p_tool[2], p_init[2], p_msrd[2],p_msrd_init[2], v_msrd, f_vc_dir, f_vc_dir_last, f_vc_dir_des, f_vc, cp, p_last,n_2_last, v_tool_dir_last ;
//	KDL::Frame dev_pose_frame[2];
	geometry_msgs::Pose dev_pose[2];
	geometry_msgs::PoseStamped dev_pose_stamped;
	std_msgs::Int8 dev_Button[2], dev_Button_Last, dev_Sent;
	geometry_msgs::Wrench tmp_wrench, mstr_wrench_cmd;
//	std::vector<double> ori_msrd_init;

	// UDP
	int local_port, udp_send_pre_sample;
	string ip_address;
	socklen_t m_sock_addr_len;
	struct sockaddr_in m_remote_addr;

protected:
	// PROPERTIES
	std::vector<double> trackParam_prop;
	int prop_local_port, m_socket, prop_udp_send_pre_sample;
	int prop_sample_per_cycle_tumor;
	int prop_dominant_hand;
	string prop_remote_addr;
	double prop_motion_range_tumor;
	double prop_F_MAX;
	//PORTS
	RTT::OutputPort<geometry_msgs::PoseStamped> pose_read_port;
	RTT::InputPort<sensor_msgs::JointState> trigger;
	RTT::OutputPort<std_msgs::Int8> botton_port;
	RTT::OutputPort<geometry_msgs::Wrench>	force_port;
	RTT::OutputPort<std_msgs::Int8>	sent_port;
};
#endif
