#ifndef TOOLBOX_HPP_
#define TOOLBOX_HPP_

#include <iostream>
#include <geometry_msgs/typekit/Types.h>
#include <sensor_msgs/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.hpp>
#include <kdl/frames.hpp>
#include <tf_conversions/tf_kdl.h>
#include <std_msgs/Int8.h>

using namespace std;

namespace conversions  {

	void poseMsgToVector(const geometry_msgs::Pose ,  vector<double>& );
	void poseMsgToPositionVector(const geometry_msgs::Pose , vector<double>&);

	void poseStampedToPositionVector(const geometry_msgs::PoseStamped , vector<double>&);

	void positionVectorToPoseMsg(const vector<double> , geometry_msgs::Pose& );

	void jointStateToJointPosVector (const sensor_msgs::JointState ,  vector<double>& );
	void jointPosToVector (const motion_control_msgs::JointPositions ,  vector<double>& );

	bool vectorToJointPos (const vector<double> ,  motion_control_msgs::JointPositions& );
	void JointPosVectorToJointState (const vector<double> ,   sensor_msgs::JointState& );
	void vectorToVector3(const vector<double> , geometry_msgs::Vector3& );
	void vectorToPoseMsg(const vector<double> , geometry_msgs::Pose&);
	void vectorToTwist(const vector<double> vec, geometry_msgs::Twist& twist);
	bool KDLFrameToVector(KDL::Frame kdlFrame, vector<double> & vector) ;
	bool vectorToKDLFrame(vector<double> matrixvector, KDL::Frame &matrix);

	geometry_msgs::Wrench createWrenchMsg(double fx, double fy, double fz, double tx, double ty, double tz);

	vector<double>  radTodeg(const vector<double> in);

	void poseReset(geometry_msgs::Pose& );
	void vec3Reset(geometry_msgs::Vector3& );
	void jointPosReset(motion_control_msgs::JointPositions& );

};


//Other functions
bool areVectorsEqual(const vector<double>v1, const vector<double>v2, const double tolerance);
bool limitCheck(vector<double> vars, vector<double> min_limits, vector<double> max_limits);
vector<double> subtractVectors(const vector<double> a, const vector<double> b);
vector<double> sumVectors(const vector<double> a, const vector<double> b);

void printVector(vector<double> vect, string ) ;

static ostream& operator<<(ostream& out, const vector<double>& vect){
	for (unsigned int iter = 0; iter < vect.size(); ++iter) {
		out << "[" << iter <<"]: "<<vect.at(iter) << "\t";
	}
	return out;
}

static ostream& operator<<(ostream& out, const KDL::Vector& kdlV){
	for (unsigned int iter = 0; iter < 3; ++iter) {
		out << "[" << iter <<"]: "<<kdlV[iter] << "\t";
	}
	return out;
}
#endif /* TOOLBOX_HPP_ */
