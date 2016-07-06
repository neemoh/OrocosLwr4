#include "toolbox.hpp"

void  conversions::poseMsgToVector(const geometry_msgs::Pose in_pose, vector<double>& out_vec) {

	out_vec.at(0) = in_pose.position.x;
	out_vec.at(1) = in_pose.position.y;
	out_vec.at(2) = in_pose.position.z;
	out_vec.at(3) = in_pose.orientation.x;
	out_vec.at(4) = in_pose.orientation.y;
	out_vec.at(5) = in_pose.orientation.z;
	out_vec.at(6) = in_pose.orientation.w;
}

void  conversions::poseMsgToPositionVector(const geometry_msgs::Pose in_pose, vector<double>& out_vec) {

	out_vec.at(0) = in_pose.position.x;
	out_vec.at(1) = in_pose.position.y;
	out_vec.at(2) = in_pose.position.z;

}
void  conversions::poseStampedToPositionVector(const geometry_msgs::PoseStamped in_pose, vector<double>& out_vec) {

	out_vec.at(0) = in_pose.pose.position.x;
	out_vec.at(1) = in_pose.pose.position.y;
	out_vec.at(2) = in_pose.pose.position.z;

}

void  conversions::vectorToPoseMsg(const vector<double> in_vec, geometry_msgs::Pose& out_pose){

	out_pose.position.x = in_vec.at(0);
	out_pose.position.y = in_vec.at(1);
	out_pose.position.z = in_vec.at(2);
	out_pose.orientation.x = in_vec.at(3);
	out_pose.orientation.y = in_vec.at(4);
	out_pose.orientation.z = in_vec.at(5);
	out_pose.orientation.w = in_vec.at(6);

}

void  conversions::positionVectorToPoseMsg(const vector<double> in_vec, geometry_msgs::Pose& out_pose){

	out_pose.position.x = in_vec.at(0);
	out_pose.position.y = in_vec.at(1);
	out_pose.position.z = in_vec.at(2);
	out_pose.orientation.x = 1.0;
	out_pose.orientation.y = 0.0;
	out_pose.orientation.z = 0.0;
	out_pose.orientation.w = 0.0;

}
void conversions::jointStateToJointPosVector (const sensor_msgs::JointState  in_joint,  vector<double>& out_vec){

	for (unsigned int i = 0; i < in_joint.position.size(); ++i) {
		out_vec.at(i) = in_joint.position.at(i);
	}
}

void conversions::jointPosToVector (const motion_control_msgs::JointPositions in_joint,  vector<double>& out_vec){

	for (unsigned int i = 0; i < in_joint.positions.size(); ++i) {
		out_vec.at(i) = in_joint.positions.at(i);
	}
}

bool conversions::vectorToJointPos (const vector<double> in_vec,  motion_control_msgs::JointPositions& out_joint){

	if (in_vec.size() != out_joint.positions.size() ){
		cout << "ERROR: Function:vectorToJointPos. Input size mismatch" << endl;
		return false;
	}

	for (unsigned int i = 0; i < in_vec.size(); ++i) {
		out_joint.positions.at(i) = in_vec.at(i);
	}
	return true;
}

void JointPosVectorToJointState (const vector<double> in_vec,   sensor_msgs::JointState& out_joint){
	for (unsigned int i = 0; i < in_vec.size(); ++i) {
		out_joint.position.at(i) = in_vec.at(i);
	}

}

void  conversions::poseReset( geometry_msgs::Pose& pose){

	pose.position.x = 0.0;
	pose.position.y = 0.0;
	pose.position.z = 0.0;
	pose.orientation.x = 1.0;
	pose.orientation.y = 0.0;
	pose.orientation.z = 0.0;
	pose.orientation.w = 0.0;

}



void  conversions::vec3Reset(geometry_msgs::Vector3& vec3){

	vec3.x = 0.0;
	vec3.y = 0.0;
	vec3.z = 0.0;
}


void conversions::jointPosReset(motion_control_msgs::JointPositions& in_joint ){

	unsigned int num_joints = in_joint.positions.size();
	for (unsigned int i = 0; i < num_joints; ++i) {
		in_joint.positions.at(i) = 0.0;
	}
}


void  conversions::vectorToVector3(const vector<double> vec, geometry_msgs::Vector3& vec3){

	vec3.x = vec.at(0);
	vec3.y = vec.at(1);
	vec3.z = vec.at(2);
}

void  conversions::vectorToTwist(const vector<double> vec, geometry_msgs::Twist& twist){

	twist.linear.x = vec.at(0);
	twist.linear.y = vec.at(1);
	twist.linear.z = vec.at(2);

}



bool conversions::KDLFrameToVector(KDL::Frame kdlFrame, vector<double> & vector_out) {

	vector_out = vector<double>(7, 0.0);
	tf::Pose tmp_Pose;
	tf::PoseKDLToTF(kdlFrame, tmp_Pose);
	tf::Quaternion tmp_quaternion = tmp_Pose.getRotation();
	tmp_quaternion.normalize();
	vector_out.at(0) = tmp_Pose.getOrigin()[0];
	vector_out.at(1) = tmp_Pose.getOrigin()[1];
	vector_out.at(2) = tmp_Pose.getOrigin()[2];
	vector_out.at(3) = tmp_quaternion[0];
	vector_out.at(4) = tmp_quaternion[1];
	vector_out.at(5) = tmp_quaternion[2];
	vector_out.at(6) = tmp_quaternion[3];
	return true;

}

bool conversions::vectorToKDLFrame(vector<double> vector_in, KDL::Frame &kdlFrame) {

	if(vector_in.size() == 7) {
		kdlFrame = KDL::Frame::Identity();
		tf::Point tmp_translation(vector_in.at(0), vector_in.at(1), vector_in.at(2));
		tf::Quaternion tmp_quaternion(vector_in.at(3), vector_in.at(4), vector_in.at(5), vector_in.at(6));
		tmp_quaternion.normalize();
		tf::Pose tmp_Pose(tmp_quaternion, tmp_translation);
		tf::PoseTFToKDL(tmp_Pose, kdlFrame);
		return true;
	}
	else {
		cout << "ERROR: In conversions::vectorToKDLFrame. Input vector must have 7 elements. Instead it has:" << vector_in.size() << endl;
		return false;
	}


}

geometry_msgs::Quaternion conversions::KDLRotToQuaternionMsg(const KDL::Rotation & in){
	geometry_msgs::Quaternion out;
	in.GetQuaternion(out.x, out.y, out.z, out.w);
	return out;
}

KDL::Rotation  			conversions::quaternionMsgToKDLRot(const geometry_msgs::Quaternion & in){
	KDL::Rotation out;
	out = KDL::Rotation::Quaternion(in.x, in.y,  in.z, in.w);
}


geometry_msgs::Wrench conversions::createWrenchMsg(double fx, double fy, double fz, double tx, double ty, double tz){

	geometry_msgs::Wrench out;
	out.force.x = fx;
	out.force.y = fy;
	out.force.z = fz;

	out.torque.x = tx;
	out.torque.y = ty;
	out.torque.z = tz;
	return out;

}




vector<double>  conversions::radTodeg(const vector<double> in){
	size_t size = in.size();

	vector<double> out = vector<double>(size, 0.0);
	for(int i=0; i<size; i++){
		out[i] = in[i]*180.0 /M_PI;
	}
	return out;

}
bool areVectorsEqual(const vector<double> vec1, const vector<double> vec2, const double tolerance){


	unsigned int v_size = vec1.size();
	double diff_sum= 0.0;

	if(v_size != vec2.size())	{
		cout << "ERROR: Function:areVectorsEqual. Vectors must have the same size" << endl;
		return false;
	}

	for (unsigned int iter=0; iter<v_size ; iter++)
	{
		diff_sum += abs(vec1.at(iter) - vec2.at(iter));
	}
	if (diff_sum <= tolerance){
		return true;}
	else
		return false;

}


bool limitCheck(const vector<double> vars, const vector<double> min_limits, const vector<double> max_limits) {

	if (vars.size() != max_limits.size() || vars.size() != min_limits.size()){
		cout << "ERROR: Function:areVectorsEqual. Vectors must have the same size" << endl;
		return false;
	}

	for (unsigned int iter = 0; iter < vars.size(); iter++)
		if ((vars.at(iter) > max_limits.at(iter)) || (vars.at(iter) < min_limits.at(iter)))
			return false;

	return true;

}

vector<double> sumVectors(const vector<double> a, const vector<double> b){
	vector<double> sum(a.size(), 0.0);

	if (a.size() != b.size()){
		cout << "ERROR: Function:sumVectors. Vectors must have the same size" << endl;
		return sum;
	}


	for (unsigned int iter = 0; iter < a.size(); iter++ ){

		sum.at(iter) = a.at(iter) + b.at(iter);
	}

	return sum;
}

vector<double> subtractVectors(const vector<double> a, const vector<double> b){
	vector<double> sub(a.size(), 0.0);

	if (a.size() != b.size()){
		cout << "ERROR: Function:sumVectors. Vectors must have the same size" << endl;
		return sub;
	}


	for (unsigned int iter = 0; iter < a.size(); iter++ ){

		sub.at(iter) = a.at(iter) - b.at(iter);
	}

	return sub;
}



// Custom made useful function

void printVector(vector<double> vect, string name = "") {
	if (!name.empty())
		cout << name << endl;
	for (unsigned int iter = 0; iter < vect.size(); ++iter) {
		cout << vect.at(iter) << "\t";
	}
	cout << endl;
}


