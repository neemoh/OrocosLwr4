#include "acGeomGen-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

AcGeomGen::AcGeomGen(std::string const& name) : TaskContext(name){

	this->addEventPort("tool_pose_current", this->port_tool_pose_current).doc("Reading the current pose of the tool");
	this->addPort("tool_pose_desired", this->port_tool_pose_desired).doc("Writing the desired pose of the tool");
	this->addPort("ac_point_cloud", this->port_ac_point_cloud).doc("Writing the desired pose of the tool");

	this->addProperty("square_ac_info",	square_ac_info_prop).doc("Properties of the square constraint.");
	this->addProperty("circle_ac_info",	circle_ac_info_prop).doc("Properties of the circle constraint.");

	this->ac_received = false;

	ac_geometry = 1;
	std::cout << "AcGeomGen constructed !" <<std::endl;
}

bool AcGeomGen::configureHook(){

	// constructing the geometries
	this->ac_sq = new acSquare(square_ac_info_prop);
	this->ac_ci = new acCircle(circle_ac_info_prop);

	// initialize port
	this->port_tool_pose_desired.setDataSample(this->tool_current_pose_msg);

	std::cout << "AcGeomGen configured !" <<std::endl;
	return true;
}


//----------------------------------------------------------------------------------------------
// START HOOK
//----------------------------------------------------------------------------------------------
bool AcGeomGen::startHook(){

	std::cout << "AcGeomGen started !" <<std::endl;
	return true;
}


//----------------------------------------------------------------------------------------------
// UPDATE HOOK
//----------------------------------------------------------------------------------------------
void AcGeomGen::updateHook(){

	// Reading data. Not checking for RTT::NewData
	this->port_tool_pose_current.read(this->tool_current_pose_msg);

	//
	if(this->port_ac_point_cloud.read(this->ac_point_cloud) == RTT::NewData){
		std::cout << "New ac point cloud received" << std::endl;
		this->pointCloudToVector(this->ac_point_cloud, this->ac_points);

		// flag
		if(this->ac_points.size()>0)
			this->ac_received = true;
	}
	//	poseMsgToPositionKDLVec(this->tool_current_pose_msg,this->tool_current_pos);
	if (this->ac_received){
		switch(this->ac_geometry){
		case 1:
			this->closestPointToACPoints(this->tool_current_pose_msg.position.x,
					this->tool_current_pose_msg.position.y,
					this->tool_current_pose_msg.position.z,
					this->tool_desired_pose_msg);
			break;
		case 2:
			this->ac_sq->getClosestPoint( this->tool_current_pose_msg.position.x, this->tool_current_pose_msg.position.y,
					this->tool_desired_pose_msg);
			break;
		case 3:
			this->ac_ci->getClosestPoint( this->tool_current_pose_msg.position.x, this->tool_current_pose_msg.position.y,
					this->tool_desired_pose_msg);
			break;
		default:
			// do nothing
			break;
		}

		// publishing the desired tool pose on the port
		if(this->ac_geometry==1 || this->ac_geometry ==2)
			this->port_tool_pose_desired.write(this->tool_desired_pose_msg);
	}
}


//----------------------------------------------------------------------------------------------
// CLOSING ANC CLEANING UP
//----------------------------------------------------------------------------------------------
void AcGeomGen::stopHook() {
	RTT::Logger::In in(this->getName());

	std::cout << "AcGeomGen executes stopping !" <<std::endl;
}

void AcGeomGen::cleanupHook() {
	RTT::Logger::In in(this->getName());

	delete ac_sq;
	delete ac_ci;
	std::cout << "AcGeomGen cleaning up !" <<std::endl;
}


//----------------------------------------------------------------------------------------------
// CHANGE AC GEOMETRY
//----------------------------------------------------------------------------------------------
void AcGeomGen::changeAcGeometry(int inp){
	RTT::Logger::In in(this->getName());

	if(this->ac_geometry==inp)
		log(RTT::Warning) << "AcGeometry was already " << ac_geometry << RTT::endlog();
	else
		this->ac_geometry = inp;


}


//-----------------------------------------------------------------------
// converting position of pose message to a KDL vec
//-----------------------------------------------------------------------

void poseMsgToPositionKDLVec( geometry_msgs::Pose pose_in, KDL::Vector &vec_out){
	vec_out[0] = pose_in.position.x;
	vec_out[1] = pose_in.position.y;
	vec_out[2] = pose_in.position.z;
}


//----------------------------------------------------------------------------------------------
// AC SQUARE CLASS
//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------
// CONSTRUCTOR
//----------------------------------------------------------------------------------------------
acSquare::acSquare(std::vector<double> in){
	// input is a vector containing: [center_x, center_y, center_z, width, height]
	center = std::vector<double>(3,0.0);
	center[0]= in[0];
	center[1]= in[1];
	center[2]= in[2];

	width = in[3];
	height = in[4];

}

//-----------------------------------------------------------------------
// CLOSEST POINT TO THE GEOMETRY
//-----------------------------------------------------------------------
void acSquare::getClosestPoint(double tool_x, double tool_y, geometry_msgs::Pose & cp_pose_msg) {

	// take the tool point to the local frame
	tool_x -= center[0];
	tool_y -= center[1];

	if(fabs(tool_y)>fabs(tool_x)){
		cp_pose_msg.position.y= height*double(sgn(tool_y));
		cp_pose_msg.position.x= tool_x;
	}
	else{
		cp_pose_msg.position.x= width*double(sgn(tool_x));
		cp_pose_msg.position.y= tool_y;
	}

	// going back to the global reference
	cp_pose_msg.position.x += center[0];
	cp_pose_msg.position.y += center[1];
	// z is constant
	cp_pose_msg.position.z = center[2];
};



//----------------------------------------------------------------------------------------------
// AC CIRCLE CLASS
//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------
// CONSTRUCTOR
//----------------------------------------------------------------------------------------------
acCircle::acCircle(std::vector<double> in){
	// input is a vector containing: [center_x, center_y, center_z, width, height]
	center = std::vector<double>(3,0.0);
	center[0]= in[0];
	center[1]= in[1];
	center[2]= in[2];

	radious = in[3];

}


void acCircle::getClosestPoint(double tool_x, double tool_y, geometry_msgs::Pose & cp_pose_msg) {

	// take the tool point to the local frame
	tool_x -= center[0];
	tool_y -= center[1];

    // finding the closest point
    double mag = sqrt(tool_x*tool_x + tool_y*tool_y);
    cp_pose_msg.position.x =  tool_x / mag * radious;
    cp_pose_msg.position.y =  tool_y / mag * radious;

	// going back to the global reference
	cp_pose_msg.position.x += center[0];
	cp_pose_msg.position.y += center[1];
	// z is constant
	cp_pose_msg.position.z = center[2];

}


void AcGeomGen::pointCloudToVector(const sensor_msgs::PointCloud2 & input, std::vector< std::vector<double> > &vecetor_out){

	size_t size = input.width * input.height;
	int x_idx = getPointCloud2FieldIndex (input, "x");
	int y_idx = getPointCloud2FieldIndex (input, "y");
	int z_idx = getPointCloud2FieldIndex (input, "z");

	int x_offset = input.fields[x_idx].offset;
	int y_offset = input.fields[y_idx].offset;
	int z_offset = input.fields[z_idx].offset;

	std::vector<double> point = std::vector<double>(3, 0.0);
	for (size_t cp = 0; cp < size; ++cp)
	{
		// Copy x/y/z
		float x,y,z;
	    memcpy (&x, &input.data[cp * input.point_step + x_offset], sizeof (float));
	    memcpy (&y, &input.data[cp * input.point_step + y_offset], sizeof (float));
	    memcpy (&z, &input.data[cp * input.point_step + z_offset], sizeof (float));

		point[0] = double(x);
		point[1] = double(y);
		point[2] = double(z);

		vecetor_out.push_back(point);
	}

//	for (size_t cp = 0; cp < size; ++cp){
//		std::cout<< "x: " << vecetor_out[cp][0] << " y: " << vecetor_out[cp][1] << " z: "<<  vecetor_out[cp][2] << std::endl;
//	}
}


void AcGeomGen::closestPointToACPoints(double _tool_x, double _tool_y, double _tool_z, geometry_msgs::Pose & cp_pose_msg){

	double min_d = 100000; // something large
	size_t i_min = 0;
	//	cout << ac_points.size() << endl;

	for(size_t i=0; i<ac_points.size(); i++){
		double dx = _tool_x - ac_points[i][0];
		double dy = _tool_y - ac_points[i][1];
		double dz = _tool_z - ac_points[i][2];
		double norm2 = dx*dx + dy*dy + dz*dz;
		if(norm2 < min_d){
			min_d = norm2;
			i_min = i;
		}

	}
	cp_pose_msg.position.x = ac_points[i_min][0];
	cp_pose_msg.position.y = ac_points[i_min][1];
	cp_pose_msg.position.z = ac_points[i_min][2];

}
/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(AcGeomGen)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(AcGeomGen)
