#ifndef OROCOS_ACGEOMGEN_COMPONENT_HPP
#define OROCOS_ACGEOMGEN_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <kdl/frames.hpp>
#include <geometry_msgs/typekit/Types.hpp>
#include <std_msgs/typekit/Types.hpp>



//----------------------------------------------------------------------------------------------
// AC SQUARE CLASS
//----------------------------------------------------------------------------------------------

class acSquare{
public:

	acSquare(std::vector<double> inp);

	void getClosestPoint( double px, double py, geometry_msgs::Pose & cp);

private:

	// center of the square [x,y,z]
	std::vector<double> center;

	double width; // x
	double height; // y
};



//----------------------------------------------------------------------------------------------
// AC CIRCLE CLASS
//----------------------------------------------------------------------------------------------

class acCircle{
public:

	acCircle(std::vector<double> inp);

	void getClosestPoint( double px, double py, geometry_msgs::Pose & cp);

private:

	// center of the square [x,y,z]
	std::vector<double> center;

	double radious;
};





//----------------------------------------------------------------------------------------------
// OROCOS COMPONENT CLASS
//----------------------------------------------------------------------------------------------

class AcGeomGen : public RTT::TaskContext{
public:
	AcGeomGen(std::string const& name);
	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

	void changeAcGeometry(int in);


private:
	geometry_msgs::Pose tool_current_pose_msg, tool_desired_pose_msg;
	KDL::Vector tool_current_pos, tool_desired_pos;

	// constraint geometries
	acSquare * ac_sq;
	acCircle * ac_ci;

	// properties
	std::vector<double> square_ac_info_prop;
	std::vector<double> circle_ac_info_prop;

	int ac_geometry;
protected:

	// ports
	RTT::InputPort<geometry_msgs::Pose> 					port_tool_pose_current;
	RTT::OutputPort<geometry_msgs::Pose> 					port_tool_pose_desired;

};
#endif



//----------------------------------------------------------------------------------------------
// OTHER FUNCTIONS
//----------------------------------------------------------------------------------------------


void poseMsgToPositionKDLVec( geometry_msgs::Pose pose_in, KDL::Vector &vec_out);



template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
