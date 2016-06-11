/*
 * LWR_Kinematics.hpp
 *
 *  Created on: Mar 4, 2016
 *      Author: nearlab
 */

#ifndef LWR4_KINEMATICS_HPP_
#define LWR4_KINEMATICS_HPP_
#include <rtt/RTT.hpp>

//Headeres
#include <iostream>
#include <cmath>
#include <kdl/frames.hpp>
#include <tf_conversions/tf_kdl.h>

namespace LWR4_Kinematics{

	bool IK(const KDL::Frame cartesian_matrix,         const unsigned int config,   const double psi,    const double ztool,   std::vector<double>& joint_positions);
	bool FK(const std::vector<double> joint_positions, unsigned int & config,       double & psi,  		 double ztool,    	   KDL::Frame &cartesian_matrix) ;
	bool redundancyCircle(const KDL::Frame T, const unsigned int config, const std::vector<double> psi, double ztool,double& joint_four, std::vector<std::vector<double> >& six_joints);
	bool validJointsForPsiVector(const KDL::Frame T, const unsigned int config, const std::vector<double> psi, double ztool,double& joint_four, std::vector<std::vector<double> >& valid_six_joints, std::vector<double> & valid_psis);
	bool validJointsForCurrentArc(const KDL::Frame T, const unsigned int config, double psi_curr,
			double ztool,std::vector<double> &psi_arc);

	bool IKPart1(const KDL::Frame T, const double ztool,double& joint_four,
			tf::Matrix3x3& As, tf::Matrix3x3& Bs, tf::Matrix3x3& Cs, tf::Matrix3x3& Aw,
			tf::Matrix3x3& Bw,tf::Matrix3x3& Cw);
	void IKPart2(const double psi, const unsigned int config, double& joint_four,
			tf::Matrix3x3& As, tf::Matrix3x3& Bs, tf::Matrix3x3& Cs, tf::Matrix3x3& Aw,
			tf::Matrix3x3& Bw,tf::Matrix3x3& Cw, std::vector<double>& six_joints);
	// Helper functions
	void modAngle(double &angle);
};

#endif /* SRC_MYAPI_LWR4_KINEMATICS_HPP_ */
