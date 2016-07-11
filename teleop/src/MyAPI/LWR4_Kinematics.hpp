//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2016, Nearlab


    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

 * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

 * Neither the name of nearlab nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author		<http://nearlab.polimi.it/>
    \author		Nima Enayati
    \Date		March - 2016
 */
//==============================================================================

//------------------------------------------------------------------------------
#ifndef LWR4_KINEMATICS_HPP_
#define LWR4_KINEMATICS_HPP_

//includes
#include <iostream>
#include <cmath>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <tf_conversions/tf_kdl.h>
#include <Eigen/Dense>
#include <Eigen/Core>

//==============================================================================
/*!
    \file       LWR4_Kinematics.hpp

    \brief
    Kinematics methods for the LWR
 */
//==============================================================================

//==============================================================================
/*!
    \class      LWR4Kinematics

    \brief
    This class contains the forward and inverse kinematics methods and available
    redundancy parameter set methods.

    \details
	The inverse kinematics is written based the 2008 paper by Shimizu, M. et al.
	The dh parameters used here is based on KUKA LWR4's parameters that	differs
	with that of Shimizu in alpha:
	         'kuka':    alpha = [pi/2 -pi/2 -pi/2 pi/2 pi/2 -pi/2 0]
	         'shimizu': alpha = [-pi/2 pi/2 -pi/2 pi/2 -pi/2 pi/2 0]
	The IK is divided in two functions because one part of the method is constant
	for all arm angles psi. Therefore in case the joint
	positions for more than 1 arm angle is needed first part can be calculated
	once and the second part will produce the joint positions for each arm angle.

	some parameters involved are:
	T:  Desired Cartesian pose
	Config: 		  8 possible solutions may be available givven T and psi. config is
	         		  an int8 (0 to 7) that describes which one of these possible
	        		  solutions is wanted.
	psi:   		  	  The redundancy parameter (AKA arm angle) (rad)
	q:		  		  Calculated joint positions.

	The code is not that readable! Check the Matlab codes if you need to modify
	Something.

 */
//==============================================================================


//------------------------------------------------------------------------------
// LWR4KinematicsCLASS
//------------------------------------------------------------------------------
class LWR4Kinematics{

public:

	// CONSTRUCTOR
	// _tool_to_ee:   This is the transformation matrix from the tool installed on the robot
	//				  to the end-effector reported in the end-effector coordinate frame.
	//				  Set identity if no tool is attached. Note that if the information of the
	//				  tool is different from that set in the krc the controller will not work
	//				  properly.


	LWR4Kinematics(KDL::Frame _tool_to_ee);

	// FK METHOD
	// Finds the Cartesian pose, the config and psi for a given joint positions vector.
	void fk(const std::vector<double> _q_fk, unsigned int & _config, double & _psi, KDL::Frame &cartesian_matrix);

	// FK_all
	// Finds the Cartesian pose, the config and psi for a given joint positions vector.
	void fk_all(const std::vector<double> _q_fk, unsigned int & _config, double & _psi, std::vector<KDL::Frame> &joint_frames);

	// Jacobian
	// gets the outbput frame vector of the fk_all and calculates the
	void jacobian( const std::vector<KDL::Frame> & linkMatrices, KDL::Jacobian & jac);

	// get Manipulability
	// Calculates the manipulability index for a given Jacobian
	// dof_param=1 : Translation index, dof_param=2 : Rotation index, else: both T and R
	void getManipulabilityIdx(const KDL::Jacobian jac, unsigned int dof_param, double & manp_idx);

	// IK METHOD
	// Finds the joint positions q for given T, config and psi
	bool ik(const KDL::Frame _T, const unsigned int _config, double _psi,
			std::vector<double>& _q);

	// Calculates valid arm angles from the current psi by simply incrementing
	// and decrementing the current psi with 1 degree steps and checking the
	// validity based on joint limits. The search stops on each side when
	// an invalid joint position is reached.
	bool validJointsForCurrentArc(const KDL::Frame _T, const unsigned int _config, double _psi_curr,
			std::vector<double> &psi_arc);

	// Calculates valid arm angles for a vector of arm angle values psi
	bool validJointsForPsiVector(const KDL::Frame T, const unsigned int config,
			const std::vector<double> psi,	std::vector<std::vector<double> >& valid_joints,
			std::vector<double>& valid_psis);

private:

	// calculates joint 4 and the reference matrices that are constant for all
	// psi values
	bool ikPart1(std::vector<double>& q);

	// calculates the joint positions for specific psi
	void ikPart2(const unsigned int _config, const double _psi, std::vector<double>& q);

	// handy functions
	void modAngle(double &a);



private:

	//DH parameters - lengths
	double dbs;
	double dse;
	double dew;
	double dwt;

	// joint limits
	double jlim1; // rads = 170 degrees for joints: 1 3 5 7
	double jlim2; // rads = 120 degrees for joints: 2 4 6

	// the Cartesian destination. end-effector to base
	KDL::Frame T_ee_to_base;

	// configuration parameter calculated from FK
	unsigned int config_fk;

	// number of joints
	unsigned int n_joints;

	// the arm angle (redundancy parameter) calculated from FK
	double psi_fk;

	// transformation from the tool installed on the robot to the end-effector
	// and its inverse.
	KDL::Frame tr_tool_to_ee, tr_ee_to_tool;

	// IK matrices reference matrices
	tf::Matrix3x3 As, Bs, Cs,Aw, Bw, Cw;

	// vector from base to shoulder
	tf::Vector3 vbs;
	// vector from wrist to tool
	tf::Vector3 vwt;
};


#endif /* SRC_MYAPI_LWR4_KINEMATICS_HPP_ */
