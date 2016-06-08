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

    \author    <http://nearlab.polimi.it/>
    \author    Nima Enayati
    \version   -
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef SRC_ACMETHODS_HPP_
#define SRC_ACMETHODS_HPP_
#include <iostream>
#include <limits>
#include <kdl/frames.hpp>

//==============================================================================
/*!
    \file       acMethods.hpp

    \brief
    Dynamic active constraints methods
*/
//==============================================================================

//==============================================================================
/*!
    \class      ac

    \brief
    This class implements an active constraint enforcement method.

    \details
	This class and it's three derived classes generate non-energy-storing active
	constraint force based on the position and linear velocity of the tool and
	the desired position (where the tool should go, e.g. closest point on the
	reference path). For a detailed description and comparison of these methods
	please refer to Enayati et al. 2016 [FIX]
*/
//==============================================================================

//-----------------------------------------------------------------------
// ABSTRACT BASE CLASS
//-----------------------------------------------------------------------

class ac{

public:

	//! virtual destructor
	virtual ~ac(){}

	//! pure virtual function for the calculation of the force.
	virtual void getForce(KDL::Vector &f_out,  const KDL::Vector p_tool, const KDL::Vector p_desired, const KDL::Vector v_msrd) =0;

	//! Sets the maximum force of the constraint method
	void setFmax(double in){F_MAX=in;}

	//! Sets the boundary threshold  (if applicable) of the constraint method
	void setBoundaryThreshold(double in){BOUNDARY_THRESHOLD=in;}

	//! Sets the elastic length (if applicable)  of the constraint method
	void setElasticLength(double in){ELASTIC_LENGTH=in;}

protected:
	//! F_MAX is the maximum force generated by an active constraint
	double F_MAX;
	//! In some methods, to improve boundary crossing the constraint is relaxed
	//! a bit in vicinity of BOUNDARY_THRESHOLD mm of the reference
	double BOUNDARY_THRESHOLD;
	//! Since the plastic methods are impedance type, an initial elastic phase
	//! is required
	double ELASTIC_LENGTH;
};


//-----------------------------------------------------------------------
// SIMULATED PLASTICY INTRODUCED BY KIKUUWE ET AL. 2008
//-----------------------------------------------------------------------
class acPlast: public ac{
public:

	//! the constructor
	acPlast(double F_MAX, double ELASTIC_LENGTH, double BOUNDARY_THRESHOLD);

	//! This method calculates the force
	void getForce(KDL::Vector &f_out,  const KDL::Vector p_tool, const KDL::Vector p_desired, const KDL::Vector v_msrd);

private:
	// internal variable the method needs to keep a track of
	KDL::Vector p_tool_last;
	KDL::Vector q;

};


//-----------------------------------------------------------------------
// PLASTIC WITH MOTION REDIRECTION - INTRODUCED BY BOWYER ET AL. 2013
//-----------------------------------------------------------------------
class acPlastRedirect: public ac{
public:

	//! the constructor
	acPlastRedirect(double F_MAX, double ELASTIC_LENGTH, double BOUNDARY_THRESHOLD);

	//! This method calculates the force
	void getForce(KDL::Vector &f_out,  const KDL::Vector p_tool, const KDL::Vector p_desired, const KDL::Vector v_msrd);

private:
	// internal variable the method needs to keep a track of
	KDL::Vector p_tool_last;
	KDL::Vector z;

};



//-----------------------------------------------------------------------
// VISCOUSE WITH REDIRECTION - INTRODUCED BY ENAYATI ET AL. 2016
//-----------------------------------------------------------------------
class acViscousRedirect: public ac{
public:

	//! the constructor
	acViscousRedirect(double F_MAX, double B_MAX, double BOUNDARY_THRESHOLD);

	//! This method calculates the force
	void getForce(KDL::Vector &f_out,  const KDL::Vector p_tool, const KDL::Vector p_desired, const KDL::Vector v_msrd);
	void setMaxViscousity(double in){B_MAX=in;}

private:
	//! the maximum viscosity coefficient
	double B_MAX;
	//! internal variable the method needs to keep a track of
	KDL::Vector f_dir_last;
	KDL::Vector v_dir_last;
	KDL::Vector n_2_last;

};


//-----------------------------------------------------------------------
//-----------------------------------------------------------------------


//-----------------------------------------------------------------------
// TOOLBOX FUNCTIONS - SHARED BETWEEN CLASSES
//-----------------------------------------------------------------------

namespace toolbox{

//! This function rotates vector_in around vector n for THETA degrees and writes it on vvector_out
int rotateVector(const KDL::Vector vector_in, KDL::Vector &vector_out,  const KDL::Vector n, const double THETA);

//! This function normalizes vec_in and if the norm is zero, it instead outputs vec_safe
int normalizeSafe(KDL::Vector & vec_in, const KDL::Vector vec_safe);

//! This function limits the value of x to the set [a,b]
double saturate (double a, double x, double b);

//! This function limits the norm of the vector x to a maximum of a
KDL::Vector saturate_vec(KDL::Vector x, double a);

//! This function calculates the norm of vector x
double vec_norm(KDL::Vector x);

}



#endif /* SRC_ACMETHODS_HPP_ */
