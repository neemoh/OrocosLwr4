/*
 * acMethods.hpp
 *
 *  Created on: Jun 7, 2016
 *      Author: nearlab
 */

#ifndef SRC_ACMETHODS_HPP_
#define SRC_ACMETHODS_HPP_

#include <rtt/RTT.hpp>
#include <kdl/frames.hpp>
#include <geometry_msgs/typekit/Types.hpp>
#include <std_msgs/typekit/Types.hpp>


class ac{

public:
//	virtual void getForce(KDL::Vector &f_out,  const KDL::Vector p_tool, const KDL::Vector p_desired, const KDL::Vector v_msrd);
	void setFmax(double in){F_MAX=in;}
	void setBoundaryThreshold(double in){F_MAX=in;}

protected:
	double F_MAX;
	double BOUNDARY_THRESHOLD;
	double ELASTIC_LENGTH;
};


// ---------------------------------------------------
class acPlast: public ac{
public:

	acPlast(double F_MAX, double ELASTIC_LENGTH, double BOUNDARY_THRESHOLD);
	void getForce(KDL::Vector &f_out,  const KDL::Vector p_tool, const KDL::Vector p_desired, const KDL::Vector v_msrd);

private:
	KDL::Vector p_tool_last;
	KDL::Vector q;

};


// ---------------------------------------------------
class acPlastRedirect: public ac{
public:

	acPlastRedirect(double F_MAX, double ELASTIC_LENGTH, double BOUNDARY_THRESHOLD);
	void getForce(KDL::Vector &f_out,  const KDL::Vector p_tool, const KDL::Vector p_desired, const KDL::Vector v_msrd);

private:
	KDL::Vector p_tool_last;
	KDL::Vector z;

};



// ---------------------------------------------------
class acViscousRedirect: public ac{
public:

	acViscousRedirect(double F_MAX, double B_MAX, double BOUNDARY_THRESHOLD);
	void getForce(KDL::Vector &f_out,  const KDL::Vector p_tool, const KDL::Vector p_desired, const KDL::Vector v_msrd);

private:
	double B_MAX;
	KDL::Vector f_dir_last;
	KDL::Vector v_dir_last;
	KDL::Vector n_2_last;

};






namespace toolbox{
// useflu functions
int rotateVector(const KDL::Vector vector_in, KDL::Vector &vector_out,  const KDL::Vector n, const double THETA);
int normalizeSafe(KDL::Vector & vec_in, const KDL::Vector vec_safe);
double saturate (double a, double x, double b);
KDL::Vector saturate_vec(KDL::Vector x, double a);
double vec_norm(KDL::Vector x);
}



#endif /* SRC_ACMETHODS_HPP_ */
