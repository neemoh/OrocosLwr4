/*
 * acMethods.cpp
 *
 *  Created on: Jun 7, 2016
 *      Author: nearlab
 */
#include "acMethods.hpp"

using namespace toolbox;

acPlast::acPlast(double F_MAX_in, double ELASTIC_LENGTH_in, double BOUNDARY_THRESHOLD_in){
	F_MAX 				= F_MAX_in;
	ELASTIC_LENGTH 		= ELASTIC_LENGTH_in;
	BOUNDARY_THRESHOLD 	= BOUNDARY_THRESHOLD_in;
	p_tool_last 		= KDL::Vector(0.0,0.0,0.0);
	q					= KDL::Vector(0.0,0.0,0.0);

}


void acPlast::getForce(KDL::Vector &f_out,  const KDL::Vector p_tool, const KDL::Vector p_desired, const KDL::Vector v_msrd){

	double F = 0.5;
	double R = F_MAX;
	int Kk = R/ELASTIC_LENGTH;
	double Bk = 1.5;
	double T = 0.001;
	KDL::Vector cp = p_desired;

	KDL::Vector q_last = q;
	double cte = Kk + (Bk/T);
	KDL::Vector ps = p_tool + (Bk * (q - p_tool_last) / (Kk * T + Bk)); // p_tool-d = p_last
	KDL::Vector ki = ps - cp;
	KDL::Vector n = ki;
	normalizeSafe(n, KDL::Vector(0.0,0.0,0.0));

	double a1 = saturate(0.0, dot(n,ki), (R/cte));
	double a2 = saturate(0.0, -dot(n,ki), (R/cte));

	KDL::Vector e = q_last - ps;
	double a3 = saturate(-a1, dot(n,e), a2);

	//I-n*n'
	KDL::Vector In1 = KDL::Vector(1 - n[0]*n[0], 0 - n[0]*n[1], 0 - n[0]*n[2]);
	KDL::Vector In2 = KDL::Vector(0 - n[1]*n[0], 1 - n[1]*n[1], 0 - n[1]*n[2]);
	KDL::Vector In3 = KDL::Vector(0 - n[2]*n[0], 0 - n[2]*n[1], 1 - n[2]*n[2]);
	//(I-n*n')*e
	KDL::Vector mat = KDL::Vector(dot(In1,e), dot(In2,e), dot(In3,e));
	double m = std::max(1.0,( ( cte/F ) * sqrt( ( vec_norm(e)*vec_norm(e) ) - ( dot(n,e)*dot(n,e) ) ) ) );

	q = ps + n * a3 + mat/m;

	KDL::Vector a4 = saturate_vec(e, (F/cte));
	KDL::Vector qf = ps + a4;
	double qfql = vec_norm(qf - q_last);
	double qql = vec_norm(q - q_last);

	if ( qfql <= qql ) q = qf;

	f_out = (Kk * (q - p_tool) + Bk * ( ( q - q_last ) - ( p_tool - p_tool_last ) ) / T);

	p_tool_last = p_tool;
}


//##################################################################################################
// BOWYER
acPlastRedirect::acPlastRedirect(double F_MAX_in, double ELASTIC_LENGTH_in, double BOUNDARY_THRESHOLD_in){

	F_MAX = F_MAX_in;
	ELASTIC_LENGTH = ELASTIC_LENGTH_in;
	BOUNDARY_THRESHOLD = BOUNDARY_THRESHOLD_in;
	p_tool_last 		= KDL::Vector(0.0,0.0,0.0);
	z					= KDL::Vector(0.0,0.0,0.0);

}


void acPlastRedirect::getForce(KDL::Vector &f_out,  const KDL::Vector p_tool, const KDL::Vector p_desired, const KDL::Vector v_msrd){

	double sig2 = 2.5;
	double fc = F_MAX;
	double theta = 0.4;
	double sig0 = fc/0.003;
	double zcss = fc/sig0;
	KDL::Vector penet = p_tool - p_desired;

	// boundary condition
	double ptrans = BOUNDARY_THRESHOLD;
	if ( vec_norm(penet) < ptrans ) theta = theta * (vec_norm(penet)/ptrans);

	z = z + p_tool- p_tool_last;

	KDL::Vector ptn = penet;
	normalizeSafe(ptn, KDL::Vector(0.0,0.0,0.0));
	KDL::Vector zn = z;
	normalizeSafe(zn, KDL::Vector(0.0,0.0,0.0));
	double a = atan2(vec_norm(ptn*zn), dot(zn, ptn));

	KDL::Vector n = penet*z;
	KDL::Vector nn = n;
	normalizeSafe(nn, KDL::Vector(0.0,0.0,0.0));

	KDL::Vector y = cos(theta)*penet + sin(theta)*(nn*penet) + (1-cos(theta))*dot(nn,penet)*nn;
	KDL::Vector yn = y;
	normalizeSafe(yn, KDL::Vector(0.0,0.0,0.0));

	if ( a <= theta && vec_norm(z) <= zcss )
		z = z * 1;
	else if ( a <= theta && vec_norm(z) > zcss )
		z = zcss*zn;
	else
	{
		if ( dot(z,yn) <= 0.0 ) z = KDL::Vector(0.0,0.0,0.0);
		else if ( (0.0 < dot(z,yn)) && (dot(z,yn) < zcss) ) z = dot(z,yn)*yn;
		else z = zcss * yn;
	}

	f_out =  -(sig0*z + sig2*v_msrd);
	p_tool_last = p_tool;
}


//##############################################################################################################


acViscousRedirect::acViscousRedirect(double F_MAX_in, double B_MAX_in, double BOUNDARY_THRESHOLD_in){

	F_MAX = F_MAX_in;
	B_MAX = B_MAX_in;
	BOUNDARY_THRESHOLD = BOUNDARY_THRESHOLD_in;
	v_dir_last = KDL::Vector(1.0,0.0,0.0);
	f_dir_last = KDL::Vector(1.0,0.0,0.0);
	n_2_last   = KDL::Vector(1.0,0.0,0.0);

}


void acViscousRedirect::getForce(KDL::Vector &f_out,  const KDL::Vector p_tool, const KDL::Vector p_desired, const KDL::Vector v_msrd){
	// UPPERCASE NAMES = SCALARS
	// LOWERCASE NAMES = VECTORS

	KDL::Vector penet = p_desired - p_tool;
	double F_VC, F_VC_SAT, V_TOOL;
	double B_M			= B_MAX;

	KDL::Vector  f_dir, penet_dir, v_tool_dir;

	penet_dir 		= penet;
	v_tool_dir 		= v_msrd;

	normalizeSafe(v_tool_dir, v_dir_last);
	normalizeSafe(penet_dir, KDL::Vector(1.0,0.0,0.0));

	double V_PENET_DOTP = dot(v_tool_dir, penet_dir);
	double PENET =vec_norm(penet);

	if (PENET< BOUNDARY_THRESHOLD){
		//			PENET = 0;
		B_M =  B_MAX * PENET/ BOUNDARY_THRESHOLD;
	}
	else
		B_M = B_MAX;

	V_TOOL = vec_norm( v_msrd );
	F_VC = B_M * sqrt( ( 1 - V_PENET_DOTP ) / 2 ) * V_TOOL;

	KDL::Vector n = v_tool_dir * penet_dir;
	KDL::Vector nn = n;
	normalizeSafe(nn, penet_dir);

	if ( V_PENET_DOTP < 0.0 )
		f_dir = penet_dir;
	KDL::Vector n_2 = f_dir_last * f_dir;

	normalizeSafe(n_2, n_2_last);

	F_VC_SAT = saturate(-F_MAX, F_VC , F_MAX );
	f_out = F_VC_SAT * f_dir;

	v_dir_last = v_tool_dir;
	f_dir_last = f_dir;
	n_2_last = n_2;

}








//##############################################################################
// #############################################################################
// #################       COMMON FUNCTIONS         ############################
// #############################################################################
//##############################################################################


int toolbox::rotateVector(const KDL::Vector vector_in, KDL::Vector &vector_out,  const KDL::Vector n, const double THETA){
	if (vec_norm(vector_in) ==0.0 || vec_norm(n) ==0.0){
//		ROS_ERROR("Null vector given as the input of rotation function.");
		return -1;
	}
	else{
		vector_out = vector_in + sin(THETA)*(n*vector_in)+(1.0-cos(THETA))*(n*(n*vector_in));
		return 0;
	}
}
int toolbox::normalizeSafe(KDL::Vector & vec_in, const KDL::Vector vec_safe) {
	// Checks the norm of the vec_in. If it is non zero the function normalizes it.
	// If the norm is zero the function uses vec_safe as the output

	double l = vec_norm(vec_in);
	if( fabs(l) >= std::numeric_limits<double>::epsilon() ) {
		vec_in[0] /= l;
		vec_in[1] /= l;
		vec_in[2] /= l;
		return 0;
	} else {

		vec_in = vec_safe;
		return -1;
	}
}

double toolbox::saturate (double a, double x, double b){

	if (x < a){
		return a;
	}
	else if (x >b){
		return b;
	}
	else
		return x;

}
KDL::Vector toolbox::saturate_vec(KDL::Vector x, double a){

	double l = vec_norm(x);
	KDL::Vector xn = x;
	normalizeSafe(xn, KDL::Vector(0.0,0.0,0.0));

	if (l <= a) return x;
	else return a * xn;

}

double toolbox::vec_norm(KDL::Vector x) {
	return sqrt( x[0]*x[0] + x[1]*x[1] + x[2]*x[2] );
}


