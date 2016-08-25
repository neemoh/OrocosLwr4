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

#include "acMethods.hpp"

using namespace toolbox;


//-----------------------------------------------------------------------
// SIMULATED PLASTICY INTRODUCED BY KIKUUWE ET AL. 2008
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
// Constructor
//-----------------------------------------------------------------------
acPlast::acPlast(double F_MAX_in, double ELASTIC_LENGTH_in, double BOUNDARY_THRESHOLD_in){
	F_MAX 				= F_MAX_in;
	ELASTIC_LENGTH 		= ELASTIC_LENGTH_in;
	BOUNDARY_THRESHOLD 	= BOUNDARY_THRESHOLD_in;
	p_tool_last 		= KDL::Vector(0.0,0.0,0.0);
	q					= KDL::Vector(0.0,0.0,0.0);

}

//-----------------------------------------------------------------------
// FORCE GENERATION FOLLOWING THE EQUATIONS DESCRIBED IN THE
// CORRESPONDING PAPER
//-----------------------------------------------------------------------

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


//-----------------------------------------------------------------------
// PLASTIC WITH MOTION REDIRECTION - INTRODUCED BY BOWYER ET AL. 2013
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
// Constructor
//-----------------------------------------------------------------------
acPlastRedirect::acPlastRedirect(double F_MAX_in, double ELASTIC_LENGTH_in, double BOUNDARY_THRESHOLD_in){

	F_MAX = F_MAX_in;
	ELASTIC_LENGTH = ELASTIC_LENGTH_in;
	BOUNDARY_THRESHOLD = BOUNDARY_THRESHOLD_in;
	p_tool_last 		= KDL::Vector(0.0,0.0,0.0);
	z					= KDL::Vector(0.0,0.0,0.0);

}

//-----------------------------------------------------------------------
// FORCE GENERATION FOLLOWING THE EQUATIONS DESCRIBED IN THE
// CORRESPONDING PAPER
//-----------------------------------------------------------------------
void acPlastRedirect::getForce(KDL::Vector &f_out,  const KDL::Vector p_tool, const KDL::Vector p_desired, const KDL::Vector v_msrd){

	double sig2 = 2.5;
	double fc = F_MAX;
	double theta = 0.4;
	double sig0 = fc/ELASTIC_LENGTH;
	double zcss = fc/sig0;
	KDL::Vector penet = p_tool - p_desired ;

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



//-----------------------------------------------------------------------
// VISCOUSE WITH REDIRECTION - INTRODUCED BY ENAYATI ET AL. 2016
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
// Constructor
//-----------------------------------------------------------------------
acViscousRedirect::acViscousRedirect(double F_MAX_in, double B_MAX_in, double BOUNDARY_THRESHOLD_in){

	F_MAX = F_MAX_in;
	B_MAX = B_MAX_in;
	BOUNDARY_THRESHOLD = BOUNDARY_THRESHOLD_in;
	v_dir_last = KDL::Vector(1.0,0.0,0.0);
	f_dir_last = KDL::Vector(1.0,0.0,0.0);
	n_2_last   = KDL::Vector(1.0,0.0,0.0);

}

//-----------------------------------------------------------------------
// FORCE GENERATION FOLLOWING THE EQUATIONS DESCRIBED IN THE
// CORRESPONDING PAPER
//-----------------------------------------------------------------------

void acViscousRedirect::getForce(KDL::Vector &f_out,  const KDL::Vector p_tool, const KDL::Vector p_desired, const KDL::Vector v_msrd){

	// UPPERCASE NAMES = SCALARS
	// LOWERCASE NAMES = VECTORS

	KDL::Vector penet = p_desired - p_tool;
	double F_VC, F_VC_SAT;
	double B_M			= B_MAX;

	KDL::Vector  f_dir, penet_dir, v_tool_dir;

	penet_dir 		= penet;
	v_tool_dir 		= v_msrd;

	normalizeSafe(v_tool_dir, v_dir_last);
	normalizeSafe(penet_dir, KDL::Vector(1.0,0.0,0.0));

	double V_PENET_DOTP = dot(v_tool_dir, penet_dir);
	double PENET =vec_norm(penet);

	// limit the viscous coefficient around the boundary to minimize the oscillation
	if (PENET< BOUNDARY_THRESHOLD){
		B_M =  B_MAX * PENET/ BOUNDARY_THRESHOLD;
	}
	else
		B_M = B_MAX;

	// calculate the magnitude of the ac force
	F_VC = B_M * sqrt( ( 1 - V_PENET_DOTP ) / 2 ) * v_msrd.Norm();

	// saturate the magnitude of the ac force
	F_VC_SAT = saturate(0.0, F_VC , F_MAX );

	//-----------------------------------------------------------------------
	// calculate the direction of the AC force
	KDL::Vector n = v_tool_dir * penet_dir;
	KDL::Vector nn = n;
	normalizeSafe(nn, penet_dir);

	double THETA = (M_PI/2) * (1 + V_PENET_DOTP);

	if ( V_PENET_DOTP < 0.0 )
		f_dir = penet_dir;
	else
		if(rotateVector(v_tool_dir, f_dir, nn, THETA) < 0)
			std::cout << "Null in rotateVector." <<"  norm(v_tool_dir) = "<< v_tool_dir.Norm()<<
			"  , norm(nn) = "<< nn.Norm() << std::endl;

	//-----------------------------------------------------------------------
	// Make the force vector from the calculated magnitude and direction
	f_out = F_VC_SAT * f_dir;

	v_dir_last = v_tool_dir;

}




//-----------------------------------------------------------------------
// SIMPLE ELASTIC WITH DAMPING
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
// Constructor
//-----------------------------------------------------------------------
acElastic::acElastic(double _F_MAX, double _k, double _b, double _dt){
	F_MAX = _F_MAX;
	k = _k;
	b = _b;
	dt = _dt;
	penet_last = KDL::Vector(0.0, 0.0, 0.0);
	penet_vel = 0;
};


//-----------------------------------------------------------------------
// FORCE GENERATION
//-----------------------------------------------------------------------

void acElastic::getForce(KDL::Vector &f_out,  const KDL::Vector p_tool, const KDL::Vector p_desired, const KDL::Vector v_msrd){

	// find the penetration vector
	KDL::Vector penet = p_desired - p_tool;

	// make the force vector
	KDL::Vector f_all = k * penet - b * v_msrd;
	double f_all_magnitude = toolbox::vec_norm(f_all);

	// limit the force to F_MAX
	if (f_all_magnitude > F_MAX)
		f_out = F_MAX/f_all_magnitude * f_all;
	else
		f_out = f_all;

	// save last penet
	penet_last = penet;

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


