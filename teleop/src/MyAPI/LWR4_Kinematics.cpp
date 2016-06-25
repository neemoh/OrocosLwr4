#include "LWR4_Kinematics.hpp"

using namespace std;


//------------------------------------------------------------------------------
// Constructor hard-coded for LWR4 DH
//------------------------------------------------------------------------------
LWR4Kinematics::LWR4Kinematics(double _ztool){

	ztool = _ztool;
	jlim1 = 2.97; // rads = 170 degrees for joints: 1 3 5 7
	jlim2 = 2.09; // rads = 120 degrees for joints: 2 4 6

	config_fk = 0;
	psi_fk = 0.0;

	dbs=0.31;
	dse=0.4;
	dew=0.39;
	dwt=0.078+ztool;

//	q_ik = std::vector<double>(7, 0.0);

	vbs = tf::Vector3(0.0,0.0,dbs);
	vwt = tf::Vector3(0.0,0.0,dwt);

};



//------------------------------------------------------------------------------
// 					LWR4Kinematics METHODS
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// PUBLIC IK METHOD
//------------------------------------------------------------------------------
bool LWR4Kinematics::ik(const KDL::Frame _T, const unsigned int _config, double _psi,
		std::vector<double>& _theta){

	// save the inputs
	T = _T;

	if(this->ikPart1(_theta)){

		// do the second part of IK
		this->ikPart2(_config, _psi, _theta);

//		// write the output
//		_theta = q_ik;

		return true;
	}
	else{
		// something went wrong
		_theta = std::vector<double>(7, 0.0);
		return false;
	}
}



//------------------------------------------------------------------------------
// IK PART1
//------------------------------------------------------------------------------
bool LWR4Kinematics::ikPart1(std::vector<double>& q){

	tf::Vector3 	X07d( tf::Vector3(T.p.data[0],T.p.data[1],T.p.data[2]) );

	tf::Matrix3x3 	R07d( tf::Matrix3x3(T.M.data[0],T.M.data[1],T.M.data[2],
			T.M.data[3],T.M.data[4],T.M.data[5],
			T.M.data[6],T.M.data[7],T.M.data[8]) );

	tf::Vector3 	X0sw( tf::Vector3(0.0,0.0,0.0)) ;

	// vector from shoulder to wrist in base frame
	X0sw = X07d - vbs - R07d*vwt;

	double X0sw_norm = X0sw.length();

	// check for the feasability of the desired Cartesian pose
	if ((X0sw_norm > (dse+dew)) || (X0sw_norm < fabs(dse-dew))) {

		std::cout<< "(LWR4_Kinematics::IKPart1) Warning: Target too far or too close" << std::endl;
		return false;
	}
	else {

		// Theta 4 is constant
		q[3]= acos(( (X0sw_norm	*X0sw_norm) - (dse*dse)-(dew*dew) ) / (2*dse*dew));

		// If joint 4 is out of the limit no solution exists
		if (fabs(q[3])>jlim2) {
			std::cout<< "(LWR4_Kinematics::IKPart1) Warning: No feasible solution found. Joint 4 is out of limit." << std::endl;
			return false;
		}
		else{

			double sin4 = sin(q[3]);
			double cos4 = cos(q[3]);

			// Find the reference for the arm angle
			double th1o  = atan2(X0sw[1] ,X0sw[0]);
			double sin1o = sin(th1o);
			double cos1o = cos(th1o);

			tf::Matrix3x3 R34(tf::Matrix3x3(cos4, 0.0,  sin4, sin4, 0.0, -cos4,  0.0, 1.0,   0.0));

			double left[2] = {(X0sw[0]*cos1o) + (X0sw[1]*sin1o), 	X0sw[2]};
			double right[2] ={dew*sin4, dse + dew*cos4};

			double cos2o = (left[0]*right[0] + left[1]*right[1]) / (right[0]*right[0] + right[1]*right[1]);
			double sin2o = (left[1]-cos2o*right[1])/right[0];

			tf::Matrix3x3  R03o(tf::Matrix3x3(cos1o*cos2o, cos1o*sin2o, -sin1o, cos2o*sin1o, sin1o*sin2o,  cos1o, sin2o,  -cos2o,  0.0));

			// Calculate rotation matrices
			tf::Vector3 U0sw = X0sw.normalized();

			tf::Matrix3x3 U0swX(tf::Matrix3x3(0, -U0sw[2], U0sw[1], U0sw[2], 0, - U0sw[0], -U0sw[1], U0sw[0], 0) );

			tf::Matrix3x3 U0sw2(tf::Matrix3x3(U0sw[0]*U0sw[0], U0sw[0]*U0sw[1], U0sw[0]*U0sw[2],
					U0sw[0]*U0sw[1], U0sw[1]*U0sw[1], U0sw[1]*U0sw[2],
					U0sw[0]*U0sw[2], U0sw[1]*U0sw[2], U0sw[2]*U0sw[2]) );
			tf::Matrix3x3 negate(tf::Matrix3x3(-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0));

			As =   U0swX*R03o;
			Bs =   negate * U0swX*U0swX*R03o;
			Cs =   U0sw2*R03o;

			Aw = (R34.transpose())*(As.transpose())*R07d;
			Bw  = (R34.transpose())*(Bs.transpose())*R07d;
			Cw  = (R34.transpose())*(Cs.transpose())*R07d;
			return true;
		}
	}
}



//------------------------------------------------------------------------------
// IK PART 2
//------------------------------------------------------------------------------
void LWR4Kinematics::ikPart2(const unsigned int config, const double psi,
		std::vector<double>& q){

		double sin_psi = sin(psi);
		double cos_psi = cos(psi);

		// calculate the joint positions based on the reference matrices calculated in
		// part 1 and the arm angle psi
		q[0]= atan2(  As[1][1]*sin_psi + Bs[1][1]*cos_psi + Cs[1][1],
				As[0][1]*sin_psi + Bs[0][1]*cos_psi + Cs[0][1]);

		q[2]= atan2( -As[2][2]*sin_psi - Bs[2][2]*cos_psi - Cs[2][2],
				As[2][0]*sin_psi + Bs[2][0]*cos_psi + Cs[2][0]);
		q[4]= atan2( -Aw[1][2]*sin_psi - Bw[1][2]*cos_psi - Cw[1][2],
				-Aw[0][2]*sin_psi - Bw[0][2]*cos_psi - Cw[0][2]);
		q[6]= atan2( -Aw[2][1]*sin_psi - Bw[2][1]*cos_psi - Cw[2][1],
				Aw[2][0]*sin_psi + Bw[2][0]*cos_psi + Cw[2][0]);

		q[1]= acos(-As[2][1]*sin_psi-Bs[2][1]*cos_psi-Cs[2][1]);
		q[5]= acos( Aw[2][2]*sin_psi+Bw[2][2]*cos_psi+Cw[2][2]);

		// Invert joints according to the selected config parameter
		if ((config & 1) > 0) {
			q[1] = -q[1];
			q[0] = q[0] + M_PI;
			q[2] = q[2] + M_PI;
		}
		if ((config & 2) > 0) {
			q[3] = -q[3];
			q[2] = q[2] + M_PI;
			q[4] = q[4] + M_PI;
		}
		if ((config & 4) > 0) {
			q[5] = -q[5];
			q[4] = q[4] + M_PI;
			q[6] = q[6] + M_PI;
		}

		// mod joint angles so they are all between -pi and pi
		unsigned int atan_joints[4] = {0, 2, 3, 5};

		for (unsigned int iter = 0; iter < 4; iter++ ) {
			modAngle(q[iter*2]);
		}
		for (unsigned int iter = 0; iter < 7; iter++) {
			q[iter] *= (fabs(q[iter]) > 0.000001);
		}

}



//------------------------------------------------------------------------------
// validJointsForCurrentArc
//------------------------------------------------------------------------------
bool LWR4Kinematics::validJointsForCurrentArc(const KDL::Frame T, const unsigned int _config, double psi_curr,
		std::vector<double> &psi_arc){

	vector<double> q(7,0.0);

	bool valid = true;

	double step = M_PI/180;
	double psi_lim = M_PI;

	double psi_n = psi_curr-step;
	double psi_p = psi_curr;

	// the loop to find the invalid arm angles

	if(ikPart1(q)){

		// increment search
		while(valid) {
			ikPart2(_config, psi_p, q);

			valid = (fabs(q[0]) <jlim1 && fabs(q[2])<jlim1 && fabs(q[4])<jlim1	&& fabs(q[6])<jlim1
					&& fabs(q[1])<jlim2 && fabs(q[3])<jlim2);

			if(valid){
				psi_p = psi_p + step;
			}
			else
				psi_p = psi_p - step;

			if (psi_p> psi_lim){
				valid = false;
				psi_p = psi_lim;
			}
		}
		valid = true;

		// decrement search
		while(valid) {
			ikPart2(_config, psi_n, q);

			valid = (fabs(q[0]) <jlim1 && fabs(q[2])<jlim1 && fabs(q[4])<jlim1 && fabs(q[6])<jlim1
					&& fabs(q[1])<jlim2 && fabs(q[5])<jlim2);

			if(valid)
				psi_n = psi_n - step;
			else
				psi_n = psi_n + step;

			if (psi_n < -psi_lim){
				valid = false;
				psi_n = -psi_lim;
			}
		}
		//	cout << "psi_p" << psi_p << endl;
		//	cout << "psi_n" << psi_n << endl;

		psi_arc[0] = psi_n;
		psi_arc[1] = psi_p;

		return true;
	}
	else
		return false;

}



//------------------------------------------------------------------------------
// validJointsForPsiVector
//------------------------------------------------------------------------------
bool LWR4Kinematics::validJointsForPsiVector(const KDL::Frame T, const unsigned int _config,
		const std::vector<double> _psi,	std::vector<std::vector<double> >& valid_joints,
		std::vector<double> & valid_psis) {

	size_t psi_size = _psi.size();

	vector<double> q(7,0.0);

	if(ikPart1(q)){

		// find the joint positions for all _psi values and check their validity
		for(size_t i=0; i<psi_size; i++ ) {

			ikPart2(_config, _psi.at(i), q);

			// check we're within joint limits or not
			if (fabs(q[0]) <jlim1 && fabs(q[2])<jlim1 && fabs(q[4])<jlim1	&& fabs(q[6])<jlim1
					&& fabs(q[1])<jlim2 && fabs(q[5])<jlim2){

				valid_psis.push_back(_psi.at(i));
				valid_joints.push_back(q);
			}
		}
		return true;
	}
	else
		return false;

	///				TOTAL TIME FOR 360 ANGLES MEASURED FROM OUTSIDE ~= 100 uS
}



//------------------------------------------------------------------------------
// FK
//------------------------------------------------------------------------------
void LWR4Kinematics::fk(const std::vector<double> joint_positions, unsigned int & config, double & nsparam,  KDL::Frame &cartesian_matrix) {

	// I am copying the implementation of Mirko Kunze here which is written in a generic
	// way. It can become faster by simplifying it for our DH parameters.

	std::vector<std::vector<double> > dh;
	unsigned int njoints = 7;

	dh.clear();
	std::vector<double> singlelink;
	singlelink.push_back(joint_positions.at(0));
	singlelink.push_back(dbs);
	singlelink.push_back(0.0);
	singlelink.push_back(M_PI / 2);
	dh.push_back(singlelink);
	singlelink.clear();
	singlelink.push_back(joint_positions.at(1));
	singlelink.push_back(0.0);
	singlelink.push_back(0.0);
	singlelink.push_back(-M_PI / 2);
	dh.push_back(singlelink);
	singlelink.clear();
	singlelink.push_back(joint_positions.at(2));
	singlelink.push_back(dse);
	singlelink.push_back(0.0);
	singlelink.push_back(-M_PI / 2);
	dh.push_back(singlelink);
	singlelink.clear();
	singlelink.push_back(joint_positions.at(3));
	singlelink.push_back(0.0);
	singlelink.push_back(0.0);
	singlelink.push_back(M_PI / 2);
	dh.push_back(singlelink);
	singlelink.clear();
	singlelink.push_back(joint_positions.at(4));
	singlelink.push_back(dew);
	singlelink.push_back(0.0);
	singlelink.push_back(M_PI / 2);
	dh.push_back(singlelink);
	singlelink.clear();
	singlelink.push_back(joint_positions.at(5));
	singlelink.push_back(0.0);
	singlelink.push_back(0.0);
	singlelink.push_back(-M_PI / 2);
	dh.push_back(singlelink);
	singlelink.clear();
	singlelink.push_back(joint_positions.at(6));
	singlelink.push_back(dwt);
	singlelink.push_back(0.0);
	singlelink.push_back(0);
	dh.push_back(singlelink);
	singlelink.clear();

	config_fk = ((int) (joint_positions.at(1) < 0)) + (2 * ((int) (joint_positions.at(3) < 0))) + (4 * ((int) (joint_positions.at(5) < 0)));
	config = config_fk;

	std::vector<KDL::Frame> linkMatrices(njoints + 1, KDL::Frame::Identity());

	//joint_positions = deg2rad(joint_positions);

	for (unsigned int linkIter = 1; linkIter < (njoints + 1); linkIter++) {
		KDL::Vector linkTmpVec = KDL::Vector::Zero();
		KDL::Rotation linkTmpRot = KDL::Rotation::Identity();
		linkTmpVec = KDL::Vector(dh.at(linkIter - 1).at(2) * cos(dh.at(linkIter - 1).at(0)), dh.at(linkIter - 1).at(2) * sin(dh.at(linkIter - 1).at(0)), dh.at(linkIter - 1).at(1));
		linkTmpRot
		= KDL::Rotation(cos(dh.at(linkIter - 1).at(0)), -sin(dh.at(linkIter - 1).at(0)) * cos(dh.at(linkIter - 1).at(3)), sin(dh.at(linkIter - 1).at(0)) * sin(dh.at(linkIter - 1).at(3)), sin(dh.at(linkIter - 1).at(0)), cos(dh.at(linkIter
				- 1).at(0)) * cos(dh.at(linkIter - 1).at(3)), -cos(dh.at(linkIter - 1).at(0)) * sin(dh.at(linkIter - 1).at(3)), 0.0, sin(dh.at(linkIter - 1).at(3)), cos(dh.at(linkIter - 1).at(3)));
		linkMatrices.at(linkIter) = KDL::Frame(linkTmpRot, linkTmpVec);
		linkMatrices.at(linkIter) = linkMatrices.at(linkIter - 1) * linkMatrices.at(linkIter);
	}
	KDL::Vector xs = linkMatrices.at(1).p;
	KDL::Vector xe = linkMatrices.at(3).p;
	KDL::Vector xw = linkMatrices.at(5).p;


	KDL::Vector xsw = xw - xs;
	KDL::Vector xse = xe - xs;

	KDL::Vector usin = KDL::Vector(0.0, 0.0, 1.0) * xsw;
	usin.Normalize();
	KDL::Vector ucos = xsw * usin;
	ucos.Normalize();

	// Nima: added negative sign so that the rotation of nsparam is around the vector from soulder to wrist.
	nsparam = -((double) atan2(dot(xse, usin), dot(xse, ucos)));

	cartesian_matrix = linkMatrices.at(njoints); //First matrix is identity

	///				TOTAL TIME MEASURED FROM OUTSIDE ~= 9 uS

}


//------------------------------------------------------------------------------
// MOD ANGLE
//------------------------------------------------------------------------------
void LWR4Kinematics::modAngle(double &angle){

	while (angle > M_PI) {
		angle -= (2.0 * M_PI);
	}
	while (angle< -M_PI) {
		angle += (2.0 * M_PI);
	}

}




