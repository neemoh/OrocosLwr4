#include "LWR4_Kinematics.hpp"

using namespace std;


///-------------------------------------------------------------------------------------------------------------------------------
bool LWR4_Kinematics::IK(const KDL::Frame T, const unsigned int config, double psi, double ztool,
		std::vector<double>& theta) {
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	// Inverse Kinematic function of a 7DOF anthropomorphic robot arm.
	//
	// T:  Desired Cartesian pose
	// Config: 		  8 possible solutions may be available givven T and psi. config is
	//         		  an int8 (0 to 7) that describes which one of these possible
	//        		  solutions is wanted.
	// psi:   		  The redundancy parameter (AKA arm angle) (rad)
	// ztool: 		  (Optional) In case a tool is attached to the robot that is at
	//                [0 0 ztool] in the 7th joint reference frame, you can use ztool to
	//                directly find the Cartesian position of the tool tip. For any different
	//                tool setup just use the corresponding transformation.
	// theta:		  Calculated joint positions.
	//
	// This function is written based the 2008 paper by Shimizu, M. et al.
	// The config parameter is not discussed in that paper. I got the idea from
	// Mirko Kunze. Everything else is clearly explained in the paper.
	// The dh parameters used here is based on KUKA LWR4's parameters that
	// differes with that of Shimizu in alpha:
	//         'kuka':    alpha = [pi/2 -pi/2 -pi/2 pi/2 pi/2 -pi/2 0]
	//         'shimizu': alpha = [-pi/2 pi/2 -pi/2 pi/2 -pi/2 pi/2 0]
	// The code is written for performance and it is not that readable! Check the
	// Matlab codes if you need to modify something.
	//
	// Nima Enayati, Nearlab, 03/2016.
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	double dbs=0.31, dse=0.4, dew=0.39, dwt=0.078+ztool;
	tf::Vector3 vbs(tf::Vector3(0.0,0.0,dbs));
	tf::Vector3 vwt(tf::Vector3(0.0,0.0,dwt));
	tf::Vector3 X07d(tf::Vector3(T.p.data[0],T.p.data[1],T.p.data[2])	);
	tf::Matrix3x3 R07d(tf::Matrix3x3(T.M.data[0],T.M.data[1],T.M.data[2],
									 T.M.data[3],T.M.data[4],T.M.data[5],
									 T.M.data[6],T.M.data[7],T.M.data[8]));
	tf::Vector3 X0sw(tf::Vector3(0.0,0.0,0.0));

	X0sw = X07d - vbs - R07d*vwt;
	double X0sw_norm = X0sw.length();

	if ((X0sw_norm > (dse+dew)) || (X0sw_norm < fabs(dse-dew))) {
		std::cout<< "(LWR4_Kinematics::IK) Warning: Target too far or too close" << std::endl;
		return false;
	}
	else {

		// Theta 4 is constant
		theta.at(3)= acos(( (X0sw_norm	*X0sw_norm) - (dse*dse)-(dew*dew) ) / (2*dse*dew));

		double sin4 = sin(theta.at(3));
		double cos4 = cos(theta.at(3));

		// Find the reference for the arm angle
		double th1o=atan2(X0sw[1] ,X0sw[0]);
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

		tf::Matrix3x3 As =   U0swX*R03o;
		tf::Matrix3x3 Bs =   negate * U0swX*U0swX*R03o;
		tf::Matrix3x3 Cs =   U0sw2*R03o;

		tf::Matrix3x3 Aw = (R34.transpose())*(As.transpose())*R07d;
		tf::Matrix3x3 Bw  = (R34.transpose())*(Bs.transpose())*R07d;
		tf::Matrix3x3 Cw  = (R34.transpose())*(Cs.transpose())*R07d;


		double sin_psi = sin(psi);
		double cos_psi = cos(psi);

		// calucalte the joint angles for the fiven psi
		theta.at(0)= atan2(  As[1][1]*sin_psi + Bs[1][1]*cos_psi + Cs[1][1],
				As[0][1]*sin_psi + Bs[0][1]*cos_psi + Cs[0][1]);

		theta.at(2)= atan2( -As[2][2]*sin_psi - Bs[2][2]*cos_psi - Cs[2][2],
				As[2][0]*sin_psi + Bs[2][0]*cos_psi + Cs[2][0]);
		theta.at(4)= atan2( -Aw[1][2]*sin_psi - Bw[1][2]*cos_psi - Cw[1][2],
				-Aw[0][2]*sin_psi - Bw[0][2]*cos_psi - Cw[0][2]);
		theta.at(6)= atan2( -Aw[2][1]*sin_psi - Bw[2][1]*cos_psi - Cw[2][1],
				Aw[2][0]*sin_psi + Bw[2][0]*cos_psi + Cw[2][0]);


		theta.at(1)= acos(-As[2][1]*sin_psi-Bs[2][1]*cos_psi-Cs[2][1]);
		theta.at(5)= acos( Aw[2][2]*sin_psi+Bw[2][2]*cos_psi+Cw[2][2]);


		//		// invert joints according to the selected config
		if ((config & 1) > 0) {
			theta.at(1) = -theta.at(1);
			theta.at(0) = theta.at(0) + M_PI;
			theta.at(2) = theta.at(2) + M_PI;
		}
		if ((config & 2) > 0) {
			theta.at(3) = -theta.at(3);
			theta.at(2) = theta.at(2) + M_PI;
			theta.at(4) = theta.at(4) + M_PI;
		}
		if ((config & 4) > 0) {
			theta.at(5) = -theta.at(5);
			theta.at(4) = theta.at(4) + M_PI;
			theta.at(6) = theta.at(6) + M_PI;
		}

		// mod angles so they are all between - pi and pi
		for (unsigned int iter = 0; iter < 7; iter += 2) {
//			while (theta.at(iter) > M_PI) {
//				theta.at(iter) -= (2.0 * M_PI);
//			}
//			while (theta.at(iter) < -M_PI) {
//				theta.at(iter) += (2.0 * M_PI);
//			}
			LWR4_Kinematics::modAngle(theta.at(iter));

		}
		for (unsigned int iter = 0; iter < 7; iter++) {
			theta.at(iter) *= (fabs(theta.at(iter)) > 0.00001);
		}

		return true;
		///				TOTAL TIME MEASURED FROM OUTSIDE ~= 0.7 uS

	}
}



//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool LWR4_Kinematics::FK(const std::vector<double> joint_positions, unsigned int & config, double & nsparam, double  ztool, KDL::Frame &cartesian_matrix) {

	std::vector<std::vector<double> > dh;
	unsigned int njoints = 7;
	double links[4];

	links[0]				= 0.31; // length from robot base to shoulder
	links[1] 				= 0.4; // length from shoulder to elbow
	links[2] 				= 0.39; // length from elbow to wrist
	links[3] 				= 0.078 + ztool; // length from wrist to flange + tool length(position of target)

	dh.clear();
	std::vector<double> singlelink;
	singlelink.push_back(joint_positions.at(0));
	singlelink.push_back(links[0]);
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
	singlelink.push_back(links[1]);
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
	singlelink.push_back(links[2]);
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
	singlelink.push_back(links[3]);
	singlelink.push_back(0.0);
	singlelink.push_back(0);
	dh.push_back(singlelink);
	singlelink.clear();

	config = ((int) (joint_positions.at(1) < 0)) + (2 * ((int) (joint_positions.at(3) < 0))) + (4 * ((int) (joint_positions.at(5) < 0)));
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

//	KDL::Vector xew = xw - xe; // Elbow to wrist
//	KDL::Vector xes = xs - xe; // Elbow to shoulder
//	elbow_tangent = xes * xew; // Tangent vector
//	elbow_tangent.Normalize();


	//std::cout << "Sizes " << linkMatrices.size() << " " << this->j_num << std::endl;
	//targetmatrix = linkMatrices.back();
	cartesian_matrix = linkMatrices.at(njoints); //First matrix is identity

	return true;
	///				TOTAL TIME MEASURED FROM OUTSIDE ~= 9 uS


}



bool LWR4_Kinematics::redundancyCircle(const KDL::Frame T, const unsigned int config, const std::vector<double> psi, double ztool,double& joint_four, std::vector<std::vector<double> >& six_joints) {
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	// This function performs inverse kinematics for a given T and a vector of psi
	// values.
	//
	// T:  			  Desired Cartesian pose 4x4
	// Config: 		  8 possible solutions may be available givven T and psi. config is
	//         		  an int8 (0 to 7) that describes which one of these possible
	//        		  solutions is wanted.
	// psi:   		  The redundancy parameter vector (AKA arm angle) (rad)
	// ztool: 		  (Optional) In case a tool is attached to the robot that is at
	//                [0 0 ztool] in the 7th joint reference frame, you can use ztool to
	//                directly find the Cartesian position of the tool tip. For any different
	//                tool setup just use the corresponding transformation. (m)
	// joint_four:	  For a fixed T, the 4th joint of the robot has a constant value
	//				  for all psi values. Therefore to save memory this joint is given
	//				  separately. (rad)
	// six_joints	  The rest of the joints (1, 2, 3, 5, 6, 7) (rad)
	//				  dimension: sizeof(psi)x6
	//
	// For more information about the IK method refer to the LWR4_Kinematics::IK function.
	//
	// Nima Enayati, Nearlab, 03/2016.
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	six_joints.clear();

//	double l1 = 2.97; //rads = 170 degrees - Joints: 1 3 5 7
//	double l2 = 2.09; //rads = 120 degrees - Joints: 2 4 6
	tf::Matrix3x3 As, Bs, Cs,Aw, Bw, Cw;
	vector<double> six_joints_temp(6,0.0);
	size_t psi_size = psi.size();

	LWR4_Kinematics::IKPart1(T,  ztool, joint_four,As, Bs, Cs,Aw, Bw, Cw);

	for(size_t i=0; i<psi_size; i++ ) {
		////		INTERNAL FOR LOOP TIMING FROM HERE
		LWR4_Kinematics::IKPart2( psi.at(i), config, joint_four,As, Bs, Cs,Aw, Bw, Cw, six_joints_temp);

		six_joints.push_back(six_joints_temp);

	}
	return true;

		///				TOTAL TIME FOR 360 ANGLES MEASURED FROM OUTSIDE ~= 100 uS
}


bool LWR4_Kinematics::validJointsForPsiVector(const KDL::Frame T, const unsigned int config,
		const std::vector<double> psi, double ztool,double& joint_four,
		std::vector<std::vector<double> >& valid_six_joints, std::vector<double> & valid_psis) {
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	// This function performs inverse kinematics for a given T and a vector of psi
	// values.
	//
	// T:  			  Desired Cartesian pose 4x4
	// Config: 		  8 possible solutions may be available givven T and psi. config is
	//         		  an int8 (0 to 7) that describes which one of these possible
	//        		  solutions is wanted.
	// psi:   		  The redundancy parameter vector (AKA arm angle) (rad)
	// ztool: 		  (Optional) In case a tool is attached to the robot that is at
	//                [0 0 ztool] in the 7th joint reference frame, you can use ztool to
	//                directly find the Cartesian position of the tool tip. For any different
	//                tool setup just use the corresponding transformation. (m)
	// joint_four:	  For a fixed T, the 4th joint of the robot has a constant value
	//				  for all psi values. Therefore to save memory this joint is given
	//				  separately. (rad)
	// six_joints	  The rest of the joints (1, 2, 3, 5, 6, 7) (rad)
	//				  dimension: sizeof(psi)x6
	//
	// For more information about the IK method refer to the LWR4_Kinematics::IK function.
	//
	// Nima Enayati, Nearlab, 03/2016.
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	double l1 = 2.97; //rads = 170 degrees - Joints: 1 3 5 7
	double l2 = 2.09; //rads = 120 degrees - Joints: 2 4 6
	tf::Matrix3x3 As, Bs, Cs,Aw, Bw, Cw;
	vector<double> six_joints(6,0.0);
	size_t psi_size = psi.size();


	LWR4_Kinematics::IKPart1(T,  ztool, joint_four,As, Bs, Cs,Aw, Bw, Cw);

	for(size_t i=0; i<psi_size; i++ ) {
		////		INTERNAL FOR LOOP TIMING FROM HERE
		LWR4_Kinematics::IKPart2( psi.at(i), config, joint_four,As, Bs, Cs,Aw, Bw, Cw, six_joints);

		if (fabs(six_joints[0]) <l1 && fabs(six_joints[2])<l1 && fabs(six_joints[3])<l1	&& fabs(six_joints[5])<l1
				&& fabs(six_joints[1])<l2 && fabs(six_joints[4])<l2){
			valid_psis.push_back(psi.at(i));
			valid_six_joints.push_back(six_joints);
		}

	}

	return true;	///				TOTAL TIME FOR 360 ANGLES MEASURED FROM OUTSIDE ~= 100 uS
}

bool LWR4_Kinematics::validJointsForCurrentArc(const KDL::Frame T, const unsigned int config, double psi_curr,
		double ztool,std::vector<double> &psi_arc){
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	// This function performs inverse kinematics for a given T and a vector of psi
	// values.
	//
	// T:  			  Desired Cartesian pose 4x4
	// Config: 		  8 possible solutions may be available givven T and psi. config is
	//         		  an int8 (0 to 7) that describes which one of these possible
	//        		  solutions is wanted.
	// psi:   		  The redundancy parameter vector (AKA arm angle) (rad)
	// ztool: 		  (Optional) In case a tool is attached to the robot that is at
	//                [0 0 ztool] in the 7th joint reference frame, you can use ztool to
	//                directly find the Cartesian position of the tool tip. For any different
	//                tool setup just use the corresponding transformation. (m)
	// joint_four:	  For a fixed T, the 4th joint of the robot has a constant value
	//				  for all psi values. Therefore to save memory this joint is given
	//				  separately. (rad)
	// six_joints	  The rest of the joints (1, 2, 3, 5, 6, 7) (rad)
	//				  dimension: sizeof(psi)x6
	//
	// For more information about the IK method refer to the LWR4_Kinematics::IK function.
	//
	// Nima Enayati, Nearlab, 03/2016.
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


	double joint_four;
	double step = M_PI/180;
	double psi_lim = M_PI;
	double l1 = 2.97; //rads = 170 degrees - Joints: 1 3 5 7
	double l2 = 2.09; //rads = 120 degrees - Joints: 2 4 6
	bool valid = true;
	tf::Matrix3x3 As, Bs, Cs,Aw, Bw, Cw;
	vector<double> six_joints(6,0.0);
	double psi_n = psi_curr-step;
	double psi_p = psi_curr;


	LWR4_Kinematics::IKPart1(T,  ztool, joint_four,As, Bs, Cs,Aw, Bw, Cw);


	while(valid) {
		////		INTERNAL FOR LOOP TIMING FROM HERE
		LWR4_Kinematics::IKPart2( psi_p, config, joint_four,As, Bs, Cs,Aw, Bw, Cw, six_joints);

		valid = (fabs(six_joints[0]) <l1 && fabs(six_joints[2])<l1 && fabs(six_joints[3])<l1	&& fabs(six_joints[5])<l1
				&& fabs(six_joints[1])<l2 && fabs(six_joints[4])<l2);

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

	while(valid) {
		////		INTERNAL FOR LOOP TIMING FROM HERE
		LWR4_Kinematics::IKPart2( psi_n, config, joint_four,As, Bs, Cs,Aw, Bw, Cw, six_joints);

		valid = (fabs(six_joints[0]) <l1 && fabs(six_joints[2])<l1 && fabs(six_joints[3])<l1 && fabs(six_joints[5])<l1
				&& fabs(six_joints[1])<l2 && fabs(six_joints[4])<l2);

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

	///				TOTAL TIME FOR 360 ANGLES MEASURED FROM OUTSIDE ~= 100 uS
}


bool LWR4_Kinematics::IKPart1(const KDL::Frame T, const double ztool,double& joint_four,
		tf::Matrix3x3& As, tf::Matrix3x3& Bs, tf::Matrix3x3& Cs, tf::Matrix3x3& Aw,
		tf::Matrix3x3& Bw,tf::Matrix3x3& Cw){

	double dbs=0.31, dse=0.4, dew=0.39, dwt=0.078+ztool;
	double l2 = 2.09; //rads = 120 degrees - Joints: 2 4 6


	tf::Vector3 vbs(tf::Vector3(0.0,0.0,dbs));
	tf::Vector3 vwt(tf::Vector3(0.0,0.0,dwt));
	tf::Vector3 X07d(tf::Vector3(T.p.data[0],T.p.data[1],T.p.data[2])	);
	tf::Matrix3x3 R07d(tf::Matrix3x3(T.M.data[0],T.M.data[1],T.M.data[2],
			T.M.data[3],T.M.data[4],T.M.data[5],
			T.M.data[6],T.M.data[7],T.M.data[8]));
	tf::Vector3 X0sw(tf::Vector3(0.0,0.0,0.0));

	X0sw = X07d - vbs - R07d*vwt;
	double X0sw_norm = X0sw.length();

	if ((X0sw_norm > (dse+dew)) || (X0sw_norm < fabs(dse-dew))) {
		std::cout<< "(LWR4_Kinematics::IKPart1) Warning: Target too far or too close" << std::endl;
		return false;
	}
	else {

		// Theta 4 is constant
		joint_four= acos(( (X0sw_norm	*X0sw_norm) - (dse*dse)-(dew*dew) ) / (2*dse*dew));

		// If joint 4 is out of the limit no solution exists
		if (fabs(joint_four)>l2) {
			std::cout<< "(LWR4_Kinematics::IKPart1) Warning: No feasible solution found. Joint 4 is out of limit." << std::endl;
			return false;
		}
		else{

			double sin4 = sin(joint_four);
			double cos4 = cos(joint_four);

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

void LWR4_Kinematics::IKPart2(const double psi, const unsigned int config, double& joint_four,
		tf::Matrix3x3& As, tf::Matrix3x3& Bs, tf::Matrix3x3& Cs, tf::Matrix3x3& Aw,
		tf::Matrix3x3& Bw,tf::Matrix3x3& Cw, std::vector<double>& six_joints){

		double sin_psi = sin(psi);
		double cos_psi = cos(psi);

		double j_local[6];

		j_local[0]= atan2(  As[1][1]*sin_psi + Bs[1][1]*cos_psi + Cs[1][1],
				As[0][1]*sin_psi + Bs[0][1]*cos_psi + Cs[0][1]);

		j_local[2]= atan2( -As[2][2]*sin_psi - Bs[2][2]*cos_psi - Cs[2][2],
				As[2][0]*sin_psi + Bs[2][0]*cos_psi + Cs[2][0]);
		j_local[3]= atan2( -Aw[1][2]*sin_psi - Bw[1][2]*cos_psi - Cw[1][2],
				-Aw[0][2]*sin_psi - Bw[0][2]*cos_psi - Cw[0][2]);
		j_local[5]= atan2( -Aw[2][1]*sin_psi - Bw[2][1]*cos_psi - Cw[2][1],
				Aw[2][0]*sin_psi + Bw[2][0]*cos_psi + Cw[2][0]);

		j_local[1]= acos(-As[2][1]*sin_psi-Bs[2][1]*cos_psi-Cs[2][1]);
		j_local[4]= acos( Aw[2][2]*sin_psi+Bw[2][2]*cos_psi+Cw[2][2]);


		//		// invert joints according to the selected config
		if ((config & 1) > 0) {
			j_local[1] = -j_local[1];
			j_local[0] = j_local[0] + M_PI;
			j_local[2] = j_local[2] + M_PI;
		}
		if ((config & 2) > 0) {
			joint_four = -joint_four;
			j_local[2] = j_local[2] + M_PI;
			j_local[3] = j_local[3] + M_PI;
		}
		if ((config & 4) > 0) {
			j_local[4] = -j_local[4];
			j_local[3] = j_local[3] + M_PI;
			j_local[5] = j_local[5] + M_PI;
		}

		// mod joint angles so they are all between -pi and pi
		unsigned int atan_joints[4] = {0, 2, 3, 5};

		for (unsigned int iter = 0; iter < 4; iter++ ) {
			LWR4_Kinematics::modAngle(j_local[atan_joints[iter]]);
		}
		for (unsigned int iter = 0; iter < 6; iter++) {
			j_local[iter] *= (fabs(j_local[iter]) > 0.00001);
			six_joints[iter] = j_local[iter];
		}


}
void LWR4_Kinematics::modAngle(double &angle){

	while (angle > M_PI) {
		angle -= (2.0 * M_PI);
	}
	while (angle< -M_PI) {
		angle += (2.0 * M_PI);
	}

}

