/*
 * interpolator.cpp
 *
 *  Created on: Mar 15, 2016
 *      Author: nearlab
 */

#include <cmath>
#include <iostream>
template <typename T> int sgn(T val);


// FROM ACTIVE PROJECT. I THINK IT'S WRITTEN BY MIRKO KUNZE

bool variableInterpolator(const std::vector<double> xTargetI, std::vector<double> xLastI, std::vector<double> vLastI, std::vector<double>& xNowI, std::vector<double>& vNowI, const std::vector<double> xMinI, const std::vector<
		double> xMaxI, const std::vector<double> vMaxI, const std::vector<double> aMaxI, bool& target_reached, const double dt) {

	// Interpolates a vector of variables: from xLastI with vLastI it calculates the next point xNowI and velocity vNowI towards the target xTargetI to be reached within one time frame time_frame, under the constraint of xMaxI, xMinI, vMaxI, aMaxI
	unsigned int interpolation_mode = 0;

	size_t num_var = xTargetI.size();
	double TARGETTOLERANCE = 0.000001;
	double TOLERANCE = 0.000001;

	std::vector<double> xToGo(num_var, 0.0);
	double sumxToGo = 0.0;
	double sumv = 0.0;

	for (size_t iter = 0; iter < num_var; iter++) {
		xToGo.at(iter) = xTargetI.at(iter) - xLastI.at(iter);
		sumxToGo += fabs(xToGo.at(iter));
		sumv += fabs(vLastI.at(iter));
	}
	if (sumxToGo < TARGETTOLERANCE && sumv < TARGETTOLERANCE) {
		// Target reached!
//		cout << "ON TARGET" << endl;
		target_reached = true;
		xNowI = xLastI;
		vNowI = vLastI;
		return true;
	}


	std::vector<double> a1(num_var, 0.0);
	std::vector<double> a2(num_var, 0.0);
	std::vector<double> T1(num_var, 0.0);
	std::vector<double> T2(num_var, 0.0);
	std::vector<double> T3(num_var, 0.0);
	std::vector<double> TSum(num_var, 0.0);
	std::vector<double> vInt(num_var, 0.0);
	double xJolt = 0.0;
	double tToVMax = 0.0;
	double radic = 0.0;
	double tBrake = 0.0;
	bool flip = false;
	double xAfterT1 = 0.0;
	double xAfterT2 = 0.0;
	double xBrakeEnd = 0.0;
	double TMax = 0.0;
	bool brakeAll = false;
	double a = 0.0;
	double v = 0.0;
	double vNowIMinNoOS = 0.0;
	double vNowIMaxNoOS = 0.0;

	switch (interpolation_mode) {
		case 0:
//			log(RTT::Debug) << "Interpolation mode 0" << endlog();
		case 1:

			/*  Brake to new velocity (if needed) */
//			for (unsigned int iter = 0; iter < num_var; iter++) {
//
//				if ( this->switchToLowerVelocity.at(iter) ) {	// need alignment for this variable
//
//					// can we reach new velocity within dt?
//					if ( fabs( fabs(vLastI.at(iter)) - vMaxI.at(iter) ) <= aMaxI.at(iter) * dt) {
//
//						vNowI.at(iter) = sign(vLastI.at(iter))*vMaxI.at(iter);
//						tBrake = fabs( fabs(vLastI.at(iter)) - vMaxI.at(iter)) / aMaxI.at(iter);
//						xNowI.at(iter) = xLastI.at(iter) + vLastI.at(iter) / 2.0 * tBrake;
//
//						/* so this is all for this variable */
//						this->switchToLowerVelocity.at(iter) = false;
//
//					}
//					else {
//						vNowI.at(iter) = vLastI.at(iter) - sign( vLastI.at(iter) - sign(vLastI.at(iter))*vMaxI.at(iter) ) * aMaxI.at(iter) * dt;
//						xNowI.at(iter) = xLastI.at(iter) + vNowI.at(iter) / 2.0 * dt;
//						//xNowI.at(iter) = xLastI.at(iter) + (vLastI.at(iter) + vNowI.at(iter)) / 2.0 * dt;
//					}
//
//					// and if, changing velocity, we hit the target?
//					if ( fabs( xToGo.at(iter) - xNowI.at(iter) ) < TARGETTOLERANCE ) {
//						vNowI.at(iter) = 0.0;
//						xNowI.at(iter) = xTargetI.at(iter);
//
//						this->switchToLowerVelocity.at(iter) = false;
//					}
//
//				}
//			}

			for (unsigned int iter = 0; iter < num_var; iter++) {


				// where would I end up if I decelerated to v = 0?
				tBrake = fabs(vLastI.at(iter)) / aMaxI.at(iter);

				xBrakeEnd = vLastI.at(iter) / 2.0 * tBrake; // seen from current position

				// find maximum of needed velocity (positive or negative)
				if (xBrakeEnd <= xToGo.at(iter)) { // more positive velocity (vInt>vLastI, vInt>0)

					// where would I end up when I accelerated up to + vmax and then braked? ( = jolt)
					tToVMax = fabs(vMaxI.at(iter) - vLastI.at(iter)) / aMaxI.at(iter);
					tBrake = vMaxI.at(iter) / aMaxI.at(iter);
					xJolt = (vLastI.at(iter) + vMaxI.at(iter)) / 2.0 * tToVMax + vMaxI.at(iter) * tBrake / 2.0;

					a1.at(iter) = aMaxI.at(iter);
					a2.at(iter) = -aMaxI.at(iter);

					if (xJolt <= xToGo.at(iter)) { // not enough, stay at vMax for some time

						vInt.at(iter) = vMaxI.at(iter);
						T1.at(iter) = tToVMax;
						T2.at(iter) = (xToGo.at(iter) - xJolt) / vMaxI.at(iter);
						T3.at(iter) = tBrake;

					}
					else { // that would be too far, find the appropriate speed maximum

						radic = xToGo.at(iter) * aMaxI.at(iter) + pow(vLastI.at(iter), 2.0) / 2.0;
						if (radic < 0.0)
							radic = 0.0; // prevent numerical errors

						vInt.at(iter) = sqrt(radic);
						T1.at(iter) = (vInt.at(iter) - vLastI.at(iter)) / aMaxI.at(iter);
						T2.at(iter) = 0.0;
						T3.at(iter) = vInt.at(iter) / aMaxI.at(iter);

					}

				}
				else { // more negative velocity (vInt<vLastI, vInt<0) where would I end up when I accelerated up to -vmax and then braked? ( = jolt)

					tToVMax = fabs(-vMaxI.at(iter) - vLastI.at(iter)) / aMaxI.at(iter);
					tBrake = vMaxI.at(iter) / aMaxI.at(iter);
					xJolt = (vLastI.at(iter) - vMaxI.at(iter)) / 2.0 * tToVMax - vMaxI.at(iter) * tBrake / 2.0;

					a1.at(iter) = -aMaxI.at(iter);
					a2.at(iter) = aMaxI.at(iter);
					if (xJolt >= xToGo.at(iter)) { // not enough, stay at - vMax for some time

						vInt.at(iter) = -vMaxI.at(iter);
						T1.at(iter) = tToVMax;
						T2.at(iter) = (xToGo.at(iter) - xJolt) / -vMaxI.at(iter);
						T3.at(iter) = tBrake;

					}
					else { // that would be too far, find the appropriate speed maximum

						radic = -xToGo.at(iter) * aMaxI.at(iter) + pow(vLastI.at(iter), 2.0) / 2.0;
						if (radic < 0.0)
							radic = 0.0; // prevent numerical errors

						vInt.at(iter) = -sqrt(radic);
						T1.at(iter) = (-vInt.at(iter) + vLastI.at(iter)) / aMaxI.at(iter);
						T2.at(iter) = 0.0;
						T3.at(iter) = -vInt.at(iter) / aMaxI.at(iter);

					}
				}

				// the case that vInt lies between 0 and vLastI can not happen, nothing would happen there
				// (e.g. a = -amax until v = vInt and then further a = -amax until v = 0)

				TSum.at(iter) = T1.at(iter) + T2.at(iter) + T3.at(iter);
			}

			if (interpolation_mode == 1) {
//				log(RTT::Debug) << "Interpolation mode 1" << endlog();

				// now find the joint that will take the longest
				TMax = *(std::max_element(TSum.begin(), TSum.end()));
				//int iTMax = std::distance(TSum.begin(), std::find(TSum.begin(), TSum.end(), TMax));

				// now adjust all the other trajectories
				for (unsigned int iter = 0; iter < num_var; iter++) {


					if (TSum.at(iter) >= (TMax - dt / 100.0))
						continue; // no need to replan, will end almost synchronously

					flip = false;
					// now to find the new vInt, solve:
					// xToGo.at(iter) = Tmax * vInt - ( (vInt - vLastI)|vInt - vLastI| + vInt|vInt| ) / 2aMax

					if (vInt.at(iter) < 0.0) { // vInt will not change its sign, always having positive vInt will simplify further calculations by reducing branches
						flip = true;
						vInt.at(iter) = -vInt.at(iter);
						xToGo.at(iter) = -xToGo.at(iter);
						vLastI.at(iter) = -vLastI.at(iter);
					}

					if ((vLastI.at(iter) < 0.0) || ((vLastI.at(iter) * TMax - pow(vLastI.at(iter), 2.0) / 2.0 / aMaxI.at(iter)) < xToGo.at(iter))) { // vInt will be higher than vLastI and 0
						radic = pow(TMax, 2.0) * pow(aMaxI.at(iter), 2.0) + 2.0 * TMax * aMaxI.at(iter) * vLastI.at(iter) - pow(vLastI.at(iter), 2.0) - 4.0 * xToGo.at(iter) * aMaxI.at(iter);
						if (radic < 0.0)
							radic = 0.0;
						vInt.at(iter) = 0.5 * (TMax * aMaxI.at(iter) + vLastI.at(iter) - sqrt(radic));
					}
					else { // vInt will lie between 0 and vLastI
						double nom = (xToGo.at(iter) * aMaxI.at(iter) - pow(vLastI.at(iter), 2.0) / 2.0);
						double den = (TMax * aMaxI.at(iter) - vLastI.at(iter));
						//if ((nom < numeric_limits<double>::min()) && (den < numeric_limits<double>::min()))	// we will stay 0 seconds on vInt, so this is undefined
						if ((nom < TOLERANCE) && (den < TOLERANCE)) // we will stay 0 seconds on vInt, so this is undefined
							vInt.at(iter) = vLastI.at(iter);
						else
							vInt.at(iter) = nom / den;
					}

					T1.at(iter) = fabs(vInt.at(iter) - vLastI.at(iter)) / aMaxI.at(iter);
					T3.at(iter) = vInt.at(iter) / aMaxI.at(iter);
					T2.at(iter) = TMax - T1.at(iter) - T3.at(iter);

					a1.at(iter) = sgn(vInt.at(iter) - vLastI.at(iter)) * aMaxI.at(iter);
					a2.at(iter) = -aMaxI.at(iter);

					if (flip) {
						vInt.at(iter) = -vInt.at(iter);
						vLastI.at(iter) = -vLastI.at(iter);
						a1.at(iter) = -a1.at(iter);
						a2.at(iter) = -a2.at(iter);
					}
				}
			}


			// now drive
			xAfterT1 = 0.0;
			xAfterT2 = 0.0;
			for (unsigned int iter = 0; iter < num_var; iter++) {


				// now let's see where I am on the desired trajectory...
				if (dt <= T1.at(iter)) {
					vNowI.at(iter) = vLastI.at(iter) + a1.at(iter) * dt;
					xNowI.at(iter) = xLastI.at(iter) + (vLastI.at(iter) + vNowI.at(iter)) / 2.0 * dt;
				}
				else {
					xAfterT1 = xLastI.at(iter) + (vInt.at(iter) + vLastI.at(iter)) / 2.0 * T1.at(iter);

					if (dt <= T1.at(iter) + T2.at(iter)) {
						vNowI.at(iter) = vInt.at(iter);
						xNowI.at(iter) = xAfterT1 + (dt - T1.at(iter)) * vInt.at(iter);
					}
					else {
						xAfterT2 = xAfterT1 + vInt.at(iter) * T2.at(iter);

						if (dt <= T1.at(iter) + T2.at(iter) + T3.at(iter)) {
							vNowI.at(iter) = vInt.at(iter) + a2.at(iter) * (dt - T1.at(iter) - T2.at(iter));
							xNowI.at(iter) = xAfterT2 + (vInt.at(iter) + vNowI.at(iter)) / 2.0 * (dt - T1.at(iter) - T2.at(iter));
						}
						else {
//							log(RTT::Debug) << "Directly to the target" << endlog();
							vNowI.at(iter) = 0.0;
							xNowI.at(iter) = xTargetI.at(iter);
						}
					}
				}
			}

			break;
		case 2:

			//vNoOS=sqrt(2*aMax*xToGo);
			brakeAll = false;
			for (unsigned int iter = 0; iter < num_var; iter++) {


				// acceleration necessary to get from the last status (x and v) to the desired x,
				// assumed that the acceleration would be constant during the whole dt
				// (which could lead to premature braking during the next checks):
				//a = (xTargetI.at(iter) - xLastI.at(iter) - vLastI.at(iter) * dt) * 2.0 / pow(dt, 2.0);
				a = (xToGo.at(iter) - vLastI.at(iter) * dt) * 2.0 / pow(dt, 2.0);

				if ((a > aMaxI.at(iter)) || (a < (-aMaxI.at(iter)))) {
					brakeAll = true;
					break;
				}

				// velocity that would be reached:
				v = vLastI.at(iter) + a * dt;

				if (v > vMaxI.at(iter) || v < -vMaxI.at(iter)) {
					brakeAll = true;
					break;
				}

				// maximum negative and positive velocity here so that no overshoot happens at xMin or xMax
				vNowIMinNoOS = -sqrt(2.0 * aMaxI.at(iter) * fabs(xMinI.at(iter) - xTargetI.at(iter)));
				vNowIMaxNoOS = sqrt(2.0 * aMaxI.at(iter) * fabs(xMaxI.at(iter) - xTargetI.at(iter)));

				if (v > vNowIMaxNoOS || v < vNowIMinNoOS) {
					brakeAll = true;
					break;
				}

				// looks like everything is ok
				xNowI.at(iter) = xTargetI.at(iter);
				vNowI.at(iter) = v;
			}

			if (brakeAll) {
				for (unsigned int iter = 0; iter < num_var; iter++) {
					// can we brake down to v=0 within dt?
					if (fabs(vLastI.at(iter)) <= aMaxI.at(iter) * dt) {
						vNowI.at(iter) = 0.0;
						tBrake = vLastI.at(iter) / aMaxI.at(iter);
						xNowI.at(iter) = xLastI.at(iter) + vLastI.at(iter) / 2.0 * tBrake;
					}
					else {
						vNowI.at(iter) = vLastI.at(iter) - sgn(vLastI.at(iter)) * aMaxI.at(iter) * dt;
						xNowI.at(iter) = xLastI.at(iter) + (vLastI.at(iter) + vNowI.at(iter)) / 2.0 * dt;
					}
				}
			}

			break;
		case 3:

//			log(RTT::Debug) << "Interpolation mode 3" << endlog();
			xNowI = xTargetI;
			vNowI = vLastI;
			break;
		default:
//			log(RTT::Warning) << "Wrong interpolation mode selected " << interpolation_mode << ", switching to 0 instead" << endlog();
			interpolation_mode = 0;;
			break;
	}
	return true;
}


template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}



