
/*=====================================================================
PIXHAWK Micro Air Vehicle Flying Robotics Toolkit
(c) 2009, 2010 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>
This file is part of the PIXHAWK project
    PIXHAWK is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    PIXHAWK is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with PIXHAWK. If not, see <http://www.gnu.org/licenses/>.
======================================================================*/

/**
 * @file
 *   @brief Implementation of altitude Kalman filter
 *
 *   @author Florian Zurbriggen <mavteam@student.ethz.ch>
 *   @author Martin Rutschmann <mavteam@student.ethz.ch>
 *
 */

#include "ukf_bmp.h"

/**
 * @param position_meas sensor data from position measurement [m]
 * @param accel_meas data from acceleration measurement [m/s^2] (bias of sensor doesn't influence the result)
 * @param position_estimation estimated position [m]
 * @param speed_estimation estimated speed [m/s]
 * @param bias_accel estimated bias of the acceleration measurement [m/s^2] -> effective acceleration = accel_meas - bias_accel
 * @param Ts Sampling rate of filter [s]
 * @param sigmaAcc standard deviation acceleration sensor [m/s^2]
 * @param sigmaPos standard deviation distance sensor [m/s]
 * @param gammaAcc prediction confidence of the acceleration sensor
 * @param gammaBiasAcc prediction confidence of the bias part
 * @param sigmaAcc
 * @param pkk_1
 * @param state_pred
 */
void altitude_kalman(float position_mes, float accel_mes, float Ts, float sigmaAcc, float sigmaPos, float gammaAcc, float gammaBiasAcc, float* pkk_1, float* state_pred, float* speed_estimation, float* bias_accel, float* position_estimation)
{
//	function [x_est_new x_pred_new Pkk_1_new] = altitude_kalman_C2(x_pred, Pkk_1, altMeas, accMeas, Ts, sgmAcc, sgmAlt, Gmacc, Gmbias)

	float P11 = pkk_1[0];
	float P12 = pkk_1[1];
	float P13 = pkk_1[2];
	float P21 = pkk_1[3];
	float P22 = pkk_1[4];
	float P23 = pkk_1[5];
	float P31 = pkk_1[6];
	float P32 = pkk_1[7];
	float P33 = pkk_1[8];

	float hPred = state_pred[0];
	float hDotPred = state_pred[1];
	float BiasPred = state_pred[2];

	// New Estimation
	float hEstNew = hPred + P11*(position_mes - hPred)/(P11 + sigmaPos*sigmaPos);
	float hDotEstNew = hDotPred + P21*(position_mes - hPred)/(P11 + sigmaPos*sigmaPos);
	float BiasEstNew = BiasPred + P31*(position_mes - hPred)/(P11 + sigmaPos*sigmaPos);

	*position_estimation = hEstNew;
	*speed_estimation = hDotEstNew;
	*bias_accel = BiasEstNew;

	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	//% here control step out of the state estimation x_est %
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	// New prediction and update (embedded)
	float hPredNew = hEstNew + Ts*hDotEstNew - Ts*Ts/2.0f*BiasEstNew + Ts*Ts/2.0f*accel_mes;
	float hDotPredNew = hDotEstNew -Ts*BiasEstNew + Ts*accel_mes;
	float BiasPredNew = BiasEstNew;

	state_pred[0]=hPredNew;
	state_pred[1]=hDotPredNew;
	state_pred[2]=BiasPredNew;


	float P11New = (1.0f/(60.0f*(P11 + sigmaPos*sigmaPos)))*(P11*(60*sigmaPos*sigmaPos + Ts*Ts*(60.0f*P22 + Ts*(20.0f*gammaAcc - 30.0f*P23 - 30.0f*P32 + 15.0f*P33*Ts + 3.0f*gammaBiasAcc*Ts*Ts))) + Ts*(30.0f*P21*(2.0f*sigmaPos*sigmaPos + P13*Ts*Ts) + 30.0f*P12*(2.0f*sigmaPos*sigmaPos + Ts*(-2.0f*P21 + P31*Ts)) + Ts*(-15.0f*P13*(2.0f*sigmaPos*sigmaPos + P31*Ts*Ts) + sigmaPos*sigmaPos*(60.0f*P22 - 30*P31 + Ts*(20.0f*gammaAcc - 30.0f*P23 - 30.0f*P32 + 15.0f*P33*Ts + 3.0f*gammaBiasAcc*Ts*Ts)))));
	float P12New = (1.0f/(8.0f*(P11 + sigmaPos*sigmaPos)))*(4.0f*P12*(2.0f*sigmaPos*sigmaPos + Ts*(-2.0f*P21 + P31*Ts)) + Ts*(-4.0f*P13*(2.0f*sigmaPos*sigmaPos + Ts*(-2.0f*P21 + P31*Ts)) + P11*(8.0f*P22 + Ts*(4.0f*gammaAcc - 8.0f*P23 - 4.0f*P32 + 4.0f*P33*Ts + gammaBiasAcc*Ts*Ts)) + sigmaPos*sigmaPos*(8.0f*P22 + Ts*(4.0f*gammaAcc - 8.0f*P23 - 4.0f*P32 + 4.0f*P33*Ts + gammaBiasAcc*Ts*Ts))));
	float P13New = ((P11 + sigmaPos*sigmaPos)*Ts*(6.0f*P23 - Ts*(3.0f*P33 + gammaBiasAcc*Ts)) + 3.0f*P13*(2.0f*sigmaPos*sigmaPos + Ts*(-2.0f*P21 + P31*Ts)))/(6.0f*(P11 + sigmaPos*sigmaPos));
	float P21New = P21 - (P11*P21)/(P11 + sigmaPos*sigmaPos) - (P31*sigmaPos*sigmaPos*Ts)/(P11 + sigmaPos*sigmaPos) + (gammaAcc*Ts*Ts)/2.0f + (gammaBiasAcc*Ts*Ts*Ts*Ts)/8.0f + (Ts*(P12*(-P21 + P31*Ts) + (P11 + sigmaPos*sigmaPos)*(P22 - P32*Ts)))/(P11 + sigmaPos*sigmaPos) - (Ts*Ts*(P13*(-P21 + P31*Ts) + (P11 + sigmaPos*sigmaPos)*(P23 - P33*Ts)))/(2.0f*(P11 + sigmaPos*sigmaPos));
	float P22New = P22 + (P12*(-P21 + P31*Ts))/(P11 + sigmaPos*sigmaPos) + (1.0f/3.0f)*Ts*(3.0f*gammaAcc - 3.0f*P23 - 3.0f*P32 + (3.0f*P13*P21)/(P11 + sigmaPos*sigmaPos) + 3.0f*P33*Ts - (3.0f*P13*P31*Ts)/(P11 + sigmaPos*sigmaPos) + gammaBiasAcc*Ts*Ts);
	float P23New = P23 - (1.0f/2.0f)*Ts*(2.0f*P33 + gammaBiasAcc*Ts) + (P13*(-P21 + P31*Ts))/(P11 + sigmaPos*sigmaPos);
	float P31New = ((P11 + sigmaPos*sigmaPos)*Ts*(6.0f*P32 - Ts*(3.0f*P33 + gammaBiasAcc*Ts)) + 3.0f*P31*(2.0f*sigmaPos*sigmaPos + Ts*(-2.0f*P12 + P13*Ts)))/(6.0f*(P11 + sigmaPos*sigmaPos));
	float P32New = P32 - (P12*P31)/(P11 + sigmaPos*sigmaPos) - P33*Ts + (P13*P31*Ts)/(P11 + sigmaPos*sigmaPos) - (gammaBiasAcc*Ts*Ts)/2.0f;
	float P33New = P33 - (P13*P31)/(P11 + sigmaPos*sigmaPos) + gammaBiasAcc*Ts;

	pkk_1[0]=P11New;
	pkk_1[1]=P12New;
	pkk_1[2]=P13New;
	pkk_1[3]=P21New;
	pkk_1[4]=P22New;
	pkk_1[5]=P23New;
	pkk_1[6]=P31New;
	pkk_1[7]=P32New;
	pkk_1[8]=P33New;

}
