
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
 *   @brief Definition of altitude Kalman filter
 *
 *   @author Florian Zurbriggen <mavteam@student.ethz.ch>
 *   @author Martin Rutschmann <mavteam@student.ethz.ch>
 *
 */

#ifndef ALTITUDE_KALMAN_H_
#define ALTITUDE_KALMAN_H_

void altitude_kalman(float position_mes, float accel_mes, float Ts, float sigmaAcc, float sigmaPos, float gammaAcc, float gammaBiasAcc, float* pkk_1, float* state_pred, float* speed_estimation, float* bias_accel, float* position_estimation);

#endif /* ALTITUDE_KALMAN_H_ */
