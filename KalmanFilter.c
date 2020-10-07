/*
 * KalmanFilter.c
 *
 *  Created on: Oct 5, 2020
 *      Author: Alex
 */
#include "MPU6050.h"
#include "QMC5883.h"
#include <math.h>

//static int16_t k_rawAcceleration[3];
//static int16_t k_rawGyro[3];
//static int16_t k_rawMagnetometer[3];
/*
void kalmanFilter_init(kalFilter_t *k_Filter)
{

    //Kalman filter variables
    k_Filter->Xk[0] = 0;
    k_Filter->Xk[1] = 0;
    k_Filter->Xk[2] = 0;
    k_Filter->Xk[3] = 0;

    //Measurement Variables
    k_Filter->phiAngle = 0.0;
    k_Filter->thethaAngle = 0.0;

    k_Filter->phiDot = 0.0;
    k_Filter->thetaDot = 0.0;

    k_Filter->accelTetha = 0.0;
    k_Filter->accelPhi = 0.0;

    //State Covariance Matrix: Variance and Covariance of Estimation process, it defines the error in the estimate (our matematical model)
    k_Filter->Pk[0][0] = 0;
    k_Filter->Pk[0][1] = 0;
    k_Filter->Pk[0][2] = 0;
    k_Filter->Pk[0][3] = 0;

    k_Filter->Pk[1][0] = 0;
    k_Filter->Pk[1][1] = 0;
    k_Filter->Pk[1][2] = 0;
    k_Filter->Pk[1][3] = 0;

    k_Filter->Pk[2][0] = 0;
    k_Filter->Pk[2][1] = 0;
    k_Filter->Pk[2][2] = 0;
    k_Filter->Pk[2][3] = 0;

    k_Filter->Pk[3][0] = 0;
    k_Filter->Pk[3][1] = 0;
    k_Filter->Pk[3][2] = 0;
    k_Filter->Pk[3][3] = 0;

   //Measurement Covariance Matrix: This are the variance and covariance of Angle and Angular Velocity Measurements
    //Upper right triangle of Covariance Matrix:
    k_Filter->R[0][0] = 577.0633;
    k_Filter->R[0][1] = 7.1660;
    k_Filter->R[0][2] = -20.2332;
    k_Filter->R[0][3] = -7.3431;

    k_Filter->R[1][1] = 583.2966;
    k_Filter->R[1][2] = 8.4916;
    k_Filter->R[1][3] = 88.8020;

    k_Filter->R[2][2] = 193.5718;
    k_Filter->R[2][3] = 11.5191;

    k_Filter->R[3][3] = 86.2707;

    //Lower Triangle
    k_Filter->R[1][0] = k_Filter->R[0][1];
    k_Filter->R[2][0] = k_Filter->R[0][2];
    k_Filter->R[2][1] = k_Filter->R[1][2];

    k_Filter->R[3][0] = k_Filter->R[0][3];
    k_Filter->R[3][1] = k_Filter->R[1][3];
    k_Filter->R[3][2] = k_Filter->R[2][3];


    //Instrument Covariance Matrix: Defines the errors due to electric wave propagation and tolerances of our measurement instruments. NOT TO BE CONFUSED WITH THE MEASUEMENT COVARIANCE METRIX
    k_Filter->Q[0][0] = 0.3;   //Acceleration Tolerances
    k_Filter->Q[0][1] = 0.0;
    k_Filter->Q[0][2] = 0.0;
    k_Filter->Q[0][3] = 0.0;

    k_Filter->Q[1][0] = 0.0;   //Acceleration Tolerances
    k_Filter->Q[1][1] = 0.3;
    k_Filter->Q[1][2] = 0.0;
    k_Filter->Q[1][3] = 0.0;

    k_Filter->Q[2][0] = 0.0;   //Ang Vel Tolerance
    k_Filter->Q[2][1] = 0.0;
    k_Filter->Q[2][2] = 0.0;
    k_Filter->Q[2][3] = 0.0;

    k_Filter->Q[3][0] = 0.0;  //Ang Vel Tolerance
    k_Filter->Q[3][1] = 0.0;
    k_Filter->Q[3][2] = 0.0;
    k_Filter->Q[3][3] = 0.0;

}


void estimateAngle(kalFilter_t * k_Filter, const float timeElapsed)
{

    MPU6050_raw_accelerometer(k_rawAcceleration);
    MPU6050_raw_gyroscope(k_rawGyro);
    QMC5883_raw_magnetometer(k_rawMagnetometer);


    //Prediction ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //Estimated Model
    k_Filter->Xk[0] = k_Filter->Xk[0] + timeElapsed*(k_Filter->Xk[2] + 1/2*timeElapsed*k_Filter->accelTetha);
    k_Filter->Xk[1] = k_Filter->Xk[1] + timeElapsed*(k_Filter->Xk[3] + 1/2*timeElapsed*k_Filter->accelPhi);
    k_Filter->Xk[2] = k_Filter->Xk[2] + timeElapsed*k_Filter->accelTetha;
    k_Filter->Xk[3] = k_Filter->Xk[3] + timeElapsed*k_Filter->accelTetha;

    //Estimated State Covariance Matrix (error in the estimate)
    k_Filter->Pk[0][0] = k_Filter->Pk[0][0] +timeElapsed*(k_Filter->Pk[2][0] + k_Filter->Pk[0][2] + timeElapsed*k_Filter->Pk[2][2] ) + k_Filter->Q[0][0];
    k_Filter->Pk[0][1] = k_Filter->Pk[0][1] +timeElapsed*(k_Filter->Pk[2][1] + k_Filter->Pk[0][3] + timeElapsed*k_Filter->Pk[2][3] ) + k_Filter->Q[0][1];
    k_Filter->Pk[0][2] = k_Filter->Pk[0][2] +timeElapsed*(k_Filter->Pk[2][2]) + k_Filter->Q[0][2];
    k_Filter->Pk[0][3] = k_Filter->Pk[0][3] +timeElapsed*(k_Filter->Pk[2][3]) + k_Filter->Q[0][3];

    k_Filter->Pk[1][0] = k_Filter->Pk[1][0] +timeElapsed*(k_Filter->Pk[3][0] + k_Filter->Pk[1][2] + timeElapsed*k_Filter->Pk[3][2] ) + k_Filter->Q[1][0];
    k_Filter->Pk[1][1] = k_Filter->Pk[1][1] +timeElapsed*(k_Filter->Pk[3][1] + k_Filter->Pk[1][3] + timeElapsed*k_Filter->Pk[3][3] ) + k_Filter->Q[1][1]; /// EERRORRR (?)
    k_Filter->Pk[1][2] = k_Filter->Pk[1][2] +timeElapsed*(k_Filter->Pk[3][2]) + k_Filter->Q[1][2];
    k_Filter->Pk[1][3] = k_Filter->Pk[1][3] +timeElapsed*(k_Filter->Pk[3][3]) + k_Filter->Q[1][3];

    k_Filter->Pk[2][0] = k_Filter->Pk[2][0] +timeElapsed*(k_Filter->Pk[2][2]) + k_Filter->Q[2][0];
    k_Filter->Pk[2][1] = k_Filter->Pk[2][1] +timeElapsed*(k_Filter->Pk[2][3]) + k_Filter->Q[2][1];
    k_Filter->Pk[2][2] = k_Filter->Pk[2][2] + k_Filter->Q[2][2];
    k_Filter->Pk[2][3] = k_Filter->Pk[2][3] + k_Filter->Q[2][3];

    k_Filter->Pk[3][0] = k_Filter->Pk[3][0] +timeElapsed*(k_Filter->Pk[3][2]) + k_Filter->Q[3][0];
    k_Filter->Pk[3][1] = k_Filter->Pk[3][1] +timeElapsed*(k_Filter->Pk[3][3]) + k_Filter->Q[3][1];
    k_Filter->Pk[3][2] = k_Filter->Pk[3][2] + k_Filter->Q[3][2];
    k_Filter->Pk[3][3] = k_Filter->Pk[3][3] + k_Filter->Q[3][3];

    //Kalman Gain ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    k_Filter->K[0][0] = k_Filter->Pk[0][0]/(k_Filter->Pk[0][0] + k_Filter->R[0][0]);
    k_Filter->K[0][1] = k_Filter->Pk[0][1]/(k_Filter->Pk[0][1] + k_Filter->R[0][1]);
    k_Filter->K[0][2] = k_Filter->Pk[0][2]/(k_Filter->Pk[0][2] + k_Filter->R[0][2]);
    k_Filter->K[0][3] = k_Filter->Pk[0][3]/(k_Filter->Pk[0][3] + k_Filter->R[0][3]);

    k_Filter->K[1][0] = k_Filter->Pk[1][0]/(k_Filter->Pk[1][0] + k_Filter->R[1][0]);
    k_Filter->K[1][1] = k_Filter->Pk[1][1]/(k_Filter->Pk[1][1] + k_Filter->R[1][1]);
    k_Filter->K[1][2] = k_Filter->Pk[1][2]/(k_Filter->Pk[1][2] + k_Filter->R[1][2]);
    k_Filter->K[1][3] = k_Filter->Pk[1][3]/(k_Filter->Pk[1][3] + k_Filter->R[1][3]);

    k_Filter->K[2][0] = k_Filter->Pk[2][0]/(k_Filter->Pk[2][0] + k_Filter->R[2][0]);
    k_Filter->K[2][1] = k_Filter->Pk[2][1]/(k_Filter->Pk[2][1] + k_Filter->R[2][1]);
    k_Filter->K[2][2] = k_Filter->Pk[2][2]/(k_Filter->Pk[2][2] + k_Filter->R[2][2]);
    k_Filter->K[2][3] = k_Filter->Pk[2][3]/(k_Filter->Pk[2][3] + k_Filter->R[2][3]);

    k_Filter->K[3][0] = k_Filter->Pk[3][0]/(k_Filter->Pk[3][0] + k_Filter->R[3][0]);
    k_Filter->K[3][1] = k_Filter->Pk[3][1]/(k_Filter->Pk[3][1] + k_Filter->R[3][1]);
    k_Filter->K[3][2] = k_Filter->Pk[3][2]/(k_Filter->Pk[3][2] + k_Filter->R[3][2]);
    k_Filter->K[3][3] = k_Filter->Pk[3][3]/(k_Filter->Pk[3][3] + k_Filter->R[3][3]);

    //Observed (measured) Angle
    //ROLL ( THETA) & Convert from radians to degrees
    k_Filter->thethaAngle =atan(k_rawAcceleration[1]/sqrt(k_rawAcceleration[0]*k_rawAcceleration[0] + k_rawAcceleration[2]*k_rawAcceleration[2]));
    k_Filter->thethaAngle  = (k_Filter->thethaAngle*180.0)/M_PI;

    // Pitch (Phi) & Convert from radians to degrees
    k_Filter->phiAngle = atan((-1)*k_rawAcceleration[0]/sqrt(k_rawAcceleration[1]*k_rawAcceleration[1] + k_rawAcceleration[2]*k_rawAcceleration[2]));
    k_Filter->phiAngle = (k_Filter->phiAngle*180.0)/M_PI;



    //Observed (measured) Angular Velocity
    //Roll Angular velocity (thetaDot)
    k_Filter->thetaDot = k_rawGyro[0];  // angularVel on Roll direction   //New angular velocity reading from gyro: theta'[n]
    //Pitch Angular velocity (phiDot)
    k_Filter->phiDot = k_rawGyro[1];    //angularVel on Pitch direction New angular velocity reading from gyro: theta'[n]

    //Observed (measured) measurements
    k_Filter->Y[0] = k_Filter->thethaAngle - k_Filter->Xk[0];  //Yk Theta (roll)
    k_Filter->Y[1] = k_Filter->phiAngle    - k_Filter->Xk[1];  //Yk Phi  (pitch)
    k_Filter->Y[2] = k_Filter->thetaDot    - k_Filter->Xk[2];  //Yk ThetaDot
    k_Filter->Y[3] = k_Filter->phiDot      - k_Filter->Xk[3];  //Yk phiDot

    //Update Estate (combining Prediction and Measurements)
    k_Filter->Xk[0] = k_Filter->Xk[0] + k_Filter->K[0][0]*k_Filter->Y[0];// + k_Filter->K[0][1]*k_Filter->Y[1] + k_Filter->K[0][2]*k_Filter->Y[2] + k_Filter->K[0][3]*k_Filter->Y[3];
    k_Filter->Xk[1] = k_Filter->Xk[1] + k_Filter->K[1][0]*k_Filter->Y[1] + k_Filter->K[1][1]*k_Filter->Y[1] + k_Filter->K[1][2]*k_Filter->Y[2] + k_Filter->K[1][3]*k_Filter->Y[3];
    k_Filter->Xk[2] = k_Filter->Xk[2] + k_Filter->K[2][0]*k_Filter->Y[0] + k_Filter->K[2][1]*k_Filter->Y[1] + k_Filter->K[2][2]*k_Filter->Y[2] + k_Filter->K[2][3]*k_Filter->Y[3];
    k_Filter->Xk[3] = k_Filter->Xk[3] + k_Filter->K[3][0]*k_Filter->Y[0] + k_Filter->K[3][1]*k_Filter->Y[1] + k_Filter->K[3][2]*k_Filter->Y[2] + k_Filter->K[3][3]*k_Filter->Y[3];

    //Update Process Matrix
    k_Filter->Pk[0][0] = (1-k_Filter->K[0][0])*k_Filter->Pk[0][0] - k_Filter->K[0][1]*k_Filter->Pk[1][0] - k_Filter->K[0][2]*k_Filter->Pk[2][0] - k_Filter->K[0][3]*k_Filter->Pk[3][0];
    k_Filter->Pk[0][1] = (1-k_Filter->K[0][0])*k_Filter->Pk[0][1] - k_Filter->K[0][1]*k_Filter->Pk[1][1] - k_Filter->K[0][2]*k_Filter->Pk[2][1] - k_Filter->K[0][3]*k_Filter->Pk[3][1];
    k_Filter->Pk[0][2] = (1-k_Filter->K[0][0])*k_Filter->Pk[0][2] - k_Filter->K[0][1]*k_Filter->Pk[1][2] - k_Filter->K[0][2]*k_Filter->Pk[2][2] - k_Filter->K[0][3]*k_Filter->Pk[3][2];
    k_Filter->Pk[0][3] = (1-k_Filter->K[0][0])*k_Filter->Pk[0][3] - k_Filter->K[0][1]*k_Filter->Pk[1][3] - k_Filter->K[0][2]*k_Filter->Pk[2][3] - k_Filter->K[0][3]*k_Filter->Pk[3][3];

    k_Filter->Pk[1][0] = (1-k_Filter->K[1][1])*k_Filter->Pk[1][0] - k_Filter->K[1][0]*k_Filter->Pk[0][0] - k_Filter->K[1][2]*k_Filter->Pk[2][0] - k_Filter->K[1][3]*k_Filter->Pk[3][0];
    k_Filter->Pk[1][1] = (1-k_Filter->K[1][1])*k_Filter->Pk[1][1] - k_Filter->K[1][0]*k_Filter->Pk[0][1] - k_Filter->K[1][2]*k_Filter->Pk[2][1] - k_Filter->K[1][3]*k_Filter->Pk[3][1];
    k_Filter->Pk[1][2] = (1-k_Filter->K[1][1])*k_Filter->Pk[1][2] - k_Filter->K[1][0]*k_Filter->Pk[0][2] - k_Filter->K[1][2]*k_Filter->Pk[2][2] - k_Filter->K[1][3]*k_Filter->Pk[3][2];
    k_Filter->Pk[1][3] = (1-k_Filter->K[1][1])*k_Filter->Pk[1][3] - k_Filter->K[1][0]*k_Filter->Pk[0][3] - k_Filter->K[1][2]*k_Filter->Pk[2][3] - k_Filter->K[1][3]*k_Filter->Pk[3][3];

    k_Filter->Pk[2][0] = (1-k_Filter->K[2][2])*k_Filter->Pk[2][0] - k_Filter->K[2][0]*k_Filter->Pk[0][0] - k_Filter->K[2][1]*k_Filter->Pk[1][0] - k_Filter->K[2][3]*k_Filter->Pk[3][0];
    k_Filter->Pk[2][1] = (1-k_Filter->K[2][2])*k_Filter->Pk[2][1] - k_Filter->K[2][0]*k_Filter->Pk[0][1] - k_Filter->K[2][1]*k_Filter->Pk[1][1] - k_Filter->K[2][3]*k_Filter->Pk[3][1];
    k_Filter->Pk[2][2] = (1-k_Filter->K[2][2])*k_Filter->Pk[2][2] - k_Filter->K[2][0]*k_Filter->Pk[0][2] - k_Filter->K[2][1]*k_Filter->Pk[1][2] - k_Filter->K[2][3]*k_Filter->Pk[3][2];
    k_Filter->Pk[2][3] = (1-k_Filter->K[2][2])*k_Filter->Pk[2][3] - k_Filter->K[2][0]*k_Filter->Pk[0][3] - k_Filter->K[2][1]*k_Filter->Pk[1][3] - k_Filter->K[2][3]*k_Filter->Pk[3][3];

    k_Filter->Pk[3][0] = (1-k_Filter->K[3][3])*k_Filter->Pk[3][0] - k_Filter->K[3][0]*k_Filter->Pk[0][0] - k_Filter->K[3][1]*k_Filter->Pk[1][0] - k_Filter->K[3][2]*k_Filter->Pk[2][0];
    k_Filter->Pk[3][0] = (1-k_Filter->K[3][3])*k_Filter->Pk[3][1] - k_Filter->K[3][0]*k_Filter->Pk[0][1] - k_Filter->K[3][1]*k_Filter->Pk[1][1] - k_Filter->K[3][2]*k_Filter->Pk[2][1];
    k_Filter->Pk[3][0] = (1-k_Filter->K[3][3])*k_Filter->Pk[3][2] - k_Filter->K[3][0]*k_Filter->Pk[0][2] - k_Filter->K[3][1]*k_Filter->Pk[1][2] - k_Filter->K[3][2]*k_Filter->Pk[2][2];
    k_Filter->Pk[3][0] = (1-k_Filter->K[3][3])*k_Filter->Pk[3][3] - k_Filter->K[3][0]*k_Filter->Pk[0][3] - k_Filter->K[3][1]*k_Filter->Pk[1][3] - k_Filter->K[3][2]*k_Filter->Pk[2][3];
}
*/
