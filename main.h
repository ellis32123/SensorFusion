// this is the header file for the program
// main.cpp

#ifndef MAIN_H
#define MAIN_H

//#include "Arduino.h"

#include <SensorFusion.h>

struct EulerAngles{
	float roll  = 0.0f,
	      pitch = 0.0f,
	      yaw   = 0.0f;
};

void initialize();

EulerAngles convertQuaternionToEulerAngles(float q0,float q1,float q2,float q3);

void sPrintInitializationData(int dps);

inline void sPrintRawGyroscope(int dps);
inline void sPrintRawAccelerometer(int dps);
inline void sPrintRawMagnetometer(int dps);
inline void sPrintRawAltimeter(int dps);

inline void sPrintMeanGyroscope(int dps);
inline void sPrintMeanAccelerometer(int dps);
inline void sPrintMeanMagnetometer(int dps);
inline void sPrintMeanAltimeter(int dps);

inline void sPrintVarianceGyroscope(int dps);
inline void sPrintVarianceAccelerometer(int dps);
inline void sPrintVarianceMagnetometer(int dps);
inline void sPrintVarianceAltimeter(int dps);


inline void sPrintRollAndPitchQuaternion(int dps);
inline void sPrintRollAndPitchMeasuredQuaternion(int dps);
inline void sPrintRollAndPitchPredictedQuaternion(int dps);
inline void sPrintRollAndPitchPMatrix(int dps);
inline void sPrintRollAndPitchPNext(int dps);
inline void sPrintRollAndPitchKalmanGain(int dps);
inline void sPrintRollAndPitchAMatrix(int dps);

inline void sPrintRollAndPitchEulerAngles(int dps);

SensorFusion sensor_fusion;


#endif // MAIN_H
