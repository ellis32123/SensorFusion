// RollAndPitchKalmanFilter.h

// this program defines the header file for the program
// 'RollAndPitchKalmanFilter.cpp' 


#ifndef ROLLANDPITCHKALMANFILTER_H
#define ROLLANDPITCHKALMANFILTER_H

#include "Arduino.h" 
	
#define ALPHA 					0.1f	// for initial predicted covariance
#define BETA					10.0f   // for dynmaic performance (ASGD)

/*****************************************************************************
			MAIN CLASS DEFINITION
*****************************************************************************/

class RollAndPitchKalmanFilter{

	public:
			     RollAndPitchKalmanFilter();
			void initializeModelParameters(float vGX, float vGY, float vGZ,
							float vAX, float vAY, float vAZ,
							 float gSP, float aSP );	

			void update();

			float getQ0();
			float getQ1();
			float getQ2();
			float getQ3();

			float getQMeasured0();
			float getQMeasured1();
			float getQMeasured2();
			float getQMeasured3();

			float getQDot0();
			float getQDot1();
			float getQDot2();
			float getQDot3();

			float getK0();
			float getK1();
			float getK2();
			float getK3();

			float getPNext0();
			float getPNext1();
			float getPNext2();
			float getPNext3();

			float getP0();
			float getP1();
			float getP2();
			float getP3();

			float getA00();
			float getA01();
                        float getA02();
			float getA03();
			float getA10();
			float getA11();
			float getA12();
                        float getA13();
			float getA20();
			float getA21();
			float getA22();
			float getA23();
			float getA30();			
			float getA31();
			float getA32();
			float getA33();

			void setGyroscopeVarianceX(float nGV);	  
			void setAccelerometerVarianceX(float nAV);		
			void setGyroscopeVarianceY(float nGV);	  
			void setAccelerometerVarianceY(float nAV);		
			void setGyroscopeVarianceZ(float nGV);	  
			void setAccelerometerVarianceZ(float nAV);		

			void setGyroscopeSamplePeriod(float nGSP);    	
			void setAccelerometerSamplePeriod(float nASP);

			void setGyroscopeSampleX(float nGS);	
			void setAccelerometerSampleX(float nAS);
			void setGyroscopeSampleY(float nGS);	
			void setAccelerometerSampleY(float nAS);
			void setGyroscopeSampleZ(float nGS);	
			void setAccelerometerSampleZ(float nAS);

	private:
		inline void initializeModelParameters();		
		inline void defineQMatrix();
		inline void defineRMatrix();
		inline void definePMatrix();
		inline void initializeQuaternions();
		inline void initializeAMatrix();

		       void updateTimeData();
		       void findGyroscopeSampleSize();
		       void updateStepSize();
		       void determineAMatrix();
		inline void determineAMatrixElementValues();
		inline void populateAMatrix(float xp, float xn,
					    float yp, float yn,
				            float zp, float zn);
		       void predictNextQuaternion();
		       void findNextPredictionCovariance();
		       void findKalmanGain();
		       void updateJacobianMatrix();
		       void updateDirectionCosineMatrix();
		       void updateObjectiveErrorFunction();
		       void findGradOEF();
		       void findAccelerometerSampleSize();
		       void computeMeasurementQuaternion();
                inline void computeMeasurementQuaternion(float ax, float ay, float az);
		       void updateMeasurementQuaternionWithASGD();
		       void updateQuaternion();
		       void updatePMatrix();

		inline void normalizeQuaternion(float *quaternion);

		       float inverseSquareRoot(float sum_of_squared_quaternion_elements);

			float sample_period_current,
			      accelerometer_sample_period,
			      gyroscope_sample_period,

			      accelerometer_varianceX,
			      gyroscope_varianceX,

			      accelerometer_varianceY,
			      gyroscope_varianceY,

			      accelerometer_varianceZ,
			      gyroscope_varianceZ,

			      accelerometer_sample_x,
			      gyroscope_sample_x, 

			      accelerometer_sample_y,
			      gyroscope_sample_y, 

			      accelerometer_sample_z,
			      gyroscope_sample_z,

			      accelerometer_sample_size,
			      gyroscope_sample_size,
			      step_size,
			      OEF[3],
			      grad_OEF[4],
			      DCM[3],
			      Q    				= 0.0f,
			      R 				= 0.0f, 
			      K[4],
			      P[4],
			      P_next[4],
			      q[4],
			      q_dot[4],
			      q_measured[4],
			      A[4][4],
			      J[3][4];

			long  current_millis = 0,
			      previous_millis = 0;
};

#endif // ROLLANDPITCHKALMANFILTER_H

// END OF FILE -- RollAndPitchKalmanFilter.h
