// SensorFusion.h
//
// This is the header file for the program 'SensorFusion.cpp'
// it takes raw data from various sensors ans processes ths data
// in various Kalman filter stages

#ifndef SENSORFUSION_H
#define SENSORFUSION_H

#include "Arduino.h"
#include <Wire.h>

#include <RollAndPitchKalmanFilter.h>	    
#include <NavigationalFrameKalmanFilter.h>

#include <Accelerometer.h>
#include <FIMU_ITG3200.h>
#include <Magnetometer.h>
#include <MPL3115A2.h>

#include <SensorInitialization.h>

// DEFINES - defines the bit corresponding to the bit
// in the variable 'interrupt_byte'
// if new data is available (checked in the function
// 'checkInterrupts') the corresponding bit in this variable is 
// made '1'. after the data is read, this should be returned to
// '0'

#define GYROSCOPE_INTERRUPT_BYTE_BIT 		0x01
#define ACCELEROMETER_INTERRUPT_BYTE_BIT 	0x02
#define MAGNETOMETER_INTERRUPT_BYTE_BIT 	0x04
#define ALTIMETER_INTERRUPT_BYTE_BIT 		0x08

#define FIMU_ITG3200_DEF_ADDR 			ITG3200_ADDR_AD0_LOW // AD0 connected to GND
// structs representing the raw sensor data


struct GyroscopeRawData{
	float gyroscope_raw_x = 0.0f;
	float gyroscope_raw_y = 0.0f;
	float gyroscope_raw_z = 0.0f;
};

struct AccelerometerRawData{
	float accelerometer_raw_x = 0.0f;
	float accelerometer_raw_y = 0.0f;
	float accelerometer_raw_z = 0.0f;
};

struct MagnetometerRawData{
	float magnetometer_raw_x = 0.0f;
	float magnetometer_raw_y = 0.0f;
	float magnetometer_raw_z = 0.0f;
};

struct AltimeterRawData{
	float altimeter_raw_x = 0.0f;
	float altimeter_raw_y = 0.0f;
	float altimeter_raw_z = 0.0f;
};

struct GPSRawData{
	float GPS_raw_x = 0.0f;
	float GPS_raw_y = 0.0f;
	float GPS_raw_z = 0.0f;
};

/*-----------------------------------------------------------------------\
 * 
 * 			structs --SensorInitializationData
 *
 *-----------------------------------------------------------------------*/


// structs representing the initialization data of the raw sensors
// the initialization class computes the rolling means and standard
// deviations as well as the mean sample period 

struct AccelerometerInitializationData{
	SensorInitialization rolling_x = SensorInitialization();
	SensorInitialization rolling_y = SensorInitialization();
	SensorInitialization rolling_z = SensorInitialization();
};

struct GyroscopeInitializationData{
	SensorInitialization rolling_x = SensorInitialization();
	SensorInitialization rolling_y = SensorInitialization();
	SensorInitialization rolling_z = SensorInitialization();
};

struct MagnetometerInitializationData{
	SensorInitialization rolling_x = SensorInitialization();
	SensorInitialization rolling_y = SensorInitialization();
	SensorInitialization rolling_z = SensorInitialization();
};

struct AltimeterInitializationData{
	SensorInitialization rolling_z = SensorInitialization();
};

struct GPSInitializationData{
	SensorInitialization rolling_x = SensorInitialization();
	SensorInitialization rolling_y = SensorInitialization();
	SensorInitialization rolling_z = SensorInitialization();
};

// struct that stores the sensor characteristics following the 
// initialization period determined by the 'SensorInitialization' classes
// listed in the structs above
struct SensorCharacteristics{
	float variance_x    = 0.0f, //
	      variance_y    = 0.0f, // <-- variances 
	      variance_z    = 0.0f, //
                            
	      mean_x        = 0.0f, //
	      mean_y        = 0.0f, // <-- mean
      	      mean_z        = 0.0f, //

	      sample_period = 0.0f; // <-- sample period
};

/*-----------------------------------------------------------------------\
 * 
 * 			orientation structs
 *
 *-----------------------------------------------------------------------*/

struct EulerAngles{
	float pitch = 0.0f,
	      roll  = 0.0f,
	      yaw   = 0.0f;
};

struct Quaternion{
	float q0  = 1.0f,
	      q1  = 0.0f,
	      q2  = 0.0f,
	      q3  = 0.0f;
};

/*-----------------------------------------------------------------------\
 * 
 * 			main class section	
 *
 *-----------------------------------------------------------------------*/

class SensorFusion{
	public:
		SensorFusion();
		        void initialize();
			void  update();

			float getRawGyroscopeX(); 	
			float getRawGyroscopeY();
			float getRawGyroscopeZ();

			float getRawAccelerometerX(); 	
			float getRawAccelerometerY();
			float getRawAccelerometerZ();

			float getRawMagnetometerX(); 	
			float getRawMagnetometerY();
			float getRawMagnetometerZ();

			float getRawAltimeterZ();

			float getMeanGyroscopeX(); 	
			float getMeanGyroscopeY();
			float getMeanGyroscopeZ();

			float getMeanAccelerometerX(); 	
			float getMeanAccelerometerY();
			float getMeanAccelerometerZ();

			float getMeanMagnetometerX(); 	
			float getMeanMagnetometerY();
			float getMeanMagnetometerZ();

			float getMeanAltimeterZ();

			float getVarianceGyroscopeX(); 	
			float getVarianceGyroscopeY();
			float getVarianceGyroscopeZ();

			float getVarianceAccelerometerX(); 	
			float getVarianceAccelerometerY();
			float getVarianceAccelerometerZ();

			float getVarianceMagnetometerX(); 	
			float getVarianceMagnetometerY();
			float getVarianceMagnetometerZ();

			float getVarianceAltimeterZ();

			float getSamplePeriodGyroscope();
			float getSamplePeriodAccelerometer();
			float getSamplePeriodMagnetometer();
			
			float getRollAndPitchQ0();
			float getRollAndPitchQ1();
			float getRollAndPitchQ2();
			float getRollAndPitchQ3();

			float getRollAndPitchMeasuredQ0();
			float getRollAndPitchMeasuredQ1();
			float getRollAndPitchMeasuredQ2();
			float getRollAndPitchMeasuredQ3();
			                                  
			float getRollAndPitchPredictedQ0();
			float getRollAndPitchPredictedQ1();
			float getRollAndPitchPredictedQ2();
			float getRollAndPitchPredictedQ3();


			void  setIsSensorInitializationPeriodFinished(bool nISIPF);

                       RollAndPitchKalmanFilter	     roll_and_pitch_kalman_filter    ; 
		       NavigationalFrameKalmanFilter navigational_frame_kalman_filter ;

	private: 


		inline void initializeSensors(); 
		inline void instantiateSensors();
		inline void defineSensorParameters();
		inline void findSensorCharacteristics();
		       void evaluateInitializationData();
		inline void evaluateGyroscopeInitializationData();
		inline void evaluateAccelerometerInitializationData();
		inline void evaluateMagnetometerInitializationData();
		inline void evaluateAltimeterInitializationData();
		inline void evaluateGPSInitializationData();

		// initialize interrupts 
		       void defineInterruptSettings();
		inline void defineGyroscopeInterruptSettings();
		inline void defineAccelerometerInterruptSettings();
		inline void defineMagnetometerInterruptSettings();
		inline void defineAltimeterInterruptSettings();

		inline void createInitializationDataObjects();
		inline void initializationLoop();
		inline void updateInitializationData();
		inline void updateGyroscopeInitializationData();
		inline void updateAccelerometerInitializationData();
		inline void updateMagnetometerInitializationData();
		inline void updateAltimeterInitializationData();
		       void checkInterrupts();
		       void updateCurrentSensorData();
		       void takeGyroscopeSamples();
		       void takeAccelerometerSamples();
		       void takeMagnetometerSamples();
		       void takeAltimeterSamples();

		       void initializeKalmanFilters(); 
		inline void createKalmanFilterClassObjects();
		inline void defineKalmanFilterParameters();

		       void updateKalmanFilters();
		inline void updateRollAndPitchKalmanFilterSamples();
		inline void updateRollAndPitchKalmanFilterGyroscopeSamples();
		inline void updateRollAndPitchKalmanFilterAccelerometerSamples();
		       void extractRollAndPitchKalmanFilterParameters();
	        inline void extractRollAndPitchQuaternion();	
	        inline void extractRollAndPitchPredictedQuaternion();
	        inline void extractRollAndPitchMeasuredQuaternion();


		       
		       uint8_t		interrupt_byte 			  	 = 0x00;

		       bool		is_sensor_initialization_period_finished = false;	

		       GyroscopeRawData 	 gyroscope_raw_data 	   =  GyroscopeRawData();
		       AccelerometerRawData 	 accelerometer_raw_data    =  AccelerometerRawData();
		       MagnetometerRawData 	 magnetometer_raw_data 	   =  MagnetometerRawData();
		       AltimeterRawData 	 altimeter_raw_data 	   =  AltimeterRawData();
		       GPSRawData 	 	 gps_raw_data 		   =   GPSRawData();

		       Quaternion		 roll_and_pitch_quaternion = Quaternion();
		       Quaternion		 roll_and_pitch_measured_quaternion = Quaternion();
		       Quaternion		 roll_and_pitch_predicted_quaternion = Quaternion();

		       GyroscopeInitializationData 	gyroscope_initialization_data;
		       AccelerometerInitializationData  accelerometer_initialization_data;
		       MagnetometerInitializationData   magnetometer_initialization_data;
		       AltimeterInitializationData      altimeter_initialization_data;
		       GPSInitializationData      	gps_initialization_data;

		       SensorCharacteristics  gyroscope_sensor_characteristics 	   =  SensorCharacteristics();
		       SensorCharacteristics  accelerometer_sensor_characteristics =  SensorCharacteristics();
		       SensorCharacteristics  magnetometer_sensor_characteristics  =  SensorCharacteristics();
		       SensorCharacteristics  altimeter_sensor_characteristics 	   =  SensorCharacteristics();
		       SensorCharacteristics  gps_sensor_characteristics 	   =  SensorCharacteristics();





		       ITG3200 		gyroscope;
		       Accelerometer 	accelerometer;
		       Magnetometer 	magnetometer;
		       MPL3115A2	altimeter;


};

#endif //SENSORFUSION_H

// END OF FILE (SensorFusion.h)
