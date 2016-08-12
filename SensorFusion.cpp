// SensorFusion.cpp

// this program initializes all the sensors' raw values 
// and determines their characteristics such as mean 
// standard deviation and sample frequency 
// 
// following this initial stage the samples are passed through several 
// filtration stages 
// 1) roll and pitch estimatation --> using a gyro-accelerometer based 
// 					Kalman filter with Quaternions
// 2) yaw compensation 	   	  --> using a magnetometer,
// 3) n-frame states estimation   --> using altimeter/GPS samples
//











//			 REQUIREMENTS
//
// 			INPUTS
//
// gyroscope samples
// accelerometer samples
// magnetometer samples
// altimeter samplea
// gps samples
//
// 			OUTPUTS
//
// serial samples for external processing
// serial filtered states
// filtered states to parent class (Vehicle onboard controller class)
// 
//
// 			INITIALIZATION TIMELINE
//
// PHASE 1
// basic data of all samples
//
// PHASE 2
// basic data of r+p quaternion
//
// PHASE 3
// basic data of navigational acceleration and its integrated states
// basic data of altimeter and gps derivative states

////////////////////////////////////////////////////////////////////////
//

#include "SensorFusion.h"

SensorFusion::SensorFusion(){
//	initializeSensors();
//	initializeKalmanFilters();
}


/*********************************************************************************
				INITIALIZE SENSORS 
*********************************************************************************/

void SensorFusion::initialize(){
	initializeSensors();
	initializeKalmanFilters();
}

// initialize all sensors and their parameters
// called by public function SensorFusion()
// all functions called here are inline
void SensorFusion::initializeSensors(){
	instantiateSensors();   	// create sensor objects

	defineSensorParameters();    // set sensor settings (eg output rate)

	findSensorCharacteristics(); // determine sd/mean/T

}

// create sensor objects
// called by private function initializeSensors()
// which is called by the public function
// and class instantiator SensorFusion();
inline void SensorFusion::instantiateSensors(){
	gyroscope 	= ITG3200();
	accelerometer 	= Accelerometer();
	magnetometer	= Magnetometer();
	altimeter	= MPL3115A2();
} 

// set individual addresses and bits on all sensors
// called by private function initializeSensors()
// which is called by the public function
// and class instantiator SensorFusion();
inline void SensorFusion::defineSensorParameters(){
	// initialize gyroscope sensor
	gyroscope.init(FIMU_ITG3200_DEF_ADDR); // set I2C address of gyro
	// initialize accelerometer sensor
	accelerometer.initialize();
	// initialize magnetometer sensor
	magnetometer.initialize();
	// initialize altimeter sensor
	altimeter.begin();

	// TODO 
	// GPS
	// DISTANCE SENSOR
	//
	// set all sensor interrupt addresses
	defineInterruptSettings();

}

/*********************************************************************************
			DEFINE INTERRUPT SETTINGS	
*********************************************************************************/


// function sets all the bits for each sensor's 
// interrupt registers
// called by the private function
// defineSensorParameters() which is called by 
// the public function SensorFusion().
void SensorFusion::defineInterruptSettings(){
	defineGyroscopeInterruptSettings();
	defineAccelerometerInterruptSettings();
	defineMagnetometerInterruptSettings();
	defineAltimeterInterruptSettings();
}

inline void SensorFusion::defineGyroscopeInterruptSettings(){
	gyroscope.setINTLogiclvl(0);        // 1 = ACTIVE_ONLOW, 	0 = ACTIVE_ONHIGH
	gyroscope.setINTDriveType(0);	    // 1 = OPEN_DRAIN , 	0 = PUSH_PULL
	gyroscope.setLatchMode(0);	    // 1 = UNTIL_INT_CLEARED, 	0 = PULSE_50US
	gyroscope.setLatchClearMode(0);     // 1 = READ_ANYREG, 	0 = READ_STATUSREG
	gyroscope.setITGReady(true);           // Enable interrupt when device is ready
	gyroscope.setRawDataReady(true);       // Enable interrupt when data is available
} 

inline void SensorFusion::defineAccelerometerInterruptSettings(){
	accelerometer.setIsDataReadyInterruptOnINT1Pin(true); 
	accelerometer.setDataReadyInterrupt(true);   		// enable data ready interrupt

}

inline void SensorFusion::defineMagnetometerInterruptSettings(){

}

inline void SensorFusion::defineAltimeterInterruptSettings(){

}

/*********************************************************************************
				SENSOR INITIALIZATION
*********************************************************************************/



// function uses the 'SensorInitialization' class to compute the 
// standard devation
// mean
// sample period 
// of the sensors
// the function continues to update the parameters
// until the boolean 'is_sensor_initialization_period_finished'
// passes true.
inline void SensorFusion::findSensorCharacteristics(){
	// inline function creating all the initialization 
	// data objects for each sensor
	createInitializationDataObjects(); 	
	initializationLoop();
	evaluateInitializationData();
}

// inline function creating all the initialization 
// data objects for each sensor
// called in the private 'findSensorCharacteristics()' function
inline void SensorFusion::createInitializationDataObjects(){
	accelerometer_initialization_data 	= AccelerometerInitializationData();
	gyroscope_initialization_data 		= GyroscopeInitializationData();
	magnetometer_initialization_data 	= MagnetometerInitializationData();
	altimeter_initialization_data 		= AltimeterInitializationData();
	gps_initialization_data 		= GPSInitializationData();
}

// main loop in the initialization stage
// continues to loop until the boolean 
// is_sensor_initialization_period_finished
// becomes true
// this is made true externally. ie an 
// external source/class analyses the data
// and determines the termination time
// of the initialization period
inline void SensorFusion::initializationLoop(){
	int i = 0; ///TEMPORARY TEST --- DELETE!!!
	while(!is_sensor_initialization_period_finished){
		// checks to see if new sensor data is available.
		//  if so these are stored in the byte 
		//  'interrupt_byte' in which when the
		//  bit corresponding to a given sensor is '1'
		//  new data is available
		//  after the data is read, this bit should be returned
		//  to zero 
		checkInterrupts(); 			
		// if the corresponding bit in 'interrupt_byte'
		// is '1' then new sensor data is available
		// the function checks this bit and if true,
		// updates the current sample for each sensor
		updateCurrentSensorData();
		updateInitializationData();
		i++;
		/////////////////////// TEST DELETE -- EVENTUALLY CHANGE SO THE BOOL CAN BE CHANGED EXTERNALLY !!!!!!!!!!!!!!!!!!
		if(i==1000)
			is_sensor_initialization_period_finished = true;
	}
}


// this function is called in the inline function 'initializationLoop()'
// the initialization loop is runs continuously until the boolean
// 'is_sensor_initialization_period_finished' is defined true 
// this is usually done using an external class
inline void SensorFusion::updateInitializationData(){
	// If new gyroscope data available
	if((interrupt_byte >> GYROSCOPE_INTERRUPT_BYTE_BIT) & 1)
		updateGyroscopeInitializationData();
	// If new accelerometer data available
	if((interrupt_byte >> ACCELEROMETER_INTERRUPT_BYTE_BIT) & 1)
		updateAccelerometerInitializationData();
	// If new Magnetometer data available
	if((interrupt_byte >> MAGNETOMETER_INTERRUPT_BYTE_BIT) & 1)
		updateMagnetometerInitializationData();
	// If new Altimeter data available
	if((interrupt_byte >> ALTIMETER_INTERRUPT_BYTE_BIT) & 1)
		updateAltimeterInitializationData();
}

inline void SensorFusion::updateGyroscopeInitializationData(){
	gyroscope_initialization_data.rolling_x.setNewSample(gyroscope_raw_data.gyroscope_raw_x);
	gyroscope_initialization_data.rolling_y.setNewSample(gyroscope_raw_data.gyroscope_raw_y);
	gyroscope_initialization_data.rolling_z.setNewSample(gyroscope_raw_data.gyroscope_raw_z);
}

inline void SensorFusion::updateAccelerometerInitializationData(){
	accelerometer_initialization_data.rolling_x.setNewSample(accelerometer_raw_data.accelerometer_raw_x);		
	accelerometer_initialization_data.rolling_y.setNewSample(accelerometer_raw_data.accelerometer_raw_y);
	accelerometer_initialization_data.rolling_z.setNewSample(accelerometer_raw_data.accelerometer_raw_z);
}

inline void SensorFusion::updateMagnetometerInitializationData(){
	magnetometer_initialization_data.rolling_x.setNewSample(magnetometer_raw_data.magnetometer_raw_x);		
	magnetometer_initialization_data.rolling_y.setNewSample(magnetometer_raw_data.magnetometer_raw_y);
	magnetometer_initialization_data.rolling_z.setNewSample(magnetometer_raw_data.magnetometer_raw_z);
}

inline void SensorFusion::updateAltimeterInitializationData(){
	altimeter_initialization_data.rolling_z.setNewSample(altimeter_raw_data.altimeter_raw_z);
}

/*********************************************************************************
			EVALUATE INITIALIZATION DATA	
*********************************************************************************/

// the functions in this subsection extract the mean and variance from 
// the 'Sensor_Initialization' class objects for each sensor

// This function is called by the 'findSensorCharacteristics()'
// function (which is called called by the public function
// 'sensorfusion()' function upon creation of the class object)
//
// The function calls various inline functions relating to // each sensor
void SensorFusion::evaluateInitializationData(){
	evaluateGyroscopeInitializationData();
	evaluateAccelerometerInitializationData();
	evaluateMagnetometerInitializationData();
	evaluateAltimeterInitializationData();
	evaluateGPSInitializationData();
}

inline void SensorFusion::evaluateGyroscopeInitializationData(){
	gyroscope_sensor_characteristics.mean_x = gyroscope_initialization_data.rolling_x.getMean();
	gyroscope_sensor_characteristics.mean_y = gyroscope_initialization_data.rolling_y.getMean();
	gyroscope_sensor_characteristics.mean_z = gyroscope_initialization_data.rolling_z.getMean();

	gyroscope_sensor_characteristics.variance_x =   gyroscope_initialization_data.rolling_x.getVariance();
	gyroscope_sensor_characteristics.variance_y =   gyroscope_initialization_data.rolling_y.getVariance();
	gyroscope_sensor_characteristics.variance_z =   gyroscope_initialization_data.rolling_z.getVariance();

	gyroscope_sensor_characteristics.sample_period = gyroscope_initialization_data.rolling_x.getSamplePeriodsMean();
}

inline void SensorFusion::evaluateAccelerometerInitializationData(){
	accelerometer_sensor_characteristics.mean_x = accelerometer_initialization_data.rolling_x.getMean();
	accelerometer_sensor_characteristics.mean_y = accelerometer_initialization_data.rolling_y.getMean();
	accelerometer_sensor_characteristics.mean_z = accelerometer_initialization_data.rolling_z.getMean();
                                                  
	accelerometer_sensor_characteristics.variance_x =   accelerometer_initialization_data.rolling_x.getVariance();
	accelerometer_sensor_characteristics.variance_y =   accelerometer_initialization_data.rolling_y.getVariance();
	accelerometer_sensor_characteristics.variance_z =   accelerometer_initialization_data.rolling_z.getVariance();

	accelerometer_sensor_characteristics.sample_period = accelerometer_initialization_data.rolling_x.getSamplePeriodsMean();
}

inline void SensorFusion::evaluateMagnetometerInitializationData(){
	magnetometer_sensor_characteristics.mean_x = magnetometer_initialization_data.rolling_x.getMean();
	magnetometer_sensor_characteristics.mean_y = magnetometer_initialization_data.rolling_y.getMean();
	magnetometer_sensor_characteristics.mean_z = magnetometer_initialization_data.rolling_z.getMean();
                                                  
	magnetometer_sensor_characteristics.variance_x =   magnetometer_initialization_data.rolling_x.getVariance();
	magnetometer_sensor_characteristics.variance_y =   magnetometer_initialization_data.rolling_y.getVariance();
	magnetometer_sensor_characteristics.variance_z =   magnetometer_initialization_data.rolling_z.getVariance();

	magnetometer_sensor_characteristics.sample_period = magnetometer_initialization_data.rolling_x.getSamplePeriodsMean();
}

inline void SensorFusion::evaluateAltimeterInitializationData(){
	altimeter_sensor_characteristics.mean_z =   altimeter_initialization_data.rolling_z.getMean();
	altimeter_sensor_characteristics.variance_z   =   altimeter_initialization_data.rolling_z.getVariance();

	altimeter_sensor_characteristics.sample_period = altimeter_initialization_data.rolling_z.getSamplePeriodsMean();
}

inline void SensorFusion::evaluateGPSInitializationData(){
	gps_sensor_characteristics.mean_x = gps_initialization_data.rolling_x.getMean();
	gps_sensor_characteristics.mean_y = gps_initialization_data.rolling_y.getMean();
	gps_sensor_characteristics.mean_z = gps_initialization_data.rolling_z.getMean();
                                                  
	gps_sensor_characteristics.variance_x =   gps_initialization_data.rolling_x.getVariance();
	gps_sensor_characteristics.variance_y =   gps_initialization_data.rolling_y.getVariance();
	gps_sensor_characteristics.variance_z =   gps_initialization_data.rolling_z.getVariance();

	gps_sensor_characteristics.sample_period = gps_initialization_data.rolling_x.getSamplePeriodsMean();
}

/*********************************************************************************
				INTERRUPTS	
*********************************************************************************/


// checks all the registered sensors' interupt pins
// to see if new data is available
// if a given sensor has new data available then
// the corresponding bit is made 1.
// the corresponding bit is determined by
// the constants of the form:
// SENSOR_INTERRUPT_BYTE_BIT
//
// this function is called in the 'initializationLoop()' 
// function and the 'update()' function below
void SensorFusion::checkInterrupts(){
	interrupt_byte = 0x00;			// reset interrupt byte from last loop
	interrupt_byte |= (gyroscope.isRawDataReady()     << GYROSCOPE_INTERRUPT_BYTE_BIT);// & 1;
	interrupt_byte |= (accelerometer.getIsDataReady() << ACCELEROMETER_INTERRUPT_BYTE_BIT);// & 1;
	//	TODO
}

inline void SensorFusion::updateCurrentSensorData(){
	// If new gyroscope data available
	if((interrupt_byte >> GYROSCOPE_INTERRUPT_BYTE_BIT) & 1)
		takeGyroscopeSamples();
	// If new accelerometer data available
	if((interrupt_byte >> ACCELEROMETER_INTERRUPT_BYTE_BIT) & 1)
		takeAccelerometerSamples();
	// If new Magnetometer data available
	if((interrupt_byte >> MAGNETOMETER_INTERRUPT_BYTE_BIT) & 1)
		takeMagnetometerSamples();
	// If new Altimeter data available
	if((interrupt_byte >> ALTIMETER_INTERRUPT_BYTE_BIT) & 1){
		takeAltimeterSamples();		
	}
}



/*********************************************************************************
			TAKE RAW SENSOR SAMPLES	
*********************************************************************************/
// these funstions are called in the inline function 'updateCurrentSensorData()'
// which is defined above int the 'INTERRUPTS' subsection
 

void SensorFusion::takeGyroscopeSamples(){
	float gyroVals[3],
	      to_radians = PI/180.0f;
	gyroscope.readGyro(&gyroVals[0],&gyroVals[1],&gyroVals[2]);
//	delay(10);
	gyroscope_raw_data.gyroscope_raw_x = gyroVals[0]*to_radians;
	gyroscope_raw_data.gyroscope_raw_y = gyroVals[1]*to_radians;
	gyroscope_raw_data.gyroscope_raw_z = gyroVals[2]*to_radians;
}

void SensorFusion::takeAccelerometerSamples(){
	accelerometer.readGValues();
//	delay(10);
	accelerometer_raw_data.accelerometer_raw_x = accelerometer.g_values.g_x;
	accelerometer_raw_data.accelerometer_raw_y = accelerometer.g_values.g_y;
	accelerometer_raw_data.accelerometer_raw_z = accelerometer.g_values.g_z;
}

void SensorFusion::takeMagnetometerSamples(){

}

void SensorFusion::takeAltimeterSamples(){

}

/*********************************************************************************
			INITIALIZE KALMAN FILTERS	
*********************************************************************************/


// called by the public void 'SensorFusion()' function 
// creates sensor fusion kalman filter objects and
// initialized their core parameters such as
// variance matrices and sample periods
void SensorFusion::initializeKalmanFilters(){
	createKalmanFilterClassObjects();	
	defineKalmanFilterParameters();	
}

// this function creates the instances of all kalman filter classes
// called by the private void 'initializeKalmanFilters()' function 
inline void SensorFusion::createKalmanFilterClassObjects(){
	roll_and_pitch_kalman_filter     = RollAndPitchKalmanFilter();
	navigational_frame_kalman_filter = NavigationalFrameKalmanFilter();
}

// this function defines the kalman filter parameters such as 
// variance and sample period
// called by the private void 'initializeKalmanFilters()' function 
inline void SensorFusion::defineKalmanFilterParameters(){
	// roll and pitch kalman filter
	roll_and_pitch_kalman_filter.initializeModelParameters(gyroscope_sensor_characteristics.variance_x,	
							       gyroscope_sensor_characteristics.variance_y,
							       gyroscope_sensor_characteristics.variance_z,
							       accelerometer_sensor_characteristics.variance_x,
							       accelerometer_sensor_characteristics.variance_y,
							       accelerometer_sensor_characteristics.variance_z,
							       gyroscope_sensor_characteristics.sample_period,
							       accelerometer_sensor_characteristics.sample_period);

}

/*********************************************************************************
				MAIN LOOP OF PROGRAM				
*********************************************************************************/

// this public function is called externally,
// the function updates all the external sensor filtration classes
// 
// all timings are determined internally,
// therefore no delay functions are required in the
// external class to ensure the sensors loop at a certain
// time.
void SensorFusion::update(){
	// checks to see if new sensor data is available.
	//  if so these are stored in the byte 
	//  'interrupt_byte' in which when the
	//  bit corresponding to a given sensor is '1'
	//  new data is available
	//  after the data is read, this bit should be returned
	//  to zero 
	checkInterrupts(); 			
	// if the corresponding bit in 'interrupt_byte'
	// is '1' then new sensor data is available
	// the function checks this bit and if true,
	// updates the current sample for each sensor
	updateCurrentSensorData();
	//function removes bias from sensor readings
	// bias values are determined in the initialization
	// phase assuming that the vehicle is kept level and stationary
	// offset value = mean when stationary and level
	removeSensorOffsets();
	// function updates all kalman filters based on what 
	// new sensor data is available. 
	updateKalmanFilters();
	// checks to see if new gyroscope data is available


}

/*********************************************************************************
			REMOVE SENSOR OFFSETS		
*********************************************************************************/

//function removes bias from sensor readings
// bias values are determined in the initialization
// phase assuming that the vehicle is kept level and stationary
// offset value = mean when stationary and level
void SensorFusion::removeSensorOffsets(){
	removeGyroscopeOffsets();
	removeAccelerometerOffsets();
	removeMagnetometerOffsets();
	removeAltimeterOffsets();
}

inline void SensorFusion::removeGyroscopeOffsets(){
	gyroscope_raw_data.gyroscope_raw_x -= gyroscope_sensor_characteristics.mean_x;
	gyroscope_raw_data.gyroscope_raw_y -= gyroscope_sensor_characteristics.mean_y;
	gyroscope_raw_data.gyroscope_raw_z -= gyroscope_sensor_characteristics.mean_z;
}

inline void SensorFusion::removeAccelerometerOffsets(){
	accelerometer_raw_data.accelerometer_raw_x -= accelerometer_sensor_characteristics.mean_x;
	accelerometer_raw_data.accelerometer_raw_y -= accelerometer_sensor_characteristics.mean_y;
	accelerometer_raw_data.accelerometer_raw_z -= accelerometer_sensor_characteristics.mean_z;
}

inline void SensorFusion::removeMagnetometerOffsets(){
	magnetometer_raw_data.magnetometer_raw_x -= magnetometer_sensor_characteristics.mean_x;
	magnetometer_raw_data.magnetometer_raw_y -= magnetometer_sensor_characteristics.mean_y;
	magnetometer_raw_data.magnetometer_raw_z -= magnetometer_sensor_characteristics.mean_z;
}

inline void SensorFusion::removeAltimeterOffsets(){
	altimeter_raw_data.altimeter_raw_z -= altimeter_sensor_characteristics.mean_z;
}

// function updates all kalman filters based on what 
// new sensor data is available. 
// called by the public void function 'update()'
void SensorFusion::updateKalmanFilters(){
	// If new gyroscope and acclerometer data available --> update roll + pitch KF
	if(((interrupt_byte >> GYROSCOPE_INTERRUPT_BYTE_BIT) & 1)  &&
           ((interrupt_byte >> ACCELEROMETER_INTERRUPT_BYTE_BIT) & 1)){
		// if new gyro and accelerometer samples
                //     -> the 'roll_and_pitch_kalman_filter' class is informed
		// and the gyro-accelerometer based quaternion state estimate is updated	
		updateRollAndPitchKalmanFilterSamples();
		roll_and_pitch_kalman_filter.update();
		extractRollAndPitchKalmanFilterParameters();
	}
	// If new accelerometer data available
	if((interrupt_byte >> ACCELEROMETER_INTERRUPT_BYTE_BIT) & 1){	
		// the 'navigational_frame_kalman_filter' class is informed
		// and the xyz axis state predictions are updated
		navigational_frame_kalman_filter.updateAccelerometerBasedStates();
	}

	// If new Magnetometer data available
	if((interrupt_byte >> MAGNETOMETER_INTERRUPT_BYTE_BIT) & 1)
		updateMagnetometerInitializationData();

	// If new Altimeter data available
	if((interrupt_byte >> ALTIMETER_INTERRUPT_BYTE_BIT) & 1)
		updateAltimeterInitializationData();

}

inline void SensorFusion::updateRollAndPitchKalmanFilterSamples(){
	updateRollAndPitchKalmanFilterGyroscopeSamples();
	updateRollAndPitchKalmanFilterAccelerometerSamples();
}

inline void SensorFusion::updateRollAndPitchKalmanFilterGyroscopeSamples(){
	roll_and_pitch_kalman_filter.setGyroscopeSampleX(gyroscope_raw_data.gyroscope_raw_x);
	roll_and_pitch_kalman_filter.setGyroscopeSampleY(gyroscope_raw_data.gyroscope_raw_y);
	roll_and_pitch_kalman_filter.setGyroscopeSampleZ(gyroscope_raw_data.gyroscope_raw_z);
}

inline void SensorFusion::updateRollAndPitchKalmanFilterAccelerometerSamples(){
	roll_and_pitch_kalman_filter.setAccelerometerSampleX(accelerometer_raw_data.accelerometer_raw_x);
	roll_and_pitch_kalman_filter.setAccelerometerSampleY(accelerometer_raw_data.accelerometer_raw_y);
	roll_and_pitch_kalman_filter.setAccelerometerSampleZ(accelerometer_raw_data.accelerometer_raw_z);
}

void SensorFusion::extractRollAndPitchKalmanFilterParameters(){
	extractRollAndPitchQuaternion();	
	extractRollAndPitchPredictedQuaternion();
	extractRollAndPitchMeasuredQuaternion();
}

inline void SensorFusion::extractRollAndPitchQuaternion(){
	roll_and_pitch_quaternion.q0 = roll_and_pitch_kalman_filter.getQ0();
	roll_and_pitch_quaternion.q1 = roll_and_pitch_kalman_filter.getQ1();
	roll_and_pitch_quaternion.q2 = roll_and_pitch_kalman_filter.getQ2();
	roll_and_pitch_quaternion.q3 = roll_and_pitch_kalman_filter.getQ3();
}

inline void SensorFusion::extractRollAndPitchPredictedQuaternion(){
	roll_and_pitch_predicted_quaternion.q0 = roll_and_pitch_kalman_filter.getQDot0();
	roll_and_pitch_predicted_quaternion.q1 = roll_and_pitch_kalman_filter.getQDot1();
	roll_and_pitch_predicted_quaternion.q2 = roll_and_pitch_kalman_filter.getQDot2();
	roll_and_pitch_predicted_quaternion.q3 = roll_and_pitch_kalman_filter.getQDot3();
}

inline void SensorFusion::extractRollAndPitchMeasuredQuaternion(){
	roll_and_pitch_measured_quaternion.q0 = roll_and_pitch_kalman_filter.getQMeasured0();
	roll_and_pitch_measured_quaternion.q1 = roll_and_pitch_kalman_filter.getQMeasured1();
	roll_and_pitch_measured_quaternion.q2 = roll_and_pitch_kalman_filter.getQMeasured2();
	roll_and_pitch_measured_quaternion.q3 = roll_and_pitch_kalman_filter.getQMeasured3();
}


/*********************************************************************************
				GETTERS
*********************************************************************************/

/* RAW GETTERS */
float SensorFusion::getRawGyroscopeX(){ return gyroscope_raw_data.gyroscope_raw_x;} 
float SensorFusion::getRawGyroscopeY(){ return gyroscope_raw_data.gyroscope_raw_y;} 
float SensorFusion::getRawGyroscopeZ(){ return gyroscope_raw_data.gyroscope_raw_z;} 

float SensorFusion::getRawAccelerometerX(){ return accelerometer_raw_data.accelerometer_raw_x;} 
float SensorFusion::getRawAccelerometerY(){ return accelerometer_raw_data.accelerometer_raw_y;} 
float SensorFusion::getRawAccelerometerZ(){ return accelerometer_raw_data.accelerometer_raw_z;} 

float SensorFusion::getRawMagnetometerX(){ return magnetometer_raw_data.magnetometer_raw_x;} 
float SensorFusion::getRawMagnetometerY(){ return magnetometer_raw_data.magnetometer_raw_y;} 
float SensorFusion::getRawMagnetometerZ(){ return magnetometer_raw_data.magnetometer_raw_z;} 

float SensorFusion::getRawAltimeterZ(){ return altimeter_raw_data.altimeter_raw_z;} 

/* MEAN GETTERS */
float SensorFusion::getMeanGyroscopeX(){ return  gyroscope_sensor_characteristics.mean_x;} 
float SensorFusion::getMeanGyroscopeY(){ return  gyroscope_sensor_characteristics.mean_y;} 
float SensorFusion::getMeanGyroscopeZ(){ return  gyroscope_sensor_characteristics.mean_z;} 

float SensorFusion::getMeanAccelerometerX(){ return  accelerometer_sensor_characteristics.mean_x;} 
float SensorFusion::getMeanAccelerometerY(){ return  accelerometer_sensor_characteristics.mean_y;} 
float SensorFusion::getMeanAccelerometerZ(){ return  accelerometer_sensor_characteristics.mean_z;} 

float SensorFusion::getMeanMagnetometerX(){ return magnetometer_sensor_characteristics.mean_x ;} 
float SensorFusion::getMeanMagnetometerY(){ return magnetometer_sensor_characteristics.mean_y ;} 
float SensorFusion::getMeanMagnetometerZ(){ return magnetometer_sensor_characteristics.mean_z ;} 

float SensorFusion::getMeanAltimeterZ(){ return altimeter_sensor_characteristics.mean_z ;}


/* VARIANCE GETTERS */
float SensorFusion::getVarianceGyroscopeX(){ return  gyroscope_sensor_characteristics.variance_x;} 
float SensorFusion::getVarianceGyroscopeY(){ return  gyroscope_sensor_characteristics.variance_y;} 
float SensorFusion::getVarianceGyroscopeZ(){ return  gyroscope_sensor_characteristics.variance_z;} 

float SensorFusion::getVarianceAccelerometerX(){ return  accelerometer_sensor_characteristics.variance_x;} 
float SensorFusion::getVarianceAccelerometerY(){ return  accelerometer_sensor_characteristics.variance_y;} 
float SensorFusion::getVarianceAccelerometerZ(){ return  accelerometer_sensor_characteristics.variance_z;} 

float SensorFusion::getVarianceMagnetometerX(){ return magnetometer_sensor_characteristics.variance_x ;} 
float SensorFusion::getVarianceMagnetometerY(){ return magnetometer_sensor_characteristics.variance_y ;} 
float SensorFusion::getVarianceMagnetometerZ(){ return magnetometer_sensor_characteristics.variance_z ;} 

float SensorFusion::getVarianceAltimeterZ(){ return altimeter_sensor_characteristics.variance_z ;} 

/* SAMPLE PERIOD GETTERS */
float SensorFusion::getSamplePeriodGyroscope()    { return gyroscope_sensor_characteristics.sample_period ;}  
float SensorFusion::getSamplePeriodAccelerometer(){ return accelerometer_sensor_characteristics.sample_period ;} 
float SensorFusion::getSamplePeriodMagnetometer() { return magnetometer_sensor_characteristics.sample_period ;} 


/* QUATERNION GETTERS */
float SensorFusion::getRollAndPitchQ0(){ return roll_and_pitch_quaternion.q0;}
float SensorFusion::getRollAndPitchQ1(){ return roll_and_pitch_quaternion.q1;}
float SensorFusion::getRollAndPitchQ2(){ return roll_and_pitch_quaternion.q2;}
float SensorFusion::getRollAndPitchQ3(){ return roll_and_pitch_quaternion.q3;}

float SensorFusion::getRollAndPitchMeasuredQ0(){ return roll_and_pitch_measured_quaternion.q0;}
float SensorFusion::getRollAndPitchMeasuredQ1(){ return roll_and_pitch_measured_quaternion.q1;}
float SensorFusion::getRollAndPitchMeasuredQ2(){ return roll_and_pitch_measured_quaternion.q2;}
float SensorFusion::getRollAndPitchMeasuredQ3(){ return roll_and_pitch_measured_quaternion.q3;}

float SensorFusion::getRollAndPitchPredictedQ0(){ return roll_and_pitch_predicted_quaternion.q0;}
float SensorFusion::getRollAndPitchPredictedQ1(){ return roll_and_pitch_predicted_quaternion.q1;}
float SensorFusion::getRollAndPitchPredictedQ2(){ return roll_and_pitch_predicted_quaternion.q2;}
float SensorFusion::getRollAndPitchPredictedQ3(){ return roll_and_pitch_predicted_quaternion.q3;}

/*********************************************************************************
			SETTERS
*********************************************************************************/

void SensorFusion::setIsSensorInitializationPeriodFinished(bool nISIPF){
	is_sensor_initialization_period_finished = nISIPF;
}

// END OF FILE (SensorFusion.cpp)
