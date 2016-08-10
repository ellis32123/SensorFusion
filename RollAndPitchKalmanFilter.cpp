// RollAndPitchKalmanFilter.cpp

// this program defines the kalman filter algorithm that
// combines accelerometer and gyroscope data in order
// determine orientation
// the 4d kalman filter is quaternion based
// it also includes an ASGD algorithm for improved dynamic performance 
//
// the gyroscope samples provide the predictiions 
// through integration -- leads to long term drift 
// therefore this must be compensated for in the long term
// using an accelerometer based quaternion


#include "RollAndPitchKalmanFilter.h"

RollAndPitchKalmanFilter::RollAndPitchKalmanFilter(){

}

/*******************************************************************************
			PARAMETER INITIALIZATION	
*******************************************************************************/

// this public function defines the 'Q' and 'R' matrices 
// based on the sensor variances and sample periods
// the function also initializes the other model parameters such
// as quaternions (predict, measured, output)
// and P/P_next/K matrices
void RollAndPitchKalmanFilter::initializeModelParameters(float vGX, float vGY, float vGZ,
							 float vAX, float vAY, float vAZ,
							 float gSP, float aSP){
	setGyroscopeVarianceX(vGX); 	
	setGyroscopeVarianceY(vGY); 
	setGyroscopeVarianceZ(vGZ); 

	setAccelerometerVarianceX(vAX); 
	setAccelerometerVarianceY(vAY); 
	setAccelerometerVarianceZ(vAZ); 

	setGyroscopeSamplePeriod(gSP);    
	setAccelerometerSamplePeriod(aSP);

	initializeModelParameters();
}			

// This function forms the 'Q' and 'R' matrices
// based on the sensor variances and sample periods
// this private function is called by the public function
// with the same name but which takes 8 argumens.
// these arguments are the variance and sample periods of the 
// gyroscope and accelerometer
inline void RollAndPitchKalmanFilter::initializeModelParameters(){
	defineQMatrix();
	defineRMatrix();
	definePMatrix();
	initializeQuaternions();
	initializeAMatrix();
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(" HELLOO !!!!!!!!!!!!!!");
	Serial.println(q[0]);
	Serial.println(q[1]);
	Serial.println(q[2]);
	Serial.println(q[3]);
	Serial.println(q_dot[0]);
	Serial.println(q_dot[1]);
	Serial.println(q_dot[2]);
	Serial.println(q_dot[3]);
	delay(1000);
}

// this function defines the process noise covariance matrix
// the matrix is diagonal with all non-zero elements
// being equal. therfore the matrix is summarised by the 
// single float 'Q' for computational efficiency
// this function is called by the inline private function 
// 'initializeModelParameters()' which is called by and
// equivalently named public function which takes
// the arguments of sensor variances and sample periods
inline void RollAndPitchKalmanFilter::defineQMatrix(){
	float time_multiplier = 0.25f*(gyroscope_sample_period*gyroscope_sample_period);
	Q = (gyroscope_varianceX + 
	    gyroscope_varianceY + 
	    gyroscope_varianceZ)*time_multiplier; 
}

inline void RollAndPitchKalmanFilter::defineRMatrix(){
	R = accelerometer_varianceX + 
	    accelerometer_varianceY + 
	    accelerometer_varianceZ;
}

inline void RollAndPitchKalmanFilter::definePMatrix(){
	P[0] = ALPHA;
	P[1] = ALPHA;
	P[2] = ALPHA;
	P[3] = ALPHA;
}

inline void RollAndPitchKalmanFilter::initializeQuaternions(){
	q[0] = 1.0f;
	q[1] = 0.0f;
	q[2] = 0.0f;
	q[3] = 0.0f;

	q_dot[0] = 1.0f;
	q_dot[1] = 0.0f;
	q_dot[2] = 0.0f;
	q_dot[3] = 0.0f;
}

// assume A matrix to be I-4x4 at the start
inline void RollAndPitchKalmanFilter::initializeAMatrix(){
	A[0][0] = 1.0f;
	A[1][1] = 1.0f;
	A[2][2] = 1.0f;
	A[3][3] = 1.0f;
}

/*******************************************************************************
			MAIN LOOPS - (Prediction)	
*******************************************************************************/


// this public function updates the state estimate when new 
// gyroscope and acclerometer data is available
void RollAndPitchKalmanFilter::update(){
	updateTimeData();
	findGyroscopeSampleSize();
	updateStepSize();
//	normalizeQuaternion(q);                  // normalize at start to ensure unit quaternion

	determineAMatrix();
	predictNextQuaternion();
	normalizeQuaternion(q_dot);
	findNextPredictionCovariance();
	findKalmanGain();
	findAccelerometerSampleSize();
	computeMeasurementQuaternion();
	updateJacobianMatrix();
	updateDirectionCosineMatrix();
	updateObjectiveErrorFunction();
	findGradOEF();
	updateMeasurementQuaternionWithASGD();
	normalizeQuaternion(q_measured);
	updateQuaternion();
	normalizeQuaternion(q);
	updatePMatrix();
}

// this function updates the time values (in milliseconds)
// as well as the sample period
// the function is called by the public void 'update()'
void RollAndPitchKalmanFilter::updateTimeData(){
	current_millis = millis();
	sample_period_current = (current_millis - previous_millis)/1000.0f;
	previous_millis = current_millis;
}

// this function determines the maginitude of the gyroscope samples
// the function is called by the public void 'update()'
void RollAndPitchKalmanFilter::findGyroscopeSampleSize(){
	gyroscope_sample_size = sqrt(gyroscope_sample_x*gyroscope_sample_x +	
	        		gyroscope_sample_y*gyroscope_sample_y +
				gyroscope_sample_z*gyroscope_sample_z );
}

// this function determines the maginitude of the accelerometer samples
// the function is called by the public void 'update()'
void RollAndPitchKalmanFilter::findAccelerometerSampleSize(){
	accelerometer_sample_size = sqrt(accelerometer_sample_x*accelerometer_sample_x +	
					 accelerometer_sample_y*accelerometer_sample_y +
					 accelerometer_sample_z*accelerometer_sample_z );
}


// this function updates the step size for the ASGD algorithm
// a large set of gyroscop samples (ie turning fast) suggests
// that the the vehicle is in a dynamic mode and therefore
// more 'trust' should be placed in state predictions lying distant
// from the expected value
// the function is called by the public void 'update()'
void RollAndPitchKalmanFilter::updateStepSize(){
	step_size = BETA*sample_period_current*gyroscope_sample_size;
}

// this function computes the state transition matrix
// based on the integration of gyroscope samples
// and a second order taylor series approximation of 
// the exponential function (for computational efficiency)
// notation:
//	xn = 'n-th' order term of x axis 
//	T2 = sample period squared
// the function is called by the public void
// 'updateGyroscopeBasedQuaternion()'
void RollAndPitchKalmanFilter::determineAMatrix(){
	determineAMatrixElementValues(); 
}

// this function finds the individual elements of the 
// state transition matrix which is based on the 
// integration of gyroscope samples
// and a second order taylor series approximation
// of the exponential function 
// notation:
//	xn = 'n-th' order term of x axis 
//	T2 = sample period squared
// the function is called by the private void
// 'determineAMatrix()'
inline void RollAndPitchKalmanFilter::determineAMatrixElementValues(){
	float T2 = gyroscope_sample_period*gyroscope_sample_period,
	      x2_and_0 = (gyroscope_sample_x*gyroscope_sample_x*T2*0.125f),// + 1.0f,
              x1       =  gyroscope_sample_x*gyroscope_sample_period*0.5f,
              xp       =  x2_and_0 + x1,
              xn       =  x2_and_0 - x1,
              y2_and_0 = (gyroscope_sample_y*gyroscope_sample_y*T2*0.125f),// + 1.0f,
              y1       =  gyroscope_sample_y*gyroscope_sample_period*0.5f,
              yp       =  y2_and_0 + y1,
              yn       =  y2_and_0 - y1,
              z2_and_0 = (gyroscope_sample_z*gyroscope_sample_z*T2*0.125f),// + 1.0f,
              z1       =  gyroscope_sample_z*gyroscope_sample_period*0.5f,
              zp       =  z2_and_0 + z1,
              zn       =  z2_and_0 - z1;
	populateAMatrix(xp,xn,yp,yn,zp,zn);
}

// this function takes elements determined in the inline function
// 'determineAMatrixElementValues()' and places them in the 
// correct elements of the A matrix
// the function is called by the private inline void
// 'determineAMatrixElementValues()'
inline void RollAndPitchKalmanFilter::populateAMatrix(float xp, float xn,
						      float yp, float yn,
						      float zp, float zn){
    /*A[0][0] = 1;*/  A[0][1] = xn;    A[0][2] = yn;    A[0][3] = zn;
      A[1][0] = xp; /*A[1][1] = 1;*/   A[1][2] = zp;    A[1][3] = yn;
      A[2][0] = yp;   A[2][1] = zn;  /*A[2][2] = 1;*/   A[2][3] = xp;
      A[3][0] = zp;   A[3][1] = yp;    A[3][2] = xn;  /*A[3][3] = 1;*/
}

// this function predicts the next orientation
// (q_dot = A*q)
// predictions are made based on the A matrix which is updates each 
// time new gyroscope data is available
// diagonal elements of the A matrix are always 1 and so 
// their multiplications are negated 
// in order to improve computational efficiency
// the function is called by the public void
// 'updateGyroscopeBasedQuaternion()'
void RollAndPitchKalmanFilter::predictNextQuaternion(){
	q_dot[0] = (/*A[0][0]*/q[0]) + (A[0][1]*q[1])    + (A[0][2]*q[2])    + (A[0][3]*q[3]);
	q_dot[1] = (A[1][0]*q[0])    + (/*A[1][1]*/q[1]) + (A[1][2]*q[2])    + (A[1][3]*q[3]);
	q_dot[2] = (A[2][0]*q[0])    + (A[2][1]*q[1])    + (/*A[2][2]*/q[2]) + (A[2][3]*q[3]);
	q_dot[3] = (A[3][0]*q[0])    + (A[3][1]*q[1])    + (A[3][2]*q[2])    + (/*A[3][3]*/q[3]);
}

// this function updates the prediction covariance 'P_next'
// (P_next = A*P*A' + Q)
// the function is called by the public void
// 'updateGyroscopeBasedQuaternion()'
void RollAndPitchKalmanFilter::findNextPredictionCovariance(){
	P_next[0] = Q + P[0];
	P_next[1] = Q + P[1];
	P_next[2] = Q + P[2];
	P_next[3] = Q + P[3];
}

void RollAndPitchKalmanFilter::computeMeasurementQuaternion(){
	float   ax = accelerometer_sample_x/accelerometer_sample_size, //
      		ay = accelerometer_sample_y/accelerometer_sample_size, // locally normalize accelerometer vector
      	  	az = accelerometer_sample_z/accelerometer_sample_size; //
	computeMeasurementQuaternion(ax, ay, az);
}

inline void RollAndPitchKalmanFilter::computeMeasurementQuaternion(float ax, float ay, float az){
	if(az >= 0){
	    q_measured[0] = sqrt((az+1)/2);
	    q_measured[1] = -ay/sqrt(2*(az +1)) ;
	    q_measured[2] =  ax/ sqrt(2*(az +1));
	    q_measured[3] = 0.0f;

	}
	else{
	    q_measured[0] = -ay/sqrt(2*(1-az));
	    q_measured[1] =  sqrt((1-az)/2);
	    q_measured[2] = 0.0f;
	    q_measured[3] = ax/ sqrt(2*(1-az));
	}
}

void RollAndPitchKalmanFilter::updateJacobianMatrix(){
    J[0][0] = -2*q_measured[2]; 
    J[0][1] =  2*q_measured[3]; 
    J[0][2] = -2*q_measured[0]; 

    J[0][3] =  2*q_measured[1];
    J[1][0] =  2*q_measured[1];
    J[1][1] =  2*q_measured[0]; 

    J[1][2] =  2*q_measured[3]; 
    J[1][3] =  2*q_measured[2];
    J[2][0] =  2*q_measured[0]; 

    J[2][1] = -2*q_measured[1]; 
    J[2][2] = -2*q_measured[2]; 
    J[2][3] =  2*q_measured[3];
}

void RollAndPitchKalmanFilter::updateDirectionCosineMatrix(){
    DCM[0] = 2*(q_measured[1]*q_measured[3] - q_measured[2]*q_measured[0]);
    DCM[1] = 2*(q_measured[2]*q_measured[3] + q_measured[1]*q_measured[0]);
    DCM[2] = q_measured[0]*q_measured[0] -
	     q_measured[1]*q_measured[1] -
	     q_measured[2]*q_measured[2] +
	     q_measured[3]*q_measured[3];
}

void RollAndPitchKalmanFilter::updateObjectiveErrorFunction(){
    OEF[0] = -accelerometer_sample_x/accelerometer_sample_size + DCM[0];
    OEF[1] = -accelerometer_sample_y/accelerometer_sample_size + DCM[1];
    OEF[2] = -accelerometer_sample_z/accelerometer_sample_size + DCM[2];
}

void RollAndPitchKalmanFilter::findGradOEF(){
    grad_OEF[0] = (J[0][0]*OEF[0]) + (J[1][0]*OEF[1]) + (J[2][0]*OEF[2]);
    grad_OEF[1] = (J[0][1]*OEF[0]) + (J[1][1]*OEF[1]) + (J[2][1]*OEF[2]);
    grad_OEF[2] = (J[0][2]*OEF[0]) + (J[1][2]*OEF[1]) + (J[2][2]*OEF[2]);
    grad_OEF[3] = (J[0][3]*OEF[0]) + (J[1][3]*OEF[1]) + (J[2][3]*OEF[2]);
}

// this function computes the new accelerometer based quaternion
// when the ASGD algorithm is taken into account
void RollAndPitchKalmanFilter::updateMeasurementQuaternionWithASGD(){
    float step_size_over_size_grad_OEF = step_size                       *
					 sqrt((grad_OEF[0]*grad_OEF[0])  +
				              (grad_OEF[1]*grad_OEF[1])  +
				              (grad_OEF[2]*grad_OEF[2])  +
				              (grad_OEF[3]*grad_OEF[3])) ;
	  
   q_measured[0] = q_measured[0] - (grad_OEF[0]*step_size_over_size_grad_OEF);
   q_measured[1] = q_measured[1] - (grad_OEF[1]*step_size_over_size_grad_OEF);
   q_measured[2] = q_measured[2] - (grad_OEF[2]*step_size_over_size_grad_OEF);
   q_measured[3] = q_measured[3] - (grad_OEF[3]*step_size_over_size_grad_OEF);
}

/**********************************************************************************
			KALMAN FILTER UPDATE STAGE FUNCTIONS
**********************************************************************************/

// this function determines the kalman gain
// K = P_next/(P_next + R)
// NOTE INVERSE - COMPUTATIONALLY DEMANDING!
// notation
//    - store = (P_next + R)
// called by various functions
// - 'updateGyroscopeBasedQuaternion()' 
// - 'updateAccelerometerBasedQuaternion()'
void RollAndPitchKalmanFilter::findKalmanGain(){
	float store[4]; 	
	store[0] = P_next[0] + R;
	store[1] = P_next[1] + R;
	store[2] = P_next[2] + R;
	store[3] = P_next[3] + R;
	
	K[0] = P_next[0]/store[0];
	K[1] = P_next[1]/store[1];
	K[2] = P_next[2]/store[2];
	K[3] = P_next[3]/store[3];
}

// this function computes the new quaternion state estimate
// q_new = q_dot + K*(q_measured - q_last)
// called by various functions
// - 'updateGyroscopeBasedQuaternion()' 
// - 'updateAccelerometerBasedQuaternion()'
void RollAndPitchKalmanFilter::updateQuaternion(){
	float q_dif[4] = {(q_measured[0] - q_dot[0]),
		          (q_measured[1] - q_dot[1]),
		          (q_measured[2] - q_dot[2]),
		          (q_measured[3] - q_dot[3]) };

	q[0] = q_dot[0] + (K[0]*q_dif[0]);
	q[1] = q_dot[1] + (K[1]*q_dif[1]);
	q[2] = q_dot[2] + (K[2]*q_dif[2]);
	q[3] = q_dot[3] + (K[3]*q_dif[3]);
}

// this function updates the predicted covariance matrix 'P'
// P = (1-K)*P_next
// called by various functions
// - 'updateGyroscopeBasedQuaternion()' 
// - 'updateAccelerometerBasedQuaternion()'
void RollAndPitchKalmanFilter::updatePMatrix(){
	P[0] = - P_next[0]*(K[0] - 1);
	P[1] = - P_next[1]*(K[1] - 1);
	P[2] = - P_next[2]*(K[2] - 1);
	P[3] = - P_next[3]*(K[3] - 1);
}
 
// this function takes the pointer to a quaternion
// this may be 'q_dot', 'q' or 'q_measured'
// the quaternion is then normalized 
// (ie to make it a unit quaternion) which is a requirement
// for much of the orientation determination algorithm
// the function is called in several places:
//  -  'updateGyroscopeBasedQuaternion()'
//  -  'updateAccelerometerBasedQuaternion()' 
inline void RollAndPitchKalmanFilter::normalizeQuaternion(float *quaternion){
	float q_temp[4] = {*(q+0),
                           *(q+1),
                           *(q+2),
                           *(q+3)},
	sum_of_squares = (q_temp[0] * q_temp[0]) +
                         (q_temp[1] * q_temp[1]) +
                         (q_temp[2] * q_temp[2]) +
                         (q_temp[3] * q_temp[3]),
	normalizer = inverseSquareRoot(sum_of_squares);

	q_temp[0] *= normalizer;
	q_temp[1] *= normalizer;
	q_temp[2] *= normalizer;
	q_temp[3] *= normalizer;
	

	*(q+0) = q_temp[0];
        *(q+1) = q_temp[1];
        *(q+2) = q_temp[2];
        *(q+3) = q_temp[3];
}

// this function is used to normalize quaternions. 
// the function determines the reciprical norm
// of 'number'. where 'number' is the sum
// of all the squared quaternion elements
// it is called by the inline void 
// 'normalizeQuaternion(float *quaternion)'
float RollAndPitchKalmanFilter::inverseSquareRoot(float sum_of_squared_quaternion_elements){
//  volatile long i;
//  volatile float x, y;
//  volatile const float f = 1.5F;
//
//  x = sum_of_squared_quaternion_elements * 0.5F;
//  y = sum_of_squared_quaternion_elements;
//  i = * ( long * ) &y;
//  i = 0x5f375a86 - ( i >> 1 );
//  y = * ( float * ) &i;
//  y = y * ( f - ( x * y * y ) );
//  return y;
	return 1.0f/sqrt(sum_of_squared_quaternion_elements); 
}


/*******************************************************************************
				GETTERS
*******************************************************************************/

float RollAndPitchKalmanFilter::getQ0(){ return q[0];}
float RollAndPitchKalmanFilter::getQ1(){ return q[1];}
float RollAndPitchKalmanFilter::getQ2(){ return q[2];}
float RollAndPitchKalmanFilter::getQ3(){ return q[3];}

float RollAndPitchKalmanFilter::getQMeasured0(){ return q_measured[0];}
float RollAndPitchKalmanFilter::getQMeasured1(){ return q_measured[1];}
float RollAndPitchKalmanFilter::getQMeasured2(){ return q_measured[2];}
float RollAndPitchKalmanFilter::getQMeasured3(){ return q_measured[3];}

float RollAndPitchKalmanFilter::getQDot0(){ return q_dot[0];}
float RollAndPitchKalmanFilter::getQDot1(){ return q_dot[1];}
float RollAndPitchKalmanFilter::getQDot2(){ return q_dot[2];}
float RollAndPitchKalmanFilter::getQDot3(){ return q_dot[3];}

float RollAndPitchKalmanFilter::getK0(){ return K[0];}
float RollAndPitchKalmanFilter::getK1(){ return K[1];}
float RollAndPitchKalmanFilter::getK2(){ return K[2];}
float RollAndPitchKalmanFilter::getK3(){ return K[3];}

float RollAndPitchKalmanFilter::getPNext0(){ return P_next[0];}
float RollAndPitchKalmanFilter::getPNext1(){ return P_next[1];}
float RollAndPitchKalmanFilter::getPNext2(){ return P_next[2];}
float RollAndPitchKalmanFilter::getPNext3(){ return P_next[3];}

float RollAndPitchKalmanFilter::getP0(){ return P[0];}
float RollAndPitchKalmanFilter::getP1(){ return P[1];}
float RollAndPitchKalmanFilter::getP2(){ return P[2];}
float RollAndPitchKalmanFilter::getP3(){ return P[3];}

float RollAndPitchKalmanFilter::getA00(){ return A[0][0];}
float RollAndPitchKalmanFilter::getA01(){ return A[0][1];}
float RollAndPitchKalmanFilter::getA02(){ return A[0][2];}
float RollAndPitchKalmanFilter::getA03(){ return A[0][3];}
float RollAndPitchKalmanFilter::getA10(){ return A[1][0];}
float RollAndPitchKalmanFilter::getA11(){ return A[1][1];}
float RollAndPitchKalmanFilter::getA12(){ return A[1][2];}
float RollAndPitchKalmanFilter::getA13(){ return A[1][3];}
float RollAndPitchKalmanFilter::getA20(){ return A[2][0];}
float RollAndPitchKalmanFilter::getA21(){ return A[2][1];}
float RollAndPitchKalmanFilter::getA22(){ return A[2][2];}
float RollAndPitchKalmanFilter::getA23(){ return A[2][3];}
float RollAndPitchKalmanFilter::getA30(){ return A[3][0];}
float RollAndPitchKalmanFilter::getA31(){ return A[3][1];}
float RollAndPitchKalmanFilter::getA32(){ return A[3][2];}
float RollAndPitchKalmanFilter::getA33(){ return A[3][3];}

/*******************************************************************************
				SETTERS
*******************************************************************************/

void RollAndPitchKalmanFilter::setGyroscopeVarianceX(float nGV)	  { gyroscope_varianceX = nGV;}
void RollAndPitchKalmanFilter::setAccelerometerVarianceX(float nAV){ accelerometer_varianceX = nAV;}

void RollAndPitchKalmanFilter::setGyroscopeVarianceY(float nGV)	  { gyroscope_varianceY = nGV;}
void RollAndPitchKalmanFilter::setAccelerometerVarianceY(float nAV){ accelerometer_varianceY = nAV;}

void RollAndPitchKalmanFilter::setGyroscopeVarianceZ(float nGV)	  { gyroscope_varianceZ = nGV;}
void RollAndPitchKalmanFilter::setAccelerometerVarianceZ(float nAV){ accelerometer_varianceZ = nAV;}

void RollAndPitchKalmanFilter::setGyroscopeSamplePeriod(float nGSP)    { gyroscope_sample_period = nGSP;}
void RollAndPitchKalmanFilter::setAccelerometerSamplePeriod(float nASP){ accelerometer_sample_period = nASP;}

void RollAndPitchKalmanFilter::setGyroscopeSampleX(float nGS)	  { gyroscope_sample_x = nGS;}
void RollAndPitchKalmanFilter::setAccelerometerSampleX(float nAS)  { accelerometer_sample_x = nAS;}

void RollAndPitchKalmanFilter::setGyroscopeSampleY(float nGS)	  { gyroscope_sample_y = nGS;}
void RollAndPitchKalmanFilter::setAccelerometerSampleY(float nAS)  { accelerometer_sample_y = nAS;}

void RollAndPitchKalmanFilter::setGyroscopeSampleZ(float nGS)	  { gyroscope_sample_z = nGS;}
void RollAndPitchKalmanFilter::setAccelerometerSampleZ(float nAS)  { accelerometer_sample_z = nAS;}


// END OF FILE -- RollAndPitchKalmanFilter.cpp
