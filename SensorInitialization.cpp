// SensorInitialization.cpp
//
// This file takes sensor data and finds basic characteristics such as 
// mean standard deviation and sample period

#include "SensorInitialization.h"

SensorInitialization::SensorInitialization()
{

}

// public function 
// triggers update() function -- (updates all characteristics based on new sample data)
void SensorInitialization::setNewSample(float nS){
    current_sample = nS;
    if(((current_sample < mean*10.0f) && (current_sample > mean/5.0f)) || current_sample == 0.0f || number_of_samples < 5){
        number_of_samples ++;
        update();
    }
}

// updates all characteristics only when a new sample is made available
// this function is called by the public function:
// 'setNewSample(float nS)'
void SensorInitialization::update(){
    updateSamplePeriod();
    updateSamplesSum();
    updateMean();
    updateDeviationsSum();
    updateStandardDeviation();
    updateVariance();
}

// function finds the difference between the time at
// the previous sample and current sample and 
// uses this to find the current sample period
void SensorInitialization::updateSamplePeriod(){
    current_micros = micros();
    T = (current_micros - previous_micros)/1000000.0f; // Difference in time in seconds
    previous_micros = current_micros;
}

// function finds the sum of all the samples recorded
void SensorInitialization::updateSamplesSum(){
    sum_samples += current_sample; //  sum of 'ALL' samples recorded
}

// function computes mean of all samples 
void SensorInitialization::updateMean(){
    mean = sum_samples/number_of_samples; //  mean of 'ALL' samples recorded
}

// deviation equals (x - mean)^2
void SensorInitialization::updateDeviationsSum(){
   	float difference = current_sample - mean; 
	sum_deviations += difference*difference; 
}

// Standard deviation is the mean of devations
// 'sum_deviations' is determined in the function: 
// updateDeviationsSum()
void SensorInitialization::updateStandardDeviation(){
    s_d = sum_deviations/number_of_samples;
}

// computes 'sigma^2'
// (aka covariance)
void SensorInitialization::updateVariance(){
    variance = s_d*s_d;
}


// SETTERS //
void SensorInitialization::setSamplesSum(float nSS){sum_samples = nSS;}
void SensorInitialization::setDeviationsSum(float nDS){sum_deviations = nDS;}
void SensorInitialization::setNumberOfSamples(unsigned long nC){ number_of_samples = nC;}


// GETTERS //
float SensorInitialization::getMean(){                   	 return mean;}
float SensorInitialization::getSamplePeriodsMean(){      	 return T;}
float SensorInitialization::getStandardDeviation(){      	 return s_d;}
float SensorInitialization::getVariance(){               	 return variance;}
unsigned long   SensorInitialization::getNumberOfSamples(){      return number_of_samples;}


