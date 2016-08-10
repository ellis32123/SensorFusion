// SensorInitialization.h
//
// header file for the program 'SensorInitialization.cpp'

#ifndef SENSORINITIALIZATION_H
#define SENSORINITIALIZATION_H

#include "Arduino.h"

class SensorInitialization
{
    public:
        SensorInitialization();

        void  setNewSample(float nS);
        void setNumberOfSamples(unsigned long nC);
        void  setSamplesSum(float nSS);
        void  setDeviationsSum(float nDS);
        float getMean();
        float getSamplePeriodsMean();
        float getStandardDeviation();
        float getVariance();
        unsigned long getNumberOfSamples();
    private:
        void            update();
        void            updateSamplePeriod();
        void            updateSamplesSum();
        void            updateMean();
        void            updateDeviationsSum();
        void            updateStandardDeviation();
        void            updateVariance();
        unsigned long   number_of_samples  = 0, current_micros = 0, previous_micros = 0;
        float           sum_samples    = 0.0f,
                        sum_deviations = 0.0f,
                        mean           = 0.0f,
                        s_d            = 0.0f,
                        T              = 0.0f,
                        variance       = 0.0f,
                        current_sample = 0.0f;
};

#endif // SENSORINITIALIZATION_H
