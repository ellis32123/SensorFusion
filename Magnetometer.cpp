#include "Magnetometer.h"
#include <Arduino.h>
#include <Wire.h>

Magnetometer::Magnetometer(){

}

void Magnetometer::initialize(){
    defineOutputRate();
    defineGain();
    tryInitialConnection();
    defineResolution();
}

void Magnetometer::defineOutputRate(){
    Write(CONFIG_REG_A, OUTPUT_RATE);
}

void Magnetometer::defineGain(){
    Write(CONFIG_REG_B, GAIN);
}

void Magnetometer::turnMagnetometerOn(){
	// change to CONTINUOUS_MODE if not using interrupts
    	Write(MODE_REG, SINGLE_MEASURE_MODE);
	turnOnMicros = micros();
}

void Magnetometer::turnMagnetometerOff(){
    Write(MODE_REG, IDLE_MODE);
}

void Magnetometer::tryInitialConnection(){
    boolean isDataRecieved = false;
    turnMagnetometerOn();
    while(microsSinceTurnOn < INITIALIZATION_TIMEOUT){
        currentMicros = micros();
        microsSinceTurnOn = currentMicros - turnOnMicros;
        readRawData();
        if(   raw_data.raw_x != INITIAL_RAW_ERROR_VALUE
           && raw_data.raw_y != INITIAL_RAW_ERROR_VALUE
           && raw_data.raw_z != INITIAL_RAW_ERROR_VALUE){
            isDataRecieved = true;
            break;
        }
    }
    if(!isDataRecieved)
        turnMagnetometerOff();
}


Raw_Magnetometer Magnetometer::readRawData(){
    uint8_t* rawDataBuffer = Read(DATA_REGISTER, NUM_OF_DATA_REGISTERS);
    raw_data.raw_x = (rawDataBuffer[0] << 8) | rawDataBuffer[1];
    raw_data.raw_y = (rawDataBuffer[2] << 8) | rawDataBuffer[3];
    raw_data.raw_z = (rawDataBuffer[4] << 8) | rawDataBuffer[5];
    return raw_data;
}

Gauss_Values Magnetometer::readGaussValues(){
    readRawData();
    gauss_values.gauss_x = raw_data.raw_x * resolution;
    gauss_values.gauss_y = raw_data.raw_y * resolution;
    gauss_values.gauss_z = raw_data.raw_z * resolution;
    return gauss_values;
}

void Magnetometer::defineResolution(){
    switch(GAIN){
        case 0x00:
            resolution = resolutionsArray[0];
            break;
        case 0x20:
            resolution = resolutionsArray[1];
            break;
        case 0x40:
            resolution = resolutionsArray[2];
            break;
        case 0x60:
            resolution = resolutionsArray[3];
            break;
        case 0x80:
            resolution = resolutionsArray[4];
            break;
        case 0xA0:
            resolution = resolutionsArray[5];
            break;
        case 0xC0:
            resolution = resolutionsArray[6];
            break;
        default:
            resolution = resolutionsArray[7];
            break;
    }
}

bool Magnetometer::getIsDataReady(){
	uint8_t* status_reg_byte = Read(STATUS_REG ,1);
	    	Write(MODE_REG, SINGLE_MEASURE_MODE);
	uint8_t data = status_reg_byte[0];
	return  data & 0x01; // no shift required as lsb 
}

void Magnetometer::Write(int address, int data){
    Wire.beginTransmission(MAGNETOMETER_ADDRESS);
    Wire.write(address);
    Wire.write(data);
    Wire.endTransmission();
}

uint8_t* Magnetometer::Read(int address, int numberOfBytes){
    uint8_t dataBuffer[numberOfBytes];
    Wire.beginTransmission(MAGNETOMETER_ADDRESS);
    Wire.write(address);
    Wire.endTransmission();

    Wire.beginTransmission(MAGNETOMETER_ADDRESS);
    Wire.requestFrom(MAGNETOMETER_ADDRESS, numberOfBytes);

    if(Wire.available() == numberOfBytes){
        for(uint8_t i = 0; i<numberOfBytes; i++){
            dataBuffer[i] = Wire.read();
        }
    }
    Wire.endTransmission();
    return dataBuffer;
}

/***********************************************************************************************
				PRINT FUNCTIONS
***********************************************************************************************/

void Magnetometer::printRawData(){
    readRawData();
    printRawX();
    printRawY();
    printRawZ();
}

void Magnetometer::printRawX(){
    Serial.print(" raw X: ");
    Serial.print(raw_data.raw_x);
}

void Magnetometer::printRawY(){
    Serial.print(" raw Y: ");
    Serial.print(raw_data.raw_y);
}

void Magnetometer::printRawZ(){
    Serial.print(" raw Z: ");
    Serial.print(raw_data.raw_z);
}

void Magnetometer::printGaussValues(){
    readGaussValues();
    printGaussValuesX();
    printGaussValuesY();
    printGaussValuesZ();
}

void Magnetometer::printGaussValuesX(){
    Serial.print(" Gauss X: ");
    Serial.print(gauss_values.gauss_x);
}

void Magnetometer::printGaussValuesY(){
    Serial.print(" Gauss Y: ");
    Serial.print(gauss_values.gauss_y);
}

void Magnetometer::printGaussValuesZ(){
    Serial.print(" Gauss Z: ");
    Serial.print(gauss_values.gauss_z);
}

// end of file -- Magnetometer.cpp
