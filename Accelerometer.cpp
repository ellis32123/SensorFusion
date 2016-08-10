#include "Accelerometer.h"
#include <Arduino.h>
#include <Wire.h>

Accelerometer::Accelerometer()
{
    //ctor
}

void Accelerometer::initialize(){
    turnAccelerometerOn();
    defineActivityControlSettings();
    defineTapThreshold();
    defineTapDuration();
    defineDataFormat();
    defineBandwidthAndOutputRate();
    defineRange();
    defineResolution();
    connectToAccelerometer();
}

void Accelerometer::turnAccelerometerOn(){
    write(POWER_CONTROL_ADDRESS,POWER_CONTROL_ON);
    turnOnMicros = micros();
}

void Accelerometer::turnAccelerometerOff(){
    write(POWER_CONTROL_ADDRESS,POWER_CONTROL_OFF);
}

void Accelerometer::defineActivityControlSettings(){
    write(ACTIVITY_AND_INACTIVITY_CONTROL_ADDRESS,ACTIVITY_AND_INACTIVITY_CONTROL);
}

void Accelerometer::defineTapThreshold(){
    write(TAP_THRESHOLD_ADDRESS,tap_threshold);
}

void Accelerometer::defineTapDuration(){
    float duration = tap_duration / 625;
    write(TAP_DURATION_ADDRESS,duration);
}

void Accelerometer::defineDataFormat(){
    write(DATA_FORMAT_ADDRESS,DATA_FORMAT);
}

void Accelerometer::defineBandwidthAndOutputRate(){
    write(BW_RATE_ADDRESS,BW_RATE);
}

void Accelerometer::defineRange(){
    uint8_t range_bits = DATA_FORMAT & 0x03;
    switch(range_bits){
        case 0x00:
            range = rangesArray[0]; // +/- 2g
            break;
        case 0x01:
            range = rangesArray[1]; // +/- 4g
            break;
        case 0x02:
            range = rangesArray[2]; // +/- 8g
            break;
        case 0x03:
            range = rangesArray[3]; // +/- 16g
            break;
        default:
            range = rangesArray[1]; // default to +/- 4g
    }
}

void Accelerometer::defineResolution(){
    uint8_t resolution_bit = DATA_FORMAT & 0x08;
    if(resolution_bit == 1) // if 13 bit
        resolution = 4.0f / 1000.0f;     // g/LSB
    else{
       resolution = (range*2.0f) / (1024.0f); // g/LSB;
    }
}

void Accelerometer::connectToAccelerometer(){
    boolean isDataRecieved = false;
    //Accelerometer::checkDeviceID();
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
        turnAccelerometerOff();
}


void Accelerometer::checkDeviceID(){
    byte* device_id_read = read(DEVICE_ID_ADDRESS, 1);
    if(device_id_read[1] == DEVICE_ID)
        Serial.println(DEVICE_ID_READ_SUCCESSFUL);
    else
        Serial.println(DEVICE_ID_READ_UNSUCCESSFUL);

}

Raw_Accelerometer Accelerometer::readRawData(){
    byte* dataBuffer = read(OUTPUT_X1_ADDRESS,NUM_OF_OUTPUT_BYTES);
    raw_data.raw_x = ((int)dataBuffer[1] << 8) | dataBuffer[0];
    raw_data.raw_y = ((int)dataBuffer[3] << 8) | dataBuffer[2];
    raw_data.raw_z = ((int)dataBuffer[5] << 8) | dataBuffer[4];
    return raw_data;
}

G_Values Accelerometer::readGValues(){
    readRawData();
    g_values.g_x = raw_data.raw_x*resolution;
    g_values.g_y = raw_data.raw_y*resolution;
    g_values.g_z = raw_data.raw_z*resolution;
    return g_values;
}

void Accelerometer::write(int address, int data){
    Wire.beginTransmission(ACCELEROMETER_ADDRESS);
    Wire.write(address);
    Wire.write(data);
    Wire.endTransmission();
}

uint8_t* Accelerometer::read(int address, int number_of_bytes){
    uint8_t data_buffer[number_of_bytes];
    Wire.beginTransmission(ACCELEROMETER_ADDRESS);
    Wire.write(address);
    Wire.endTransmission();

    Wire.beginTransmission(ACCELEROMETER_ADDRESS);
    Wire.requestFrom(ACCELEROMETER_ADDRESS, number_of_bytes);

    if(Wire.available() == number_of_bytes){
		for(uint8_t i=0; i<number_of_bytes; i++){
		    data_buffer[i] = Wire.read();
		}
	    }
	    return data_buffer;
	}

uint8_t Accelerometer::readByte(int address){
    uint8_t data_buffer;
    Wire.beginTransmission(ACCELEROMETER_ADDRESS);
    Wire.write(address);
    Wire.endTransmission();

    Wire.beginTransmission(ACCELEROMETER_ADDRESS);
    Wire.requestFrom(ACCELEROMETER_ADDRESS, 1);

    if(Wire.available() == 1){
		    data_buffer = Wire.read();
    }
	    return data_buffer;
}



	void Accelerometer::printRawData(){
	    readRawData();
	    printRawX();
	    printRawY();
	    printRawZ();
	}

	void Accelerometer::printRawX(){
	    Serial.print(" raw X: ");
	    Serial.print(raw_data.raw_x);
	}

	void Accelerometer::printRawY(){
	    Serial.print(" raw Y: ");
	    Serial.print(raw_data.raw_y);
	}

	void Accelerometer::printRawZ(){
	    Serial.print(" raw Z: ");
	    Serial.print(raw_data.raw_z);
	}

	void Accelerometer::printGValues(){ readGValues();
	    printGValuesX();
	    printGValuesY();
	    printGValuesZ();
	}

	void Accelerometer::printGValuesX(){
	    Serial.print(" gX: ");
	    Serial.print(g_values.g_x,5);
	}

	void Accelerometer::printGValuesY(){
	    Serial.print(" gY: ");
	    Serial.print(g_values.g_y,5);
	}

	void Accelerometer::printGValuesZ(){
	    Serial.print(" gZ: ");
	    Serial.print(g_values.g_z,5);
	}

/****************************************************************************
			INTERRUPT TRIGGER CHECK
****************************************************************************/


	bool    Accelerometer::getIsInterruptTriggered(){
			buffer = readByte(INTERRUPT_SOURCE_ADDRESS);
			return buffer != 0x00;
	}
	bool    Accelerometer::getIsDataReady(){		
			buffer = readByte(INTERRUPT_SOURCE_ADDRESS); 
			return (buffer >> DATA_READY_BIT) & 1;
	} 
	bool    Accelerometer::getIsSingleTapTriggered(){       
			buffer = readByte(INTERRUPT_SOURCE_ADDRESS); 
			return (buffer >> SINGLE_TAP)  & 1     ;
	} 			      
	bool    Accelerometer::getIsDoubleTapTriggered(){	
			buffer = readByte(INTERRUPT_SOURCE_ADDRESS); 
			return (buffer >> DOUBLE_TAP)  & 1     ;
	} 			      
	bool    Accelerometer::getIsActivityTriggered(){	
			buffer = readByte(INTERRUPT_SOURCE_ADDRESS); 
			return (buffer >> ACTIVITY)  & 1  	  ;
	} 			      
	bool    Accelerometer::getIsInactivityTriggered(){	
			buffer = readByte(INTERRUPT_SOURCE_ADDRESS); 
			return (buffer >> INACTIVITY	  )  & 1 ;
	} 			      
	bool    Accelerometer::getIsFreeFallTriggered(){	
			buffer = readByte(INTERRUPT_SOURCE_ADDRESS); 
			return (buffer >> FREE_FALL	  )  & 1 ;
	}
	bool    Accelerometer::getIsWatermarkTriggered(){	
			buffer = readByte(INTERRUPT_SOURCE_ADDRESS); 
			return (buffer >> WATERMARK	  )  & 1 ;
	}			      
	bool    Accelerometer::getIsOverrunTriggered(){		
			buffer = readByte(INTERRUPT_SOURCE_ADDRESS); 
			return (buffer >> OVERRUN	  )  & 1 ;
	}			      


/****************************************************************************
			INTERRUPT PIN DEFINITIONS 
***************************************************************************/

	void    Accelerometer::setIsDataReadyInterruptOnINT1Pin(bool iDRIOI1P){	
			buffer = readByte(INTERRUPT_MAP_ADDRESS);			      
//			write(INTERRUPT_MAP_ADDRESS, (( buffer & DATA_READY_BIT) | iDRIOI1P << DATA_READY_BIT));
buffer = 0x80;
		write(INTERRUPT_MAP_ADDRESS,  buffer); 

	}
	void    Accelerometer::setIsSingleTapInterruptOnINT1Pin(bool iSTIOI1P){	
			 buffer = readByte(INTERRUPT_MAP_ADDRESS);
			 write(INTERRUPT_MAP_ADDRESS, (( buffer & SINGLE_TAP) 	 | iSTIOI1P  << SINGLE_TAP));
	}
	void    Accelerometer::setIsDoubleTapInterruptOnINT1Pin(bool iDTIOI1P){ 
			 buffer = readByte(INTERRUPT_MAP_ADDRESS);
			 write(INTERRUPT_MAP_ADDRESS, (( buffer & DOUBLE_TAP )	 | iDTIOI1P << DOUBLE_TAP));
	}
	void    Accelerometer::setIsActivityInterruptOnINT1Pin(bool iAIOI1P){	
			 buffer = readByte(INTERRUPT_MAP_ADDRESS);
			 write(INTERRUPT_MAP_ADDRESS, (( buffer & ACTIVITY)	 | iAIOI1P << ACTIVITY));
	}
	void    Accelerometer::setIsInactivityInterruptOnINT1Pin(bool iIOI1P){	
			 buffer = readByte(INTERRUPT_MAP_ADDRESS);
			 write(INTERRUPT_MAP_ADDRESS, (( buffer & INACTIVITY) 	 | iIOI1P << INACTIVITY));
	}
	void    Accelerometer::setIsFreeFallInterruptOnINT1Pin(bool iFFIOI1P){	
			 buffer = readByte(INTERRUPT_MAP_ADDRESS);
			 write(INTERRUPT_MAP_ADDRESS, (( buffer & FREE_FALL)	 | iFFIOI1P << FREE_FALL));
	}
	void    Accelerometer::setIsWatermarkInterruptOnINT1Pin(bool iWIOI1P){	
			 buffer = readByte(INTERRUPT_MAP_ADDRESS);
			 write(INTERRUPT_MAP_ADDRESS, (( buffer & WATERMARK) 	 | iWIOI1P << WATERMARK));
	}
	void    Accelerometer::setIsOverrunInterruptOnINT1Pin(bool iOIOI1P){
			 buffer = readByte(INTERRUPT_MAP_ADDRESS);
			 write(INTERRUPT_MAP_ADDRESS, (( buffer & OVERRUN)	  | iOIOI1P << OVERRUN));
	}

/****************************************************************************
			INTERRUPT ENABLES
****************************************************************************/

	void    Accelerometer::setDataReadyInterrupt(bool isDataReadyEnabled){
			 buffer = readByte(INTERRUPT_ENABLE_ADDRESS); 

			 buffer |= 1 << DATA_READY_BIT;
			 buffer = 0x80; /////////////////////////// NEEDS CHANGING!!!!
			 write(INTERRUPT_ENABLE_ADDRESS, buffer);
	}
	void    Accelerometer::setSingleTapInterrupt(bool isSingleTapEnabled){	
			 buffer = readByte(INTERRUPT_ENABLE_ADDRESS); 
			 write(INTERRUPT_ENABLE_ADDRESS, (( buffer & SINGLE_TAP)     | isSingleTapEnabled << SINGLE_TAP));
	}
	void    Accelerometer::setDoubleTapInterrupt(bool isDoubleTapEnabled){	
			 buffer = readByte(INTERRUPT_ENABLE_ADDRESS); 
			 write(INTERRUPT_ENABLE_ADDRESS, (( buffer & DOUBLE_TAP)     | isDoubleTapEnabled << DOUBLE_TAP));
	}
	void    Accelerometer::setActivityInterrupt(bool isActivityEnabled){   	
			 buffer = readByte(INTERRUPT_ENABLE_ADDRESS); 
			 write(INTERRUPT_ENABLE_ADDRESS, (( buffer & ACTIVITY)       | isActivityEnabled << ACTIVITY));
	}
	void    Accelerometer::setInactivityInterrupt(bool isInactivityEnabled){ 
			 buffer = readByte(INTERRUPT_ENABLE_ADDRESS); 
			 write(INTERRUPT_ENABLE_ADDRESS, (( buffer & INACTIVITY)     | isInactivityEnabled << INACTIVITY));
	}
	void    Accelerometer::setFreeFallInterrupt(bool isFreeFallEnabled){     
			 buffer = readByte(INTERRUPT_ENABLE_ADDRESS); 
			 write(INTERRUPT_ENABLE_ADDRESS, (( buffer & FREE_FALL)      | isFreeFallEnabled << FREE_FALL));
	}
	void    Accelerometer::setWatermarkInterrupt(bool isWatermarkEnabled){   
			 buffer = readByte(INTERRUPT_ENABLE_ADDRESS); 
			 write(INTERRUPT_ENABLE_ADDRESS, (( buffer & WATERMARK)      | isWatermarkEnabled << WATERMARK));
	}
	void    Accelerometer::setOverrunInterrupt(bool isOverrunEnabled){       
			 buffer = readByte(INTERRUPT_ENABLE_ADDRESS); 
			 write(INTERRUPT_ENABLE_ADDRESS, (( buffer & OVERRUN)        | isOverrunEnabled << OVERRUN));
	}
// END OF FILE (ACCELEROMETER.CPP)
