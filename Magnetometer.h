#ifndef Magnetometer_h
#define Magnetometer_h

#include <Arduino.h>
#include <Wire.h>

#define MAGNETOMETER_I2C_ADDRESS_WRITE 0x3D
#define MAGNETOMETER_I2C_ADDRESS_READ  0x3C
#define MAGNETOMETER_ADDRESS           0x1E

///////////// DEFINE REGISTERS //////////////////

#define CONFIG_REG_A          0x00 // RW
#define CONFIG_REG_B          0x01 // RW
#define MODE_REG              0x02 // RW
#define DATA_REGISTER         0x03 // R
#define STATUS_REG            0x09 // R
#define INDENTIFICATION_REG_A 0x0A // R
#define INDENTIFICATION_REG_B 0x0B // R
#define INDENTIFICATION_REG_C 0x0C // R
#define NUM_OF_DATA_REGISTERS 6

///// DEFINE SAMPLE RATE - CONFIG_REG_A ////////
#define OUTPUT_RATE           0x18 // 75.0HZ
///// DEFINE GAIN- CONFIG_REG_B ////////////////
#define GAIN                  0x00 // +/-0.88Ga resolution = 4.35 (mG/LSB)
///// DEFINE OPERATIN MODE - MODE_REG ////////////////
#define CONTINUOUS_MODE       0x00 // sample continuously
#define SINGLE_MEASURE_MODE   0x00 // wait until data rdy is read before sampling again
#define IDLE_MODE             0x03 // idle device

#define INITIALIZATION_TIMEOUT 1000000  //try to connect for 1 second before giving up
#define INITIAL_RAW_ERROR_VALUE  999
///// DEFINE MESSAGES ////////////////
#define INITIALIZING_MESSAGE            "\nInitializing Magnetometer"
#define MAGNETOMETER_TURNED_ON_MESSAGE  "\nMagnetometer turned on"
#define MAGNETOMETER_TURNED_OFF_MESSAGE "\nMagnetometer turned off"
#define CONNECTION_SUCCESFUL_MESSAGE    "\nConnected to Magnetomer succesfully \nrecieving data"
#define CANNOT_CONNECT_ERROR_MESSAGE    "\nCannot connect to Magnetometer \nno data available \n1) check wiring \n2) check I2C Address"

struct Raw_Magnetometer{
    uint16_t raw_x = INITIAL_RAW_ERROR_VALUE,
            raw_y = INITIAL_RAW_ERROR_VALUE,
            raw_z = INITIAL_RAW_ERROR_VALUE;
};

struct Gauss_Values{
    float   gauss_x,
            gauss_y,
            gauss_z;
};

class Magnetometer{

	public:
	    Magnetometer();
	    Raw_Magnetometer     readRawData();
	    Gauss_Values         readGaussValues();

	    void initialize();
	    bool 	 getIsDataReady();
	    void         turnMagnetometerOn();
	    void         turnMagnetometerOff();
	    void         printRawData();
	    void         printRawX();
	    void         printRawY();
	    void         printRawZ();
	    void         printGaussValues();
	    void         printGaussValuesX();
	    void         printGaussValuesY();
	    void         printGaussValuesZ();
	    float        resolution;
	    long         turnOnMicros       = 0,
			 microsSinceTurnOn  = 0;

	    Raw_Magnetometer raw_data     = Raw_Magnetometer();
	    Gauss_Values     gauss_values = Gauss_Values();

	protected:
	    void     Write(int address, int data);
	    uint8_t* Read (int address, int numberOfBytes);

	private:
	    void defineOutputRate();
	    void defineGain();
	    void defineResolution();
	    void tryInitialConnection();

	    float resolutionsArray [8] = {0.73,0.92,1.22,1.52,2.27,2.56,3.03,4.35}; // mG/LSB
	    long  currentMicros  = 0,
		  previousMicros = 0;
};

#endif // Magnetometer_h
