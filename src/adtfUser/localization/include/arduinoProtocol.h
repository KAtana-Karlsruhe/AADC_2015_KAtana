/**********************************************************************
* Copyright (c) 2013 BFFT Gesellschaft fuer Fahrzeugtechnik mbH.
* All rights reserved.
**********************************************************************
* $Author:: spiesra $  $Date:: 2014-09-16 13:29:48#$ $Rev:: 26104   $
**********************************************************************/
#ifndef ARDUINOPROTOCOL_H
#define ARDUINOPROTOCOL_H

#pragma pack(push)  /* push current alignment to stack */
#pragma pack(1)     /* set alignment to 1 unsigned char boundary */

/*! this headers includes all the struct which are transmitted between the odroid and the arduino. It contains the structs with the sensor data and with the actuator data */

// main frames:

/*! the ARDUINO start of frame identifier */
const unsigned char ID_ARD_SOF = 0x0D;
/*! the ARDUINO empty frame identifier */
const unsigned char ID_ARD_EMPTY = 0x00;



/*! the ARDUINO dummy (placeholder) identifier */
const unsigned char ID_ARD_DUMMY = 0xFF;

// sensor data packages:

/*! the ARDUINO identifier for steering_servo sensor*/
const unsigned char ID_ARD_SENS_STEER_ANGLE = 0xA1;
/*! the tag for the steering angle*/
typedef struct tag_SteeringAngleData
{
	unsigned short ui16Angle;				/*!< angle of servo, range from 0...180, zero position is 90 */
} tSteeringAngleData;


/*! the ARDUINO identifier for wheelencoders sensor */
const unsigned char ID_ARD_SENS_WHEELENC = 0xB2;
/*! the tag for the wheel encoder data*/
typedef struct tag_WheelEncoderData
{
	unsigned int ui32LeftWheel;			/*!< counter from left wheel */
	unsigned int ui32RightWheel;			/*!< counter from right wheel */
} tWheelEncoderData;


/*! the ARDUINO identifier for inertial measurement unit; range and interpretation is dependent on mode of IMU; */
const unsigned char ID_ARD_SENS_IMU = 0x81;
/*! the tag for the wheel imudata data*/
typedef struct tag_ImuData
{
  signed short i16Q_w;					/*!< quaternion w-axis */
  signed short i16Q_x;					/*!< quaternion x-axis */
  signed short i16Q_y;					/*!< quaternion y-axis */
  signed short i16Q_z;					/*!< quaternion z-axis */
  
  signed short i16A_x;					/*!< acceleration x-axis */
  signed short i16A_y;					/*!< acceleration y-axis */
  signed short i16A_z; 					/*!< acceleration z-axis */
  
  signed short i16Temperature;			/*!< temperature */
} tImuData;


/*! the ARDUINO identifier for infrared_sensors */
const unsigned char ID_ARD_SENS_IR = 0xD1;
/*! the tag for the ir data*/
typedef struct tag_IrData
{
	unsigned short ui16FrontCenterLongrange;   /*!< voltage from front center longrange sensor */
	unsigned short ui16FrontCenterShortange;   /*!< voltage from front center shortrange sensor */
	unsigned short ui16FrontLeftLongrange;	    /*!< voltage from front left longrange sensor  */
	unsigned short ui16FrontLeftShortrange;    /*!< voltage from front left shortrange sensor  */
	unsigned short ui16FrontRightLongrange;    /*!< voltage from front right longrange sensor  */
	unsigned short ui16FrontRightShortrange;   /*!< voltage from front right shortrange sensor  */
	unsigned short ui16RearCenterShortrange;   /*!< voltage from rear center shortrange sensor  */
	unsigned short ui16RearLeftShortrange;	    /*!< voltage from rear left shortrange sensor  */
	unsigned short ui16RearRightShortrange;    /*!< voltage from rear right shortrange sensor  */
} tIrData;

/*! the ARDUINO identifier for  photo_sensor */
const unsigned char ID_ARD_SENS_PHOTO = 0xE1;
/*! the tag for the photo data*/
typedef struct tag_PhotoData
{
	unsigned short ui16Luminosity;				/*!< luminosity value in lux */
} tPhotoData;


/*! the ARDUINO identifier for ultrsasonic sensors */
const unsigned char ID_ARD_SENS_US = 0x92;
/*! the tag for the ultra sonic data*/
typedef struct tag_UsData
{
	unsigned short ui16FrontLeft;				/*!< data from ultrasonic sensor front left; value in cm */
	unsigned short ui16FrontRight;				/*!< data from ultrasonic sensor front right; value in cm */
	unsigned short ui16RearLeft;					/*!< data from ultrasonic sensor rear left; value in cm */
	unsigned short ui16RearRight;				/*!< data from ultrasonic sensor rear right; value in cm */
} tUsData;

//
/*! the ARDUINO identifier for voltage measurements */
const unsigned char ID_ARD_SENS_VOLTAGE = 0xC1;
/*! the tag for the voltage data*/
typedef struct tag_VoltageData
{
	unsigned short ui16Measurement;				/*!< data from voltage measurement pins; has to be calculated */	
	unsigned short ui16Power;						/*!< data from voltage power pins; has to be calculated */
} tVoltageData;

//actuator packages:


/*! the ARDUINO identifier for watchdog toggle */
const unsigned char ID_ARD_ACT_WD_TOGGLE = 0x72;
/*! the tag for the watchdog toggle data*/
typedef struct tag_WatchdogToggleData
{
	unsigned char ui8Toggle;						/*!< flag for enable or disable */
} tWatchdogToggleData;


/*! the ARDUINO identifier for enabling watchdog */
const unsigned char ID_ARD_ACT_WD_ENABLE = 0x71;
/*! the tag for the watchdog enable data*/
typedef struct tag_WatchdogEnableData
{
	unsigned char ui8IsEnabled;						/*!< flag for enable or disable */
} tWatchdogEnableData;

/*! the ARDUINO identifier for motor relais */
const unsigned char ID_ARD_ACT_MOT_RELAIS = 0x73;
/*! the tag for the motor relais data*/
typedef struct tag_MotorRelaisData
{
	unsigned char ui8IsEnabled;						/*!< flag for enable or disable */
} tMotorRelaisData;


/*! the ARDUINO identifier for steering_servo actuator */
const unsigned char ID_ARD_ACT_STEER_ANGLE = 0x63;


/*! the ARDUINO identifier for acceleration servo actuator */
const unsigned char ID_ARD_ACT_ACCEL_SERVO = 0x64;
/*! the tag for the acceleration servo data*/
typedef struct tag_AccelerationServoData
{
	unsigned char ui8Angle;						/*!< angle of servo, range from 0...180, zero position is 90 */
} tAccelerationServoData;

/*! the ARDUINO identifier for enable or disable the lights*/
const unsigned char ID_ARD_ACT_LIGHT_DATA = 0x65;

/*! these ids are used in the second byte to address the light */
const unsigned char ID_ARD_ACT_LIGHT_DATA_HEAD = 0x01; 
const unsigned char ID_ARD_ACT_LIGHT_DATA_BACK = 0x02; 
const unsigned char ID_ARD_ACT_LIGHT_DATA_BRAKE = 0x04; 
const unsigned char ID_ARD_ACT_LIGHT_DATA_TURNLEFT = 0x08; 
const unsigned char ID_ARD_ACT_LIGHT_DATA_TURNRIGHT = 0x10;
const unsigned char ID_ARD_ACT_LIGHT_DATA_REVERSE = 0x20;
/*! the tag for the light data*/
typedef struct tag_LightData
{
	unsigned char ui8LightMode;				/*!< SIGNAL Nr. Please see defines*/
	unsigned char ui8IsEnabled;				/*!< flag for enable or disable */
} tLightData;



/*! the main frame around every data package which comes from the sensors and actuator*/
typedef struct tag_ArduinoHeader
{
    unsigned char   i8SOF;					/*!< the start of frame */
    unsigned char   chID;					/*!< identifier of package */
    unsigned int ui32ArduinoTimestamp;		/*!< timestamp from arduino */
    unsigned char   i8DataLength;			/*!< the length of the data */
} tArduinoHeader;

/*! the ARDUINO communication frame with its data area */
typedef union tag_ArduionDataUnion
{
    tSteeringAngleData      sSteeringData;		/*!< the steering angle data */
    tWheelEncoderData       sWheelEncData;		/*!< the wheel encoder data */
    tImuData                sImuData;			/*!< the inertial measurement unit data */
    tIrData                 sIrData;			/*!< the infrared data */
    tPhotoData              sPhotoData;			/*!< the photo sensor data */
    tUsData                 sUsData;			/*!< the ultra sonic sensor data */
    tVoltageData            sVoltageData;		/*!< the voltage data */
    tWatchdogToggleData     sWdToggleData;		/*!< the watchdog toggle data */
    tWatchdogEnableData     sWDEnableData;		/*!< the watchdog enable data */
    tMotorRelaisData        sMotorRelaisData;	/*!< the motor relais switch data */
    tAccelerationServoData  sAccServoData;		/*!< the acceleration servo data */
    tLightData              sLightData;			/*!< the light data */
} tArduinoDataUnion;

/*! the tag for the arduino frame*/
typedef struct tag_ArduinoFrame
{
    tArduinoHeader      sHeader;	/*!< the header part of the data */
    tArduinoDataUnion   sData;		/*!<  variable length contains data structs*/
} tArduinoFrame;


#pragma pack(pop)   /* restore original alignment from stack */

#endif // ARDUINOPROTOCOL_H







