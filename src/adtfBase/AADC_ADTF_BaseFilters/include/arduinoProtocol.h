/**********************************************************************
* Copyright (c) 2013 BFFT Gesellschaft fuer Fahrzeugtechnik mbH.
* All rights reserved.
**********************************************************************
* $Author:: forchhe#$  $Date:: 2015-03-09 10:07:20#$ $Rev:: 32771   $
**********************************************************************/
#ifndef ARDUINOPROTOCOL_H
#define ARDUINOPROTOCOL_H


#pragma pack(push)  /* push current alignment to stack */
#pragma pack(1)     /* set alignment to 1 tUInt8 boundary */

/*! this headers includes all the struct which are transmitted between the odroid and the arduino. It contains the structs with the sensor data and with the actuator data */

// main frames:

/*! the ARDUINO start of frame identifier */
const tUInt8 ID_ARD_SOF = 0x0D;
/*! the ARDUINO empty frame identifier */
const tUInt8 ID_ARD_EMPTY = 0x00;

/*! the ARDUINO dummy (placeholder) identifier */
const tUInt8 ID_ARD_DUMMY = 0xFF;

// sensor data packages:

/*! the ARDUINO identifier for steering_servo sensor*/
const tUInt8 ID_ARD_SENS_STEER_ANGLE = 0xA1;
/*! the tag for the steering angle*/
typedef struct tag_SteeringAngleData
{
    tUInt16 ui16Angle;                /*!< angle of servo, range from 0...180, zero position is 90 */
} tSteeringAngleData;


/*! the ARDUINO identifier for wheelencoders sensor */
const tUInt8 ID_ARD_SENS_WHEELENC = 0xB2;
/*! the tag for the wheel encoder data*/
typedef struct tag_WheelEncoderData
{
    tUInt32 ui32LeftWheel;            /*!< counter from left wheel */
    tUInt32 ui32RightWheel;            /*!< counter from right wheel */
} tWheelEncoderData;


/*! the ARDUINO identifier for inertial measurement unit; range and interpretation is dependent on mode of IMU; */
const tUInt8 ID_ARD_SENS_IMU = 0x81;
/*! the tag for the wheel imudata data*/
typedef struct tag_ImuData
{
  tInt16 i16Q_w;                    /*!< quaternion w-axis */
  tInt16 i16Q_x;                    /*!< quaternion x-axis */
  tInt16 i16Q_y;                    /*!< quaternion y-axis */
  tInt16 i16Q_z;                    /*!< quaternion z-axis */
  
  tInt16 i16A_x;                    /*!< acceleration x-axis */
  tInt16 i16A_y;                    /*!< acceleration y-axis */
  tInt16 i16A_z;                     /*!< acceleration z-axis */
  
  tInt16 i16Temperature;            /*!< temperature */
} tImuData;


/*! the ARDUINO identifier for infrared_sensors */
const tUInt8 ID_ARD_SENS_IR = 0xD1;
/*! the tag for the ir data*/
typedef struct tag_IrData
{
    tUInt16 ui16FrontCenterLongrange;   /*!< voltage from front center longrange sensor */
    tUInt16 ui16FrontCenterShortange;   /*!< voltage from front center shortrange sensor */
    tUInt16 ui16FrontLeftLongrange;     /*!< voltage from front left longrange sensor  */
    tUInt16 ui16FrontLeftShortrange;    /*!< voltage from front left shortrange sensor  */
    tUInt16 ui16FrontRightLongrange;    /*!< voltage from front right longrange sensor  */
    tUInt16 ui16FrontRightShortrange;   /*!< voltage from front right shortrange sensor  */
    tUInt16 ui16RearCenterShortrange;   /*!< voltage from rear center shortrange sensor  */
    tUInt16 ui16RearLeftShortrange;        /*!< voltage from rear left shortrange sensor  */
    tUInt16 ui16RearRightShortrange;    /*!< voltage from rear right shortrange sensor  */
} tIrData;

/*! the ARDUINO identifier for  photo_sensor */
const tUInt8 ID_ARD_SENS_PHOTO = 0xE1;
/*! the tag for the photo data*/
typedef struct tag_PhotoData
{
    tUInt16 ui16Luminosity;                /*!< luminosity value in lux */
} tPhotoData;


/*! the ARDUINO identifier for ultrsasonic sensors */
const tUInt8 ID_ARD_SENS_US = 0x92;
/*! the tag for the ultra sonic data*/
typedef struct tag_UsData
{
    tUInt16 ui16FrontLeft;                /*!< data from ultrasonic sensor front left; value in cm */
    tUInt16 ui16FrontRight;                /*!< data from ultrasonic sensor front right; value in cm */
    tUInt16 ui16RearLeft;                    /*!< data from ultrasonic sensor rear left; value in cm */
    tUInt16 ui16RearRight;                /*!< data from ultrasonic sensor rear right; value in cm */
} tUsData;

//
/*! the ARDUINO identifier for voltage measurements */
const tUInt8 ID_ARD_SENS_VOLTAGE = 0xC1;
/*! the tag for the voltage data*/
typedef struct tag_VoltageData
{
    tUInt16 ui16Measurement;                /*!< data from voltage measurement pins; has to be calculated */    
    tUInt16 ui16Power;                        /*!< data from voltage power pins; has to be calculated */
} tVoltageData;

//actuator packages:


/*! the ARDUINO identifier for watchdog toggle */
const tUInt8 ID_ARD_ACT_WD_TOGGLE = 0x72;
/*! the tag for the watchdog toggle data*/
typedef struct tag_WatchdogToggleData
{
    tUInt8 ui8Toggle;                        /*!< flag for enable or disable */
} tWatchdogToggleData;


/*! the ARDUINO identifier for enabling watchdog */
const tUInt8 ID_ARD_ACT_WD_ENABLE = 0x71;
/*! the tag for the watchdog enable data*/
typedef struct tag_WatchdogEnableData
{
    tUInt8 ui8IsEnabled;                        /*!< flag for enable or disable */
} tWatchdogEnableData;

/*! the ARDUINO identifier for motor relais */
const tUInt8 ID_ARD_ACT_MOT_RELAIS = 0x73;
/*! the tag for the motor relais data*/
typedef struct tag_MotorRelaisData
{
    tUInt8 ui8IsEnabled;                        /*!< flag for enable or disable */
} tMotorRelaisData;


/*! the ARDUINO identifier for steering_servo actuator */
const tUInt8 ID_ARD_ACT_STEER_ANGLE = 0x63;


/*! the ARDUINO identifier for acceleration servo actuator */
const tUInt8 ID_ARD_ACT_ACCEL_SERVO = 0x64;
/*! the tag for the acceleration servo data*/
typedef struct tag_AccelerationServoData
{
    tUInt8 ui8Angle;                        /*!< angle of servo, range from 0...180, zero position is 90 */
} tAccelerationServoData;

/*! the ARDUINO identifier for enable or disable the lights*/
const tUInt8 ID_ARD_ACT_LIGHT_DATA = 0x65;

/*! these ids are used in the second byte to address the light */
const tUInt8 ID_ARD_ACT_LIGHT_DATA_HEAD = 0x01; 
const tUInt8 ID_ARD_ACT_LIGHT_DATA_BACK = 0x02; 
const tUInt8 ID_ARD_ACT_LIGHT_DATA_BRAKE = 0x04; 
const tUInt8 ID_ARD_ACT_LIGHT_DATA_TURNLEFT = 0x08; 
const tUInt8 ID_ARD_ACT_LIGHT_DATA_TURNRIGHT = 0x10;
const tUInt8 ID_ARD_ACT_LIGHT_DATA_HAZZARD = 0x18;
const tUInt8 ID_ARD_ACT_LIGHT_DATA_REVERSE = 0x20;


/*! the tag for the light data*/
typedef struct tag_LightData
{
    tUInt8 ui8LightMode;                /*!< SIGNAL Nr. Please see defines*/
    tUInt8 ui8IsEnabled;                /*!< flag for enable or disable */
} tLightData;



/*! the main frame around every data package which comes from the sensors and actuator*/
typedef struct tag_ArduinoHeader
{
    tUInt8   ui8SOF;                    /*!< the start of frame */
    tUInt8   ui8ID;                    /*!< identifier of package */
    tUInt32  ui32ArduinoTimestamp;        /*!< timestamp from arduino */
    tUInt8   ui8DataLength;            /*!< the length of the data */
} tArduinoHeader;

/*! the ARDUINO communication frame with its data area */
typedef union tag_ArduionDataUnion
{
    tSteeringAngleData      sSteeringData;        /*!< the steering angle data */
    tWheelEncoderData       sWheelEncData;        /*!< the wheel encoder data */
    tImuData                sImuData;            /*!< the inertial measurement unit data */
    tIrData                 sIrData;            /*!< the infrared data */
    tPhotoData              sPhotoData;            /*!< the photo sensor data */
    tUsData                 sUsData;            /*!< the ultra sonic sensor data */
    tVoltageData            sVoltageData;        /*!< the voltage data */
    tWatchdogToggleData     sWdToggleData;        /*!< the watchdog toggle data */
    tWatchdogEnableData     sWDEnableData;        /*!< the watchdog enable data */
    tMotorRelaisData        sMotorRelaisData;    /*!< the motor relais switch data */
    tAccelerationServoData  sAccServoData;        /*!< the acceleration servo data */
    tLightData              sLightData;            /*!< the light data */
} tArduinoDataUnion;

/*! the tag for the arduino frame*/
typedef struct tag_ArduinoFrame
{
    tArduinoHeader      sHeader;    /*!< the header part of the data */
    tArduinoDataUnion   sData;        /*!<  variable length contains data structs*/
} tArduinoFrame;


#pragma pack(pop)   /* restore original alignment from stack */

#endif // ARDUINOPROTOCOL_H







