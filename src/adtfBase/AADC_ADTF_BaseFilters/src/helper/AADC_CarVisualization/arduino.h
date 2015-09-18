
#ifndef _ARDUINO_H_
#define _ARDUINO_H_

#define MEDIA_TYPE_AADC 0xaadc


#define MEDIA_SUBTYPE_COM 0x1
#define MEDIA_SUBTYPE_RAW_SENSORDATA 0x2
#define MEDIA_SUBTYPE_SENSORDATA 0x3
#define MEDIA_SUBTYPE_RAW_AKTORDATA 0x4
#define MEDIA_SUBTYPE_AKTOR_BOOL_ENABLED 0x5


#define MEDIA_SUBTYPE_STEERANGLE 0xA1
#define MEDIA_SUBTYPE_ACCEL 0x81
#define MEDIA_SUBTYPE_USS 0x92
#define MEDIA_SUBTYPE_WHEELRPM 0xB1
#define MEDIA_SUBTYPE_WHEELCOUNTER 0xB2
#define MEDIA_SUBTYPE_VOLTAGE 0xC1
#define MEDIA_SUBTYPE_PHOTO 0xE1
#define MEDIA_SUBTYPE_IR 0xD1
#define MEDIA_SUBTYPE_GYRO 0x82


			
#pragma pack(push, 1)

typedef struct{
	tBool isEnabled;
}tAktorBool;

typedef struct{
	tFloat32 value;
}tAktorFloat;

typedef struct
{
	int nState;
}tReverseLight;

typedef struct
{
	int nState;
}tHeadLight;

typedef struct
{
	int nState;
}tTurnSignal;

typedef struct
{
	int nState;
}tBrakeLight;


typedef struct
{
	tFloat32 nDistanceFL;
	tFloat32 nDistanceFR;
	tFloat32 nDistanceRL;
	tFloat32 nDistanceRR;
}tUSS;

typedef struct
{
	tFloat32 nBackS;
	tFloat32 nLeftBackS;
	tFloat32 nLeftFrontS;
	tFloat32 nLeftFrontL;
	tFloat32 nFrontS;
	tFloat32 nFrontL;
	tFloat32 nRightFrontS;
	tFloat32 nRightFrontL;
	tFloat32 nRightBackS;
}tIR;

typedef struct
{
	tFloat32 nValue;
}tPhoto;

typedef struct
{
	tFloat32 fX;
	tFloat32 fY;
	tFloat32 fZ;
	tFloat32 fTempInDegree;
}tGyro;

typedef struct
{
	tFloat32 fX;
	tFloat32 fY;
	tFloat32 fZ;
}tACCEL;




typedef struct
{
	tFloat32 nSteerAngle;
}tLWS;

/*! rounds per minute*/
typedef struct
{
	tFloat32 fRpmLeft;
	tFloat32 fRpmRight;
}tRPMWHEEL;

/*!Counts of rounds -> distance*/
typedef struct
{
	tFloat32 countWheelLeft;
	tFloat32 countWheelRight;
}tCOUNTWHEEL;

typedef struct
{
	tFloat32 v1;
	tFloat32 v2;
}tVolt;

#pragma pack(pop)

#endif // _ARDUINO_H_
