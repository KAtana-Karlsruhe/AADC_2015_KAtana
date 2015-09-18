/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.
 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: forchhe#$  $Date:: 2014-09-11 10:10:54#$ $Rev:: 25921   $
**********************************************************************/

/*! \brief Car Visualization
 *         
 *  Dieser Filter dient zum Remote Steuern des Fahrzeugs mittels GUI auf einem Notebook. Achtung: wurde bisher nur unter Windows 7 getestet.
 */

#ifndef _CAR_CONTROL_CONTROL_FILTER_HEADER
#define _CAR_CONTROL_CONTROL_FILTER_HEADER

#define __guid "adtf.aadc.aadc_car_control"

#include "stdafx.h"
#include "displaywidget.h"

namespace SensorDefinition
{
    /*! */
    enum IrSensorDefinition
    {
        IR_FRONT_CENTER_LONG = 0,        /**< */
        IR_FRONT_RIGHT_LONG,            /**< */
        IR_FRONT_LEFT_LONG,                /**< */
        IR_FRONT_CENTER_SHORT,            /**< */
        IR_FRONT_RIGHT_SHORT,            /**< */
        IR_FRONT_LEFT_SHORT,            /**< */
        IR_REAR_CENTER_SHORT,            /**< */
        IR_REAR_RIGHT_SHORT,            /**< */
        IR_REAR_LEFT_SHORT            /**< */
    };
    /*! */
    enum UsSensorDefinition
    {        
        US_FRONT_RIGHT=0,            /**< */
        US_FRONT_LEFT,                /**< */
        US_REAR_RIGHT,                /**< */    
        US_REAR_LEFT                /**< */
    };
    /*! */
    enum ACCSensorDefinition
    {
        ACC_X = 0,                    /**< */
        ACC_Y,                        /**< */
        ACC_Z                        /**< */
    };
    /*! */
    enum GYROSensorDefinition
    {
        GYRO_YAW = 0,                /**< */
        GYRO_PITCH,                    /**< */
        GYRO_ROLL,                    /**< */
        GYRO_TEMP                    /**< */
    };
    /*! */
    enum WheelSensorDefinition
    {
        WHEEL_LEFT = 0,                /**< */
        WHEEL_RIGHT,                /**< */
        WHEEL_FULL_STRUCT,            /**< */
        DISTANCE_LEFT,                /**< */
        DISTANCE_RIGHT                /**< */
    };
    /*! */
    enum VoltageSensorDefinition
    {
        VOLTAGE_MEASUREMENT = 0,    /**< */
        VOLTAGE_ENGINE                /**< */
    };
}
/*! */
struct tUSS {tFloat32 nDistanceFL; tFloat32 nDistanceFR; tFloat32 nDistanceRL;    tFloat32 nDistanceRR;};
/*! */
struct tIR {tFloat32 nBackS; tFloat32 nLeftBackS; tFloat32 nLeftFrontS; tFloat32 nLeftFrontL; tFloat32 nFrontS;    tFloat32 nFrontL; tFloat32 nRightFrontS; tFloat32 nRightFrontL; tFloat32 nRightBackS;};
/*! */
struct tPhoto {    tFloat32 nValue;};
/*! */
struct tGyro {tFloat32 fW; tFloat32 fX; tFloat32 fY; tFloat32 fZ;};
/*! */
struct tACCEL {tFloat32 fX; tFloat32 fY; tFloat32 fZ;};
/*! */
struct tLWS {tFloat32 nSteerAngle;};
/*! */
struct tRPMWHEEL {tFloat32 fRpmLeft; tFloat32 fRpmRight;};
/*! */
struct tCOUNTWHEEL {tFloat32 countWheelLeft; tFloat32 countWheelRight;};
/*! */
struct tVolt {tFloat32 v1; tFloat32 v2;};
/*! */
struct tReverseLight {tFloat32 nState;};
/*! */
struct tHeadLight {tFloat32 nState;};
/*! */
struct tTurnSignal {tFloat32 nState;};
/*! */
struct tBrakeLight {tFloat32 nState;};

/*! */
enum tGear
{
    Forward,    /**< */
    Backward    /**< */

};
/*! */
enum tSteerDirection
{
    Left,            /**< */
    Right,            /**< */
    Straightforward/**< */
};
/*! */
struct tSPEED
{
    tInt            nSpeed;                /**< */ // 0 (neutral) - 128 (fullspeed) 
    tGear            eGearShift;            /**< */ // enum Forward or Backward
    tBool            bBrake;                /**< */ // flag 
    tInt            nBrakePressure;        /**< */ // 1 - 128
};
/*! */
struct tSTEER
{
    tUInt            nAngleInPercent;    /**< */ // will be ignored if eSteerDirection is straightforward
    tSteerDirection    eSteerDirection;    /**< */ // enum Left, Right or Straightforward
};
/*!
* This filter is test filter to display all the sensor signals and to control the basic functions of the car
*/
class CarControlFilter : public QObject, public cBaseQtFilter, public IKeyEventHandler
{
    ADTF_DECLARE_FILTER_VERSION(__guid, "AADC Car Visualization", OBJCAT_Application, "Car Visualization", 1, 0, 2, "");    

    Q_OBJECT
    
signals:

    void signalUpdateUS(const int&, const float&, const bool&);
    void signalUpdateSteeringAngle(const float&);
    void signalUpdateLuminosityValue(const float&);
    void signalUpdateACCIndividual(const int&, const float&);
    void signalUpdateGYROIndividual(const int&, const float&);
    void signalUpdateRPMIndividual(const int&, const float&);
    void signalUpdateDistanceIndividual(const int&, const float&);
    void signalUpdateVoltageIndividual(const int&, const float&);
    void signalUpdateStatus(int);
    void signalUpdateIRIndividual(const int&, const double&);


public: // construction
    CarControlFilter(const tChar *);
    virtual ~CarControlFilter();

    // overrides cFilter
    virtual tResult Init(tInitStage eStage, __exception = NULL);
    virtual tResult Start(__exception = NULL);
    virtual tResult Stop(__exception = NULL);
    virtual tResult Shutdown(tInitStage eStage, __exception = NULL);


    /*!Sends a not aus signal when esc key is pressed*/
    tResult OnKeyEvent(tKeyEventCode eCode, tInt nParam1 = 0, tInt nParam2 = 0, tInt nFlags = 0, tVoid* pEventData = NULL);

    /*!Handles all the input from the arduino*/
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

    /*! Thread that sends a timed alive signal to the car*/
    tResult Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr=NULL);

    void slotSteering(int val);
    void slotSpeed(int val);
    void slotHeadlight(tHeadLight sHeadLight);
    void slotBrakelight(tBrakeLight sBrakeLight);
    void slotReverselight(tReverseLight sReverseLight);
    void slotTurnSignal(tTurnSignal sTurnSignal);
    
    int m_value;

protected: // Implement cBaseQtFilter

    /*! Creates the widget instance*/
    tHandle CreateView();

    /*! Destroys the widget instance*/
    tResult ReleaseView();

    tResult ProcessUSSInput(const SensorDefinition::UsSensorDefinition& sensor_def, tFloat32 signalValue );
    tResult ProcessACCInput(const SensorDefinition::ACCSensorDefinition& sensor_def, tFloat32 signalValue);
    tResult ProcessGyroInput(const SensorDefinition::GYROSensorDefinition& sensor_def, tFloat32 signalValue);
    tResult ProcessSteeringInput(tFloat32 signalValue);
    tResult ProcessLuminosityInput(tFloat32 signalValue);    
    tResult ProcessRPMInput(const SensorDefinition::WheelSensorDefinition& sensor_def, tFloat32 signalValue);
    tResult ProcessDistanceInput(const SensorDefinition::WheelSensorDefinition& sensor_def , tFloat32 signalValue);
    tResult ProcessVoltageInput(const SensorDefinition::VoltageSensorDefinition& sensor_def, tFloat32 signalValue);
    tResult ProcessIrInput(const SensorDefinition::IrSensorDefinition& sensor_def, tFloat32 signalValue);
    tResult TransmitTurnSignalLeftMessage(tBool state);
    tResult TransmitTurnSignalRightMessage(tBool state);
    tResult TransmitBrakeLightMessage(tBool state);
    tResult TransmitReverseLightMessage(tBool state);
    tResult TransmitHeadLightMessage(tBool state);
    
    tFloat32 getSignalValueFromMediaSample(IMediaSample* pMediaSample);

protected:

    /*! The displayed widget*/
    DisplayWidget *m_pWidget;

   
    //single value inputs
    cInputPin               m_pin_input_range_front_left;
    cInputPin               m_pin_input_range_front_right;
    cInputPin               m_pin_input_range_rear_left;
    cInputPin               m_pin_input_range_rear_right;

    cInputPin               m_pin_input_ir_front_center_long;
    cInputPin               m_pin_input_ir_front_center_short;
    cInputPin               m_pin_input_ir_front_left_long;
    cInputPin               m_pin_input_ir_front_left_short;
    cInputPin               m_pin_input_ir_front_right_long;
    cInputPin               m_pin_input_ir_front_right_short;

    cInputPin               m_pin_input_ir_rear_center_short;
    cInputPin               m_pin_input_ir_rear_left_short;
    cInputPin               m_pin_input_ir_rear_right_short;
    
    cInputPin               m_pin_input_accleration_x;
    cInputPin               m_pin_input_accleration_y;
    cInputPin               m_pin_input_accleration_z;
    //cInputPin               m_pin_input_acc_temp;
    
    cInputPin               m_pin_input_gyro_yaw;
    cInputPin               m_pin_input_gyro_pitch;
    cInputPin               m_pin_input_gyro_roll;
    

    cInputPin                m_pin_input_wheel_sensor_left;      //!< rpm value
    cInputPin                m_pin_input_wheel_sensor_right;     //!< rpm value
    cInputPin                m_pin_input_distance_left;          //!< traveld distance since boot
    cInputPin                m_pin_input_distance_right;         //!< traveld distance since boot

    cInputPin                m_pin_input_voltage_meassurement;   //!< Current voltage meassurement circuit
    cInputPin                m_pin_input_voltage_engine;         //!< Current voltage engine circuit

    cInputPin                m_pin_input_luminosity_sensor;      //!< Current luminosity meassurement
    cInputPin                m_pin_input_steering_angle_sensor;  //!< Current steering_angle meassurement
    

    cOutputPin                m_pin_output_accelerate;
    cOutputPin                m_pin_output_steer_angle;
    cOutputPin                m_pin_output_break_light_enabled;
    cOutputPin                m_pin_output_turn_left_enabled;
    cOutputPin                m_pin_output_turn_right_enabled;
    cOutputPin                m_pin_output_reverse_light_enabled;
    cOutputPin                m_pin_output_head_light_enabled;

    IGlobalKeyEventManager*     m_pKeyEventManager;

    tTimeStamp m_timeout;
    tHandle m_hTimer;
 
    tSPEED m_pGear;    
    
    /*! Coder Descriptor for the output pins*/
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalOutput;
    cObjectPtr<IMediaTypeDescription> m_pCoderDescBoolSignalOutput;
    
    /*! Coder Descriptor for the input pins*/
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalInput;
};

#endif
