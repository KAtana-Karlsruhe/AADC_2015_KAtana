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

#include "stdafx.h"
#include "CarVisualizationFilter.h"


ADTF_FILTER_PLUGIN("AADC Car Control", __guid, CarControlFilter);

//int CarControlFilter::counter = 0;
using namespace SensorDefinition;


CarControlFilter::CarControlFilter(const tChar* __info) : 
QObject(),
cBaseQtFilter(__info)
{
    m_timeout = 500;
    SetPropertyInt("Timeout (ms)", m_timeout);

    m_pGear.eGearShift = Forward;
}

CarControlFilter::~CarControlFilter()
{
}

tHandle CarControlFilter::CreateView()
{
    QWidget* pWidget = (QWidget*)m_pViewport->VP_GetWindow();
    m_pWidget = new DisplayWidget(pWidget, this);
    connect(this, SIGNAL(signalUpdateUS(const int&, const float&, const bool&)), m_pWidget, SLOT(slotUpdateUS(const int&, const float&, const bool&)), Qt::QueuedConnection);
    connect(this, SIGNAL(signalUpdateSteeringAngle(const float&)), m_pWidget, SLOT(slotUpdateSteeringAngle(const float&)), Qt::QueuedConnection);
    connect(this, SIGNAL(signalUpdateLuminosityValue(const float&)), m_pWidget, SLOT(slotUpdateLuminosityValue(const float&)), Qt::QueuedConnection);
    connect(this, SIGNAL(signalUpdateACCIndividual(const int&, const float&)), m_pWidget, SLOT(slotUpdateACCIndividual(const int&, const float&)), Qt::QueuedConnection);
    connect(this, SIGNAL(signalUpdateGYROIndividual(const int&, const float&)), m_pWidget, SLOT(slotUpdateGYROIndividual(const int&, const float&)), Qt::QueuedConnection);
    connect(this, SIGNAL(signalUpdateVoltageIndividual(const int&, const float&)), m_pWidget, SLOT(slotUpdateVoltageIndividual(const int&, const float&)), Qt::QueuedConnection);
    connect(this, SIGNAL(signalUpdateRPMIndividual(const int&, const float&)), m_pWidget, SLOT(slotUpdateRPMIndividual(const int&, const float&)), Qt::QueuedConnection);
    connect(this, SIGNAL(signalUpdateDistanceIndividual(const int&, const float&)), m_pWidget, SLOT(slotUpdateDistanceIndividual(const int&, const float&)), Qt::QueuedConnection);
    connect(this, SIGNAL(signalUpdateStatus(int)), m_pWidget, SLOT(slotUpdateStatus(int)), Qt::QueuedConnection);
    connect(this, SIGNAL(signalUpdateIRIndividual(const int&, const double&)), m_pWidget, SLOT(slotUpdateIRIndividual(const int& ,const double&)), Qt::QueuedConnection);
    return (tHandle)m_pWidget;
}

tResult CarControlFilter::ReleaseView()
{
    if (m_pWidget != NULL)
    {
        delete m_pWidget;
        m_pWidget = NULL;
    }
    RETURN_NOERROR;
}

tResult CarControlFilter::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Init(eStage, __exception_ptr));

    // pins need to be created at StageFirst
    if (eStage == StageFirst)
    {
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
        
        //input descriptor
        tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue);        
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalInput)); 
    
        //uss sensors
        RETURN_IF_FAILED(m_pin_input_range_front_left.Create("range_front_left", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_range_front_left));
        RETURN_IF_FAILED(m_pin_input_range_front_right.Create("range_front_right", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_range_front_right));
        RETURN_IF_FAILED(m_pin_input_range_rear_left.Create("range_rear_left", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_range_rear_left));
        RETURN_IF_FAILED(m_pin_input_range_rear_right.Create("range_rear_right", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_range_rear_right));

        //IR sensors
        //center front
        RETURN_IF_FAILED(m_pin_input_ir_front_center_long.Create("ir_front_center_long", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_ir_front_center_long));
        RETURN_IF_FAILED(m_pin_input_ir_front_center_short.Create("ir_front_center_short", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_ir_front_center_short));

        //front left 
        RETURN_IF_FAILED(m_pin_input_ir_front_left_long.Create("ir_front_left_long", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_ir_front_left_long));
        RETURN_IF_FAILED(m_pin_input_ir_front_left_short.Create("ir_front_left_short", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_ir_front_left_short));
        //front right
        RETURN_IF_FAILED(m_pin_input_ir_front_right_long.Create("ir_front_right_long", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_ir_front_right_long));
        RETURN_IF_FAILED(m_pin_input_ir_front_right_short.Create("ir_front_right_short", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_ir_front_right_short));
        //rear
        RETURN_IF_FAILED(m_pin_input_ir_rear_center_short.Create("ir_rear_center_short", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_ir_rear_center_short));
        RETURN_IF_FAILED(m_pin_input_ir_rear_left_short.Create("ir_rear_left_short", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_ir_rear_left_short));
        RETURN_IF_FAILED(m_pin_input_ir_rear_right_short.Create("ir_rear_right_short", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_ir_rear_right_short));

        //acc
        RETURN_IF_FAILED(m_pin_input_accleration_x.Create("acc_x", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_accleration_x));
        RETURN_IF_FAILED(m_pin_input_accleration_y.Create("acc_y", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_accleration_y));
        RETURN_IF_FAILED(m_pin_input_accleration_z.Create("acc_z", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_accleration_z));
        //RETURN_IF_FAILED(m_pin_input_acc_temp.Create("acc_temp", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        //RETURN_IF_FAILED(RegisterPin(&m_pin_input_acc_temp));
        //gyro
        RETURN_IF_FAILED(m_pin_input_gyro_yaw.Create("yaw", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_gyro_yaw));
        RETURN_IF_FAILED(m_pin_input_gyro_pitch.Create("pitch", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_gyro_pitch));
        RETURN_IF_FAILED(m_pin_input_gyro_roll.Create("roll", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_gyro_roll));
        //wheels
        RETURN_IF_FAILED(m_pin_input_wheel_sensor_left.Create("RPM_left", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_wheel_sensor_left));
        RETURN_IF_FAILED(m_pin_input_wheel_sensor_right.Create("RPM_right", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_wheel_sensor_right));
        //distance
        RETURN_IF_FAILED(m_pin_input_distance_left.Create("distance_left", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_distance_left));
        RETURN_IF_FAILED(m_pin_input_distance_right.Create("distance_right", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_distance_right));
        //voltage
        RETURN_IF_FAILED(m_pin_input_voltage_meassurement.Create("voltage_meassurement", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_voltage_meassurement));
        RETURN_IF_FAILED(m_pin_input_voltage_engine.Create("voltage_engine", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_voltage_engine));
        //photosensor
        RETURN_IF_FAILED(m_pin_input_luminosity_sensor.Create("luminosity", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_luminosity_sensor));
        RETURN_IF_FAILED(m_pin_input_steering_angle_sensor.Create("steering_angle", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_input_steering_angle_sensor));


        

//Output pins-------------------------

        tChar const * strDescSignalValueOutput = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValueOutput);        
        cObjectPtr<IMediaType> pTypeSignalValueOutput = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValueOutput,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
        RETURN_IF_FAILED(pTypeSignalValueOutput->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalOutput)); 
        
        tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");
        RETURN_IF_POINTER_NULL(strDescBoolSignalValue);        
        cObjectPtr<IMediaType> pTypeBoolSignalValueOutput = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
        RETURN_IF_FAILED(pTypeBoolSignalValueOutput->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescBoolSignalOutput)); 
        
        // Speed Output
        RETURN_IF_FAILED(m_pin_output_accelerate.Create("accelerate", pTypeSignalValueOutput, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_output_accelerate));
        
        // Steer Angle Output
        RETURN_IF_FAILED(m_pin_output_steer_angle.Create("steerAngle", pTypeSignalValueOutput, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_output_steer_angle));       

        
        // Headlight Output
        RETURN_IF_FAILED(m_pin_output_head_light_enabled.Create("headLightEnabled", pTypeBoolSignalValueOutput, this));
        RETURN_IF_FAILED(RegisterPin(&m_pin_output_head_light_enabled));
        
        // Turn signal Output
        RETURN_IF_FAILED(m_pin_output_turn_left_enabled.Create("turnSignalLeftEnabled", pTypeBoolSignalValueOutput, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_output_turn_left_enabled));
        RETURN_IF_FAILED(m_pin_output_turn_right_enabled.Create("turnSignalRightEnabled", pTypeBoolSignalValueOutput, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_output_turn_right_enabled));
                        
        // Brakelight Output
        RETURN_IF_FAILED(m_pin_output_break_light_enabled.Create("brakeLightEnabled", pTypeBoolSignalValueOutput, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_output_break_light_enabled));
        
        // Reverselight Output
        RETURN_IF_FAILED(m_pin_output_reverse_light_enabled.Create("reverseLightEnabled", pTypeBoolSignalValueOutput, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_pin_output_reverse_light_enabled));
        
    }
    else if(eStage == StageGraphReady)
    {
        cObjectPtr<IApplication> pApplication;
        if (IS_OK(_runtime->GetObject(OID_ADTF_APPLICATION, IID_ADTF_APPLICATION, (tVoid**)&pApplication)))
        {//setupt key event handler
            tHandle hKeyEventManager = NULL;
            cException oLocalEx;
            tResult nResult = pApplication->GetInternalHandle(ADTF_APPLICATION_KEYEVENTMANAGER_HANDLE, &hKeyEventManager, &oLocalEx);
            if (IS_OK(nResult))
               m_pKeyEventManager = static_cast<IGlobalKeyEventManager*>(hKeyEventManager);
        }
    }
    RETURN_NOERROR;
}

tResult CarControlFilter::Start(__exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Start(__exception_ptr));
    if (m_pKeyEventManager)
    {
        ucom::cException oLocalEx;
        if (IS_FAILED(m_pKeyEventManager->RegisterKeyEventHandler(this, 0, &oLocalEx)))
        {
            __catch_exception(oLocalEx)
            {
                //error occured, but we dont break up here
                LOG_EXCEPTION(oLocalEx);
            }
        }
    }
    m_timeout = GetPropertyInt("Timeout (ms)");
    THROW_IF_POINTER_NULL(_kernel);
    m_hTimer = _kernel->TimerCreate(m_timeout * 1000, 0, static_cast<IRunnable*>(this));
    RETURN_NOERROR;
}

tResult CarControlFilter::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr/* =NULL */)
{
    return cFilter::Run(nActivationCode, pvUserData, szUserDataSize, __exception_ptr);
}

tResult CarControlFilter::Stop(__exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Stop(__exception_ptr));
    if(m_pKeyEventManager)
        m_pKeyEventManager->UnregisterKeyEventHandler(this);
    _kernel->TimerDestroy(m_hTimer);
    m_hTimer = NULL;
    RETURN_NOERROR;
}

tResult CarControlFilter::Shutdown(tInitStage eStage, __exception)
{ 
   return cBaseQtFilter::Shutdown(eStage, __exception_ptr);
}

tResult CarControlFilter::OnKeyEvent(tKeyEventCode eCode, tInt nParam1, tInt nParam2, tInt nFlags, tVoid* pEventData)
{
    //key event received: key pressed
    if (eCode == IKeyEventHandler::EC_KeyDown)
    {
        LOG_INFO(cString::Format("Key-Code %d", nParam1));
    }
    RETURN_NOERROR;
}

tResult CarControlFilter::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);

    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {    
            //write values with zero
            tFloat32 signalValue = 0;
            tUInt32 timeStamp = 0;
            if (pMediaSample != NULL && m_pCoderDescSignalInput != NULL)
            {
                // read-out the incoming Media Sample
                __adtf_sample_read_lock_mediadescription(m_pCoderDescSignalInput, pMediaSample, pCoderInput);
                                
                //get values from media sample        
                pCoderInput->Get("f32Value", (tVoid*)&signalValue);
                pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            }                
            else
                RETURN_ERROR(ERR_FAILED);
            
            
            //USS
            if (pSource == &m_pin_input_range_front_left)
                ProcessUSSInput(US_FRONT_LEFT,signalValue);
            else if (pSource == &m_pin_input_range_front_right)
                ProcessUSSInput(US_FRONT_RIGHT,signalValue);
            else if (pSource == &m_pin_input_range_rear_right)
                ProcessUSSInput(US_REAR_RIGHT, signalValue);
            else if (pSource == &m_pin_input_range_rear_left)
                ProcessUSSInput(US_REAR_LEFT, signalValue);
            //IR Front
            else if (pSource == &m_pin_input_ir_front_center_long)    
                ProcessIrInput(IR_FRONT_CENTER_LONG,signalValue);
            else if (pSource == &m_pin_input_ir_front_center_short)    
                ProcessIrInput(IR_FRONT_CENTER_SHORT,signalValue);
            else if (pSource == &m_pin_input_ir_front_left_long)    
                ProcessIrInput(IR_FRONT_LEFT_LONG,signalValue);
            else if (pSource == &m_pin_input_ir_front_left_short)    
                ProcessIrInput(IR_FRONT_LEFT_SHORT,signalValue);
            else if (pSource == &m_pin_input_ir_front_right_long)    
                ProcessIrInput(IR_FRONT_RIGHT_LONG, signalValue);
            else if (pSource == &m_pin_input_ir_front_right_short)    
                ProcessIrInput(IR_FRONT_RIGHT_SHORT, signalValue);
            //IR Back
            else if (pSource == &m_pin_input_ir_rear_center_short)
                ProcessIrInput(IR_REAR_CENTER_SHORT, signalValue);
            else if (pSource == &m_pin_input_ir_rear_left_short)
                ProcessIrInput(IR_REAR_LEFT_SHORT, signalValue);
            else if (pSource == &m_pin_input_ir_rear_right_short)
                ProcessIrInput(IR_REAR_RIGHT_SHORT, signalValue);
            //ACC
            else if (pSource == &m_pin_input_accleration_x)
                ProcessACCInput(ACC_X, signalValue);
            else if (pSource == &m_pin_input_accleration_y)
                ProcessACCInput(ACC_Y, signalValue);
            else if (pSource == &m_pin_input_accleration_z)
                ProcessACCInput(ACC_Z, signalValue);

            //GYRO
            else if (pSource == &m_pin_input_gyro_yaw)
                ProcessGyroInput(GYRO_YAW, signalValue);
            else if (pSource == &m_pin_input_gyro_pitch)
                ProcessGyroInput(GYRO_PITCH, signalValue);
            else if (pSource == &m_pin_input_gyro_roll)
                ProcessGyroInput(GYRO_ROLL, signalValue);
            /*else if (pSource == &m_pin_input_acc_temp)
                ProcessGyroInput(GYRO_TEMP, signalValue);*/
            //Wheel Sensors
            else if (pSource == &m_pin_input_wheel_sensor_left)
                ProcessRPMInput(WHEEL_LEFT, signalValue);
            else if (pSource == &m_pin_input_wheel_sensor_right)
                ProcessRPMInput(WHEEL_RIGHT, signalValue);
            //Distance
            else if (pSource == &m_pin_input_distance_left)
                ProcessDistanceInput(DISTANCE_LEFT, signalValue);
            else if (pSource == &m_pin_input_distance_right)
                ProcessDistanceInput(DISTANCE_RIGHT, signalValue);
            //Voltage
            else if (pSource == &m_pin_input_voltage_meassurement)
                ProcessVoltageInput(VOLTAGE_MEASUREMENT, signalValue);
            else if (pSource == &m_pin_input_voltage_engine)
                ProcessVoltageInput(VOLTAGE_ENGINE, signalValue);
            //steering
            else if (pSource == &m_pin_input_steering_angle_sensor)
                ProcessSteeringInput(signalValue);
            //luminosity
            else if (pSource == &m_pin_input_luminosity_sensor)
                ProcessLuminosityInput(signalValue);                
    }
    RETURN_NOERROR;
}

tResult CarControlFilter::ProcessUSSInput(const SensorDefinition::UsSensorDefinition& sensor_def, tFloat32 signalValue )
{
    if(sensor_def == US_FRONT_LEFT)
    {        
        //process it
        emit signalUpdateUS(1, signalValue, true);        
    }
    if(sensor_def == US_FRONT_RIGHT)
    {
        //process it
        emit signalUpdateUS(2, signalValue, true);        
    }
    if(sensor_def == US_REAR_RIGHT)
    {   
        //process 
        emit signalUpdateUS(3, signalValue, true);;
    }
    if(sensor_def == US_REAR_LEFT)
    {
        //process 
        emit signalUpdateUS(4, signalValue, true);
    }
    
    RETURN_NOERROR;
}

tResult CarControlFilter::ProcessIrInput(const IrSensorDefinition& sensor_def, tFloat32 signalValue)
{
    emit signalUpdateIRIndividual((int)sensor_def, (double)(signalValue));    
    RETURN_NOERROR;
}

tResult CarControlFilter::ProcessACCInput(const ACCSensorDefinition& sensor_def, tFloat32 signalValue)
{
    emit signalUpdateACCIndividual((int)sensor_def,signalValue);
    RETURN_NOERROR;
}

tResult CarControlFilter::ProcessGyroInput(const GYROSensorDefinition& sensor_def, tFloat32 signalValue)
{
    emit signalUpdateGYROIndividual((int)sensor_def, signalValue);
    RETURN_NOERROR;
}

tResult CarControlFilter::ProcessRPMInput(const WheelSensorDefinition& sensor_def, tFloat32 signalValue)
{
    emit signalUpdateRPMIndividual((int)sensor_def, signalValue);
    RETURN_NOERROR;
}

tResult CarControlFilter::ProcessDistanceInput(const WheelSensorDefinition& sensor_def, tFloat32 signalValue)
{

    emit signalUpdateDistanceIndividual((int)sensor_def, signalValue);
    RETURN_NOERROR;
}

tResult CarControlFilter::ProcessVoltageInput(const VoltageSensorDefinition& sensor_def, tFloat32 signalValue)
{
    emit signalUpdateVoltageIndividual((int)sensor_def, signalValue);
    RETURN_NOERROR;
}

tResult CarControlFilter::ProcessSteeringInput(tFloat32 signalValue)
{
    emit signalUpdateSteeringAngle(signalValue);
    RETURN_NOERROR;
}

tResult CarControlFilter::ProcessLuminosityInput(tFloat32 signalValue)
{
    emit signalUpdateLuminosityValue(signalValue);
    RETURN_NOERROR;
}


void CarControlFilter::slotSteering(int val)
{
    tFloat32 flValue= ((tFloat32)val)*35.f/((tFloat32)100);
    tUInt32 timeStamp = 0;
                    
    //LOG_INFO(cString::Format("slot speed: %f",flValue));    
    
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSignalOutput->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSample->AllocBuffer(nSize);
       
    //write date to the media sample with the coder of the descriptor
    {
        __adtf_sample_write_lock_mediadescription(m_pCoderDescSignalOutput, pMediaSample, pCoder);
        
        pCoder->Set("f32Value", (tVoid*)&(flValue));    
        pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);    
    }
    //transmit media sample over output pin
    pMediaSample->SetTime(_clock->GetStreamTime());
    m_pin_output_steer_angle.Transmit(pMediaSample);
}


void CarControlFilter::slotSpeed(int val)
{
    tFloat32 flValue= (tFloat32)(val);
    tUInt32 timeStamp = 0;
                            
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescSignalOutput->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSample->AllocBuffer(nSize);
       
    //write date to the media sample with the coder of the descriptor
    {
        __adtf_sample_write_lock_mediadescription(m_pCoderDescSignalOutput, pMediaSample, pCoder);
        
        pCoder->Set("f32Value", (tVoid*)&(flValue));    
        pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);    
    }
    
    //transmit media sample over output pin
    pMediaSample->SetTime(_clock->GetStreamTime());
    m_pin_output_accelerate.Transmit(pMediaSample);
}



void CarControlFilter::slotHeadlight(tHeadLight sHeadLight)
{
    TransmitHeadLightMessage(tBool(sHeadLight.nState));
}

void CarControlFilter::slotReverselight(tReverseLight sReverseLight)
{
    TransmitReverseLightMessage(tBool(sReverseLight.nState));
}

void CarControlFilter::slotBrakelight(tBrakeLight sBrakeLight)
{
    TransmitBrakeLightMessage(tBool(sBrakeLight.nState));
}

void CarControlFilter::slotTurnSignal(tTurnSignal sTurnSignal)
{
    if (sTurnSignal.nState == 1)
        TransmitTurnSignalLeftMessage(true);
    else if (sTurnSignal.nState == 3)
        TransmitTurnSignalRightMessage(true);
    else if (sTurnSignal.nState == 0)
        {
        TransmitTurnSignalRightMessage(false);
        TransmitTurnSignalLeftMessage(false);
        }
}


tResult CarControlFilter::TransmitTurnSignalLeftMessage(tBool state)
{
    //write values with zero
    tUInt32 timeStamp = 0;
    
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescBoolSignalOutput->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSample->AllocBuffer(nSize);
       
    //write date to the media sample with the coder of the descriptor
    {
        __adtf_sample_write_lock_mediadescription(m_pCoderDescBoolSignalOutput, pMediaSample, pCoderOutput);
    
        pCoderOutput->Set("bValue", (tVoid*)&(state));    
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);    
    }

    //transmit media sample over output pin
    pMediaSample->SetTime(_clock->GetStreamTime());
    m_pin_output_turn_left_enabled.Transmit(pMediaSample);
    
    RETURN_NOERROR;
}

tResult CarControlFilter::TransmitTurnSignalRightMessage(tBool state)
{
    //write values with zero
    tUInt32 timeStamp = 0;
    
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescBoolSignalOutput->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSample->AllocBuffer(nSize);
       
    //write date to the media sample with the coder of the descriptor
    {

        __adtf_sample_read_lock_mediadescription(m_pCoderDescBoolSignalOutput, pMediaSample, pCoderOutput);

        pCoderOutput->Set("bValue", (tVoid*)&(state));    
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);    
    
    }

    //transmit media sample over output pin
    pMediaSample->SetTime(_clock->GetStreamTime());
    m_pin_output_turn_right_enabled.Transmit(pMediaSample);
    
    RETURN_NOERROR;
}

tResult CarControlFilter::TransmitBrakeLightMessage(tBool state)
{
    //write values with zero
    tUInt32 timeStamp = 0;
    
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescBoolSignalOutput->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSample->AllocBuffer(nSize);
       
    //write date to the media sample with the coder of the descriptor
    {
        __adtf_sample_read_lock_mediadescription(m_pCoderDescBoolSignalOutput, pMediaSample, pCoderOutput);
    
        pCoderOutput->Set("bValue", (tVoid*)&(state));    
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);    
    }

    //transmit media sample over output pin
    pMediaSample->SetTime(_clock->GetStreamTime());
    m_pin_output_break_light_enabled.Transmit(pMediaSample);
        
    RETURN_NOERROR;
}

tResult CarControlFilter::TransmitReverseLightMessage(tBool state)
{
    //write values with zero
    tUInt32 timeStamp = 0;
    
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescBoolSignalOutput->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSample->AllocBuffer(nSize);
       
    //write date to the media sample with the coder of the descriptor
    {
        __adtf_sample_write_lock_mediadescription(m_pCoderDescBoolSignalOutput, pMediaSample, pCoderOutput);
    
        pCoderOutput->Set("bValue", (tVoid*)&(state));    
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);    
    }

    //transmit media sample over output pin
    pMediaSample->SetTime(_clock->GetStreamTime());
    m_pin_output_reverse_light_enabled.Transmit(pMediaSample);

    RETURN_NOERROR;
}

tResult CarControlFilter::TransmitHeadLightMessage(tBool state)
{
    //write values with zero
    tUInt32 timeStamp = 0;
    
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescBoolSignalOutput->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSample->AllocBuffer(nSize);
       
    //write date to the media sample with the coder of the descriptor
    {
        __adtf_sample_write_lock_mediadescription(m_pCoderDescBoolSignalOutput, pMediaSample, pCoderOutput);
    
        pCoderOutput->Set("bValue", (tVoid*)&(state));    
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);    
    }

    //transmit media sample over output pin
    pMediaSample->SetTime(_clock->GetStreamTime());
    m_pin_output_head_light_enabled.Transmit(pMediaSample);

    RETURN_NOERROR;

}

tFloat32 CarControlFilter::getSignalValueFromMediaSample(IMediaSample* pMediaSample)
{
    //write values with zero
    tFloat32 value = 0;
    tUInt32 timeStamp = 0;   
    if (pMediaSample != NULL && m_pCoderDescSignalInput != NULL)
    {
        // read-out the incoming Media Sample
        __adtf_sample_read_lock_mediadescription(m_pCoderDescSignalInput, pMediaSample, pCoderInput);

               
        //get values from media sample        
        pCoderInput->Get("f32Value", (tVoid*)&value);
        pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);

    }
    else
    {
        LOG_ERROR("CarControlFilter: could not read incoming media sample");       
    }
    
    return value;         
}
