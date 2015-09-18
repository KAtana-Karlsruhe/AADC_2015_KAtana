/**
 *
 * ADTF Stering Calibration Filter
 *
 */
#include "stdafx.h"
#include "SteeringCalibration.h"

/// Create filter shell
ADTF_FILTER_PLUGIN("KATANA Steering Calibration Filter", OID_ADTF_TEMPLATE_FILTER, cSteeringCalibrationFilter);


cSteeringCalibrationFilter::cSteeringCalibrationFilter(const tChar* __info):cFilter(__info)
{
  	m_maxValue = 100;
	SetPropertyFloat("Max Value",m_maxValue);
	SetPropertyFloat("Max Value" NSSUBPROP_REQUIRED, tTrue);
}

cSteeringCalibrationFilter::~cSteeringCalibrationFilter()
{

}

tResult cSteeringCalibrationFilter::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))
    
    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {
        CreateInputPins(__exception_ptr);
	CreateOutputPins(__exception_ptr);
    }

    m_maxValue = tFloat32(GetPropertyFloat("Max Value"));
    RETURN_NOERROR;
}

tResult cSteeringCalibrationFilter::CreateInputPins(__exception)
{	
    
	RETURN_IF_FAILED(m_oInput.Create("steeringIn", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_oInput));
	RETURN_NOERROR;
}

tResult cSteeringCalibrationFilter::CreateOutputPins(__exception)
{
	cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
    
	tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);        
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);	
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal)); 
	
	RETURN_IF_FAILED(m_oOutput.Create("steeringOut", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_oOutput));

    RETURN_NOERROR;
}

tResult cSteeringCalibrationFilter::Shutdown(tInitStage eStage, __exception)
{
    // call the base class implementation
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cSteeringCalibrationFilter::OnPinEvent(	IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{	
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {		
		if (pMediaSample != NULL && m_pCoderDescSignal != NULL)
        {
        // read-out the incoming Media Sample
        cObjectPtr<IMediaCoder> pCoderInput;
        RETURN_IF_FAILED(m_pCoderDescSignal->Lock(pMediaSample, &pCoderInput));

        //write values with zero
        tFloat32 value = 0;
        tUInt32 timeStamp = 0;
               
        //get values from media sample        
        pCoderInput->Get("f32Value", (tVoid*)&value);
        pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
        m_pCoderDescSignal->Unlock(pCoderInput);       
        
	// delete upper values
        if(value > m_maxValue) {
	  value = m_maxValue;
	} else if(value < (-1)*m_maxValue) {
	    value = (-1)*m_maxValue;
	}
      
        //create new media sample
        cObjectPtr<IMediaSample> pMediaSample;
        AllocMediaSample((tVoid**)&pMediaSample);

        //allocate memory with the size given by the descriptor
        cObjectPtr<IMediaSerializer> pSerializer;
        m_pCoderDescSignal->GetMediaSampleSerializer(&pSerializer);
        tInt nSize = pSerializer->GetDeserializedSize();
        pMediaSample->AllocBuffer(nSize);
           
        //write date to the media sample with the coder of the descriptor
        cObjectPtr<IMediaCoder> pCoderOutput;
        m_pCoderDescSignal->WriteLock(pMediaSample, &pCoderOutput);	
	
        pCoderOutput->Set("f32Value", (tVoid*)&(value));	
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);    
        m_pCoderDescSignal->Unlock(pCoderOutput);

        //transmit media sample over output pin
        pMediaSample->SetTime(pMediaSample->GetTime());
        m_oOutput.Transmit(pMediaSample);
        }
	}
	
	RETURN_NOERROR;
}
