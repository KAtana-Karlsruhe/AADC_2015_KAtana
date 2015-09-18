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

// arduinofilter.cpp : Definiert die exportierten Funktionen für die DLL-Anwendung.
//
#include "stdafx.h"
#include "cCubic.h"
#include "SensorCalibration.h"

ADTF_FILTER_PLUGIN("AADC Calibration Interpolation", OID_ADTF_SENSOR_CALIBRATION, SensorCalibration)

SensorCalibration::SensorCalibration(const tChar* __info) : cFilter(__info), m_bDebugModeEnabled(tFalse)
{

    SetPropertyInt("Interpolation", 1);
    SetPropertyStr("Interpolation" NSSUBPROP_VALUELISTNOEDIT, "1@linear|2@cubic|3@none");

    SetPropertyStr("Configuration File For Interpolation",""); 
    SetPropertyBool("Configuration File For Interpolation" NSSUBPROP_FILENAME, tTrue); 
    SetPropertyStr("Configuration File For Interpolation" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)"); 
    
    SetPropertyBool("Border Warnings to Console",tFalse);
    SetPropertyBool("Print initial table to Console",tFalse);
}

SensorCalibration::~SensorCalibration()
{
}

tResult SensorCalibration::CreateInputPins(__exception)
{    

    RETURN_IF_FAILED(m_oInput.Create("input_value", new cMediaType(0, 0, 0, "tSignalValue"), static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oInput));
    RETURN_NOERROR;
}

tResult SensorCalibration::CreateOutputPins(__exception)
{
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));
    
    tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
    RETURN_IF_POINTER_NULL(strDescSignalValue);        
    cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
    RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignal)); 
    
    RETURN_IF_FAILED(m_oOutput.Create("output_value", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&m_oOutput));
    RETURN_NOERROR;
}

tResult SensorCalibration::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst)
        {
                CreateInputPins(__exception_ptr);
                CreateOutputPins(__exception_ptr);
        }
    else if (eStage == StageNormal)
        {
            //load xml files for linear interpolation
            m_calibrationMode = GetPropertyInt("Interpolation");
            THROW_IF_FAILED(LoadConfigurationData());

            //create class for cubic spline interpolation
            if (m_calibrationMode==2)
                {
                    m_cubicInterpolation = new Cubic(tInt(m_xValues.size()),m_xValues,m_yValues);                
                }
            m_bDebugModeEnabled = GetPropertyBool("Border Warnings to Console");

        }
    else if(eStage == StageGraphReady)
        {    
            
        }
                
    RETURN_NOERROR;
}

tResult SensorCalibration::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult SensorCalibration::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}

tResult SensorCalibration::Shutdown(tInitStage eStage, __exception)
{
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult SensorCalibration::OnPinEvent(    IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{    
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {        
        if (pMediaSample != NULL && m_pCoderDescSignal != NULL)
        {

            //write values with zero
            tFloat32 value = 0;
            tUInt32 timeStamp = 0;
            {   // focus for sample read lock
                // read-out the incoming Media Sample
                __adtf_sample_read_lock_mediadescription(m_pCoderDescSignal,pMediaSample,pCoderInput);
                //get values from media sample        
                pCoderInput->Get("f32Value", (tVoid*)&value);
                pCoderInput->Get("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
             }
        
            // doing the calibration
            if (m_calibrationMode==1)
                value = getLinearInterpolatedValue(value);    
            else if (m_calibrationMode==2)
                value = getCubicSplineInterpolatedValue(value);
            //else if (GetPropertyInt("Interpolation", 1)==3)    //just for explanation
            //    value = value;
            
            //if (m_bDebugModeEnabled) LOG_INFO(cString::Format("Sensorfilter received: ID %x Value %f",ID,value));   
                
            //create new media sample
            cObjectPtr<IMediaSample> pNewMediaSample;
            AllocMediaSample((tVoid**)&pNewMediaSample);

            //allocate memory with the size given by the descriptor
            cObjectPtr<IMediaSerializer> pSerializer;
            m_pCoderDescSignal->GetMediaSampleSerializer(&pSerializer);
            tInt nSize = pSerializer->GetDeserializedSize();
            pNewMediaSample->AllocBuffer(nSize);
        
            {   // focus for sample write lock
                //write date to the media sample with the coder of the descriptor
                __adtf_sample_write_lock_mediadescription(m_pCoderDescSignal,pNewMediaSample,pCoderOutput);
    
                pCoderOutput->Set("f32Value", (tVoid*)&(value));    
                pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
            }    

            //transmit media sample over output pin
            pNewMediaSample->SetTime(pMediaSample->GetTime());
            m_oOutput.Transmit(pNewMediaSample);
        }
    }
    
    RETURN_NOERROR;
}

tResult SensorCalibration::LoadConfigurationData()
{
    //Get path of configuration file
      m_fileConfig = GetPropertyStr("Configuration File For Interpolation");
    if (m_fileConfig.IsEmpty())
        {
        LOG_ERROR("SensorCalibration: Configured configuration file not found");
        RETURN_ERROR(ERR_INVALID_FILE);
        }    
    ADTF_GET_CONFIG_FILENAME(m_fileConfig);
    m_fileConfig = m_fileConfig.CreateAbsolutePath(".");

    //Load file, parse configuration, print the data
   
    if (cFileSystem::Exists(m_fileConfig))
    {
        cDOM oDOM;
        oDOM.Load(m_fileConfig);        
        //load settings for calibration mode
        cDOMElementRefList oElemsSettings;
        if(IS_OK(oDOM.FindNodes("calibration/settings", oElemsSettings)))
            {                
             for (cDOMElementRefList::iterator itElem = oElemsSettings.begin(); itElem != oElemsSettings.end(); ++itElem)
                { 
                cDOMElement* pConfigElement;
                if (IS_OK((*itElem)->FindNode("mode", pConfigElement)))
                    {                       
                    
                    cString rdMode = pConfigElement->GetData();
                    LOG_INFO(adtf_util::cString::Format("SensorCalibration: %s",rdMode.GetPtr()));
                    if (cString::Compare(rdMode,"linear")==0)
                        m_calibrationMode=1;        
                    else if (cString::Compare(rdMode,"cubic")==0)
                        m_calibrationMode=2;
                    else if (cString::Compare(rdMode,"none")==0)
                        m_calibrationMode=3;
                    }            
                }
            }
        cString rdMode;
        switch (m_calibrationMode)
            {
            case 1:
                rdMode = "linear";
                break;
            case 2:
                rdMode = "cubic";
                break;
            case 3:
                rdMode = "none";
                break;
            }
        LOG_INFO(adtf_util::cString::Format("SensorCalibration: Calibration mode is %s",rdMode.GetPtr()));    
        //load supporting points
        if (m_calibrationMode!=3)
            {
            cDOMElementRefList oElems;
             if(IS_OK(oDOM.FindNodes("calibration/supportingPoints/point", oElems)))
                {                
                 for (cDOMElementRefList::iterator itElem = oElems.begin(); itElem != oElems.end(); ++itElem)
                    {

                    cDOMElement* pConfigElement;
                    if (IS_OK((*itElem)->FindNode("xValue", pConfigElement)))
                    {                   
                        m_xValues.push_back(tFloat32(cString(pConfigElement->GetData()).AsFloat64()));
                    }
                    if (IS_OK((*itElem)->FindNode("yValue", pConfigElement)))
                    {
                        m_yValues.push_back(tFloat32(cString(pConfigElement->GetData()).AsFloat64()));
                    }
                    }
                }        
            if (oElems.size() > 0)
                {                
                if (GetPropertyBool("Print initial table to Console"))
                {
                    for (tUInt i = 0; i<m_xValues.size();i++)
                        {
                        if (i>m_yValues.size()) break;
                        LOG_INFO(cString::Format("SensorCalibration: supportingPoint #%d: (%lf/%lf)",i,m_xValues[i],m_yValues[i]));
                        }
                    }
                }
            else
                {
                        LOG_ERROR("SensorCalibration: no supporting points in given file found!");
                        RETURN_ERROR(ERR_INVALID_FILE);
                }
            //checks if data are valid
            RETURN_IF_FAILED(CheckConfigurationData());
            }
    }
    else
    {
        LOG_ERROR("SensorCalibration: Configured configuration file not found");
        RETURN_ERROR(ERR_INVALID_FILE);
    }
    
    RETURN_NOERROR;
}

tResult SensorCalibration::CheckConfigurationData()
{
    //checks if the xValues of the calibration table are increasing
    for (vector<tFloat32>::iterator it = m_xValues.begin(); it != m_xValues.end() ; it++)
        {
            vector<tFloat32>::iterator it2 = it;
            it2++;
            if (it2 != m_xValues.end())
            {                
                if ((tFloat32(*it) > tFloat32(*it2)))
                    {
                    LOG_ERROR(cString::Format("SensorCalibration: The xValues in the file %s are not in increasing order. Please reorder the points!",m_fileConfig.GetPtr()));
                    RETURN_ERROR(ERR_INVALID_FILE);
                    }
            }
        }
    
    RETURN_NOERROR;
}

tFloat32 SensorCalibration::getLinearInterpolatedValue(tFloat32 inputValue)
{    
    if (inputValue<m_xValues.front())
        {
        if (m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("SensorCalibration: requested x-value %f is lower than smallest x-value in calibration table",inputValue));
        return m_yValues.front();
        }
    else if (inputValue>m_xValues.back())
        {
        if (m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("SensorCalibration: requested x-value %f is higher than highes x-value in calibration table",inputValue));
        return m_yValues.back();
        }

    tUInt index;
    if (m_xValues.size() > 2)
        {    
            for (index = 0; index<m_xValues.size();index++)
                {
                    if (m_xValues[index]>=inputValue) break;
                }
            if (index!=0) index = index -1;
            //index = std::distance(m_xValues.begin(), std::lower_bound(m_xValues.begin(), m_xValues.end(), inputValue));
        }
        
    else index = 0;
    
    if ((m_xValues[index+1]-m_xValues[index])!=0)
        {                
        tFloat32 value = (inputValue-m_xValues[index])*(m_yValues[index+1]-m_yValues[index])/(m_xValues[index+1]-m_xValues[index])+m_yValues[index];
        
        //tFloat64 security check
        if (value > *max_element(m_yValues.begin(),m_yValues.end() ))
            return *max_element(m_yValues.begin(),m_yValues.end()) ;
        else if (value < *min_element(m_yValues.begin(),m_yValues.end()) )
            return *min_element(m_yValues.begin(),m_yValues.end()) ;
        else
            return value;
        }
    else
        {
        LOG_ERROR("SensorCalibration: divison by zero in linear interpolation!");
        return 0;
        }
}

tFloat32 SensorCalibration::getCubicSplineInterpolatedValue(tFloat32 inputValue)
{    
    if (inputValue<m_xValues.front())
        {
        if (m_bDebugModeEnabled) LOG_WARNING("SensorCalibration: requested x-value lower than smallest x-value in calibration table");
        return m_yValues.front();
        }
    else if (inputValue>m_xValues.back())
        {
        if (m_bDebugModeEnabled) LOG_WARNING("SensorCalibration: requested x-value higher than highes x-value in calibration table");
        return m_yValues.back();
        }
        
    return tFloat32(m_cubicInterpolation->getValue(tFloat64(inputValue)));
}
