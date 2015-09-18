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
* $Author:: spiesra $  $Date:: 2014-09-16 13:29:48#$ $Rev:: 26104   $
**********************************************************************/


#include "stdafx.h"
#include "cRawSerialDevice.h"
#include "arduinoCommunicationFilter.h"


ADTF_FILTER_PLUGIN("AADC Arduino Communication", OID_ADTF_ARDUINOCOM_FILTER, cArduinoCOM)

const tInt MAX_COMPORT_BUFFER_SIZE = 1024;

cArduinoCOM::cArduinoCOM(const tChar* __info) : cFilter (__info), m_timeLastDataReceived(0)
{
    SetPropertyStr("COM Port", "/dev/ttyACM0");
    SetPropertyInt("Connection Time Out in sec", 3);
}

cArduinoCOM::~cArduinoCOM()
{
}

cArduinoCOM::cReceiveThread::cReceiveThread(): cKernelThread(), m_pParent(NULL)
{
}

tResult cArduinoCOM::cReceiveThread::SetParent(cArduinoCOM* pParent)
{
    m_pParent = pParent;
    RETURN_NOERROR;
}

tResult cArduinoCOM::cReceiveThread::ThreadFunc()
{
    RETURN_IF_POINTER_NULL(m_pParent);
    return m_pParent->ReceiveFrameFromSerial();
}

tResult cArduinoCOM::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

    if (eStage == StageFirst)
    {
        tResult nResult = CreateOutputPins(__exception_ptr);
        if (IS_FAILED(nResult)) THROW_ERROR_DESC(nResult, "Failed to create Output Pins");
        nResult = CreateInputPins(__exception_ptr);
        if (IS_FAILED(nResult)) THROW_ERROR_DESC(nResult, "Failed to create Input Pins");
    }
    else if (eStage == StageNormal)
    {
        m_timeOutSeconds = GetPropertyInt("Connection Time Out in sec");        
        //open Serial Device
        if (IS_OK(m_oSerialDevice.CheckForDevice(GetPropertyStr("COM Port"))))
        {
            if IS_FAILED(m_oSerialDevice.Open(GetPropertyStr("COM Port"),
                                    115200, cSerialDevice::SER_NOPARITY, 8, cSerialDevice::SER_ONESTOPBIT,
                                    MAX_COMPORT_BUFFER_SIZE, MAX_COMPORT_BUFFER_SIZE, m_timeOutSeconds * 1000))
                THROW_ERROR_DESC(ERR_DEVICE_IO, "Serial Communication: Cannot open device on com port");
        }
        else
        {
            THROW_ERROR_DESC(ERR_DEVICE_IO, "Serial Communication: Cannot find device on com port");
        }
        m_oReceiveThread.SetParent(this);
        m_oReceiveThread.Create();
    }
    RETURN_NOERROR;
}


tResult cArduinoCOM::Start(__exception)
{
    m_oReceiveThread.Run();
    return cFilter::Start(__exception_ptr);
}

tResult cArduinoCOM::Stop(__exception)
{
    m_oReceiveThread.Suspend();
    return cFilter::Stop(__exception_ptr);
}

tResult cArduinoCOM::Shutdown(tInitStage eStage, __exception)
{
    if(eStage == StageNormal)
    {            
        m_oReceiveThread.Release();
        m_oSerialDevice.Close();
    }
    return cFilter::Shutdown(eStage,__exception_ptr);
}

tResult cArduinoCOM::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {
        RETURN_IF_POINTER_NULL(pMediaSample);

        if (pSource == &m_oCOMInputPin)
            ReceiveMediaSample(pMediaSample);            
    }
    else if (nEventCode == IPinEventSink::PE_MediaTypeChanged && pSource != NULL)
    {
        cObjectPtr<IMediaType> pType;
        pSource->GetMediaType(&pType);
        if (pType != NULL)
        {
            cObjectPtr<IMediaTypeDescription> pMediaTypeDesc;
            RETURN_IF_FAILED(pType->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&pMediaTypeDesc));
            m_pCoderDescArduinoInputData = pMediaTypeDesc;                        
        }        
    }    
    RETURN_NOERROR;
}

tResult cArduinoCOM::CreateOutputPins(__exception)
{    
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    tChar const * strDescArduino = pDescManager->GetMediaDescription("tArduinoData");
    RETURN_IF_POINTER_NULL(strDescArduino);

    cObjectPtr<IMediaType> pTypeArduinoData = new cMediaType(0, 0, 0, "tArduinoData", strDescArduino,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
    RETURN_IF_FAILED(m_oCOMOutputPin.Create("COM_output", pTypeArduinoData, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(pTypeArduinoData->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescArduinoOutputData)); 

    RETURN_IF_FAILED(RegisterPin(&m_oCOMOutputPin));

    RETURN_NOERROR;
}

tResult cArduinoCOM::CreateInputPins(__exception)
{
    RETURN_IF_FAILED(m_oCOMInputPin.Create("COM_input", new cMediaType(0, 0, 0, "tArduinoData" ), static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oCOMInputPin));

        RETURN_NOERROR;
}

tResult cArduinoCOM::ReceiveFrameFromSerial()
{
    //buffer
    tUInt8 header[7];
    tUInt8 payload[32];
    cMemoryBlock::MemSet(header, 0x00, sizeof(header));
    cMemoryBlock::MemSet(payload, 0x00, sizeof(payload));
    
    tInt read = 0;
    tUInt32 bytesAvail = m_oSerialDevice.BytesAvailable();
    // length of largest data package
    if( bytesAvail > 40)
    {        
        m_timeLastDataReceived = GetStreamTime();        
        //look for a SOF flag indicating an header
        do
        {
            read = m_oSerialDevice.Read(&header[0], 1);
            if(read != 1) LOG_ERROR("Serial Communication: received data does not match read request");
        } while(header[0] != ID_ARD_SOF);
		
		//read the header with 6 bytes
        read = m_oSerialDevice.Read(header + 1, 6);
        if(read != 6) LOG_ERROR("Serial Communication: received data does not match read request");            

        //check whether length of data is valid
        if(header[6] <= 32)
        {
			//read the payload            
			m_oSerialDevice.Read(payload, header[6]);

            memcpy(&m_oReceivedFrame, header, 7);
            memcpy(((unsigned char*)&m_oReceivedFrame) + 7, payload, header[6]);

            //check frame
            if(m_oReceivedFrame.sHeader.ui8ID != ID_ARD_SENS_STEER_ANGLE &&
                m_oReceivedFrame.sHeader.ui8ID != ID_ARD_SENS_WHEELENC &&
                m_oReceivedFrame.sHeader.ui8ID != ID_ARD_SENS_IMU &&
                m_oReceivedFrame.sHeader.ui8ID != ID_ARD_SENS_IR &&
                m_oReceivedFrame.sHeader.ui8ID != ID_ARD_SENS_PHOTO &&
                m_oReceivedFrame.sHeader.ui8ID != ID_ARD_SENS_US &&
                m_oReceivedFrame.sHeader.ui8ID != ID_ARD_SENS_VOLTAGE &&
                m_oReceivedFrame.sHeader.ui8ID != ID_ARD_ACT_WD_TOGGLE &&
                m_oReceivedFrame.sHeader.ui8ID != ID_ARD_ACT_WD_ENABLE &&
                m_oReceivedFrame.sHeader.ui8ID != ID_ARD_ACT_MOT_RELAIS &&
                m_oReceivedFrame.sHeader.ui8ID != ID_ARD_ACT_STEER_ANGLE &&
                m_oReceivedFrame.sHeader.ui8ID != ID_ARD_ACT_ACCEL_SERVO &&
                m_oReceivedFrame.sHeader.ui8ID != ID_ARD_ACT_LIGHT_DATA &&
                m_oReceivedFrame.sHeader.ui8ID != ID_ARD_ACT_LIGHT_DATA_HEAD &&
                m_oReceivedFrame.sHeader.ui8ID != ID_ARD_ACT_LIGHT_DATA_BACK &&
                m_oReceivedFrame.sHeader.ui8ID != ID_ARD_ACT_LIGHT_DATA_BRAKE &&
                m_oReceivedFrame.sHeader.ui8ID != ID_ARD_ACT_LIGHT_DATA_TURNLEFT &&
                m_oReceivedFrame.sHeader.ui8ID != ID_ARD_ACT_LIGHT_DATA_TURNRIGHT &&
                m_oReceivedFrame.sHeader.ui8ID != ID_ARD_ACT_LIGHT_DATA_REVERSE)
            {
                LOG_ERROR("Serial Communication: received identifier does not match protocol");
            }
            
            //print out the frame to console
            /*const char *p = (const char*)&m_oReceivedFrame;
            for (unsigned int j = 0; j < sizeof(tArduinoFrame); j++)
            {
                printf("%02X ", p[j]) ;
            } 
            std::cout << std::endl;*/
        }
        else
        {
            LOG_ERROR("Serial Communication: length of frame does not match protocol");    
        }
        //end Check frame
        if(!IS_OK(SendMediaSample()))
        {
            LOG_ERROR("Could not send MediaSample from serial data");
        }

        RETURN_NOERROR;
    }
    else if (bytesAvail == 0)
    {                 
        //checks if some data was received in time specified by the property 
        if ((GetStreamTime()-m_timeLastDataReceived)>m_timeOutSeconds*1000000) 
        {
            LOG_ERROR("No frames received on serial interface"); 
            m_timeLastDataReceived = GetStreamTime();
        }
		//wait 1 msec to block thread to prevent too much calls // the timeout with the serial device in this configuration is approx. 0,61 msec	
		adtf_util::cSystem::Sleep(1000);
    }
    else
	{
	//wait 1 msec to block thread to prevent to much calls // // the timeout with the serial device in this configuration is approx. 0,61 msec		 
	adtf_util::cSystem::Sleep(1000);
    }

    RETURN_NOERROR;
}

tResult cArduinoCOM::SendMediaSample()
{    
    //create new media sample
    cObjectPtr<IMediaSample> pMediaSample;
    AllocMediaSample((tVoid**)&pMediaSample);

    //allocate memory with the size given by the descriptor
    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescArduinoOutputData->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();
    pMediaSample->AllocBuffer(nSize);

    //write date to the media sample with the coder of the descriptor
    
    {   // focus for sample write lock
        //write date to the media sample with the coder of the descriptor
        __adtf_sample_write_lock_mediadescription(m_pCoderDescArduinoOutputData,pMediaSample,pCoderOutput);

        pCoderOutput->Set("ui8SOF", (tVoid*)&(m_oReceivedFrame.sHeader.ui8SOF));    
        pCoderOutput->Set("ui8ID", (tVoid*)&(m_oReceivedFrame.sHeader.ui8ID));
        pCoderOutput->Set("ui32ArduinoTimestamp", (tVoid*)&(m_oReceivedFrame.sHeader.ui32ArduinoTimestamp));

        pCoderOutput->Set("ui8DataLength", (tVoid*)&(m_oReceivedFrame.sHeader.ui8DataLength));
        pCoderOutput->Set("ui8Data", (tVoid*)&(m_oReceivedFrame.sData));
    }  
    
    //transmit media sample over output pin
    pMediaSample->SetTime(_clock->GetStreamTime());
    m_oCOMOutputPin.Transmit(pMediaSample);

    RETURN_NOERROR;
}

tResult cArduinoCOM::ReceiveMediaSample(IMediaSample *pMediaSample)
{                
    if (pMediaSample != NULL && m_pCoderDescArduinoInputData != NULL)
    {
        //write values with zero
        tUInt8 ui8SOF = 0;
        tUInt8 ui8ID = 0;
        tUInt32 ui32ArduinoTimestamp = 0;
        tUInt8 ui8DataLength = 0;
        //data cant be more than 18 bytes
        tUInt8 achFrameData[25];

        {   // focus for sample read lock
            // read-out the incoming Media Sample
            __adtf_sample_read_lock_mediadescription(m_pCoderDescArduinoInputData,pMediaSample,pCoder);

            //get values from media sample        
            pCoder->Get("ui8SOF", (tVoid*)&ui8SOF);    
            pCoder->Get("ui8ID", (tVoid*)&ui8ID);
            pCoder->Get("ui32ArduinoTimestamp", (tVoid*)&ui32ArduinoTimestamp);
            pCoder->Get("ui8DataLength", (tVoid*)&ui8DataLength);
            pCoder->Get("ui8Data", (tVoid*)&(achFrameData));
        }

        // construct frame
        tArduinoFrame frameToSend;

        //construct header
        frameToSend.sHeader.ui8SOF = ui8SOF;
        frameToSend.sHeader.ui8ID = ui8ID;
        frameToSend.sHeader.ui32ArduinoTimestamp = 0;
        frameToSend.sHeader.ui8DataLength = ui8DataLength;   

        //constrcut payload (dataFrame)
        memcpy(((tUInt8*)&frameToSend) + 7, achFrameData, ui8DataLength);            

        /*const char *p = (const char*)&frameToSend;
        for (unsigned int j = 0; j < sizeof(tArduinoFrame); j++)
        {
            printf("%02X ", p[j]) ;
        }printf("\n");*/

        if (!(IS_OK(WriteFrameToSerial((tUInt8*)&frameToSend, ui8DataLength + sizeof(tArduinoHeader)))))
        {
            LOG_ERROR("Serial Communcation: Could not send frame on serial interface");            
            RETURN_ERROR(ERR_DEVICE_IO);
        }
    }
    RETURN_NOERROR;
}

tResult cArduinoCOM::WriteFrameToSerial(tUInt8* frame, int bytesToWrite)
{
    if(m_oSerialDevice.Write(frame, bytesToWrite) == bytesToWrite) RETURN_NOERROR;
    RETURN_ERROR(ERR_DEVICE_IO);
}

tTimeStamp cArduinoCOM::GetStreamTime()
{
    return (_clock != NULL) ? _clock->GetStreamTime() : cSystem::GetTime();
}
