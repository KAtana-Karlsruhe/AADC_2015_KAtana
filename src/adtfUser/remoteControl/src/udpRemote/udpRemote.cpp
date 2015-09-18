/**
 *
 * ADTF Empty Filter Demo.
 *
 * @file
 * Copyright &copy; Audi Electronics Venture GmbH. All rights reserved
 *
 * $Author: ELAMIHA $
 * $Date: 2014-03-27 16:40:09 +0100 (Thu, 27 Mar 2014) $
 * $Revision: 45736 $
 *
 * @remarks             This example shows how to implement a common adtf 
 *                      filter for processing data.
 *
 */
#include "stdafx.h"
#include "udpRemote.h"
#include <iostream>

ADTF_FILTER_PLUGIN("Demo UDP Filter Plugin", OID_ADTF_UDP_REMOTE, UdpRemote)

/**
 * Constructor.
 * @param pParent The parent filter
 */
UdpRemote::cReceiveThread::cReceiveThread():
        cKernelThread(),
        m_pParent(NULL)
{
}

tResult UdpRemote::cReceiveThread::SetParent(UdpRemote* pParent)
{
    m_pParent = pParent;
    RETURN_NOERROR;
}

/**
 * The thread function which will be called in a loop.
 * @return Standard Result Code.
 */
tResult UdpRemote::cReceiveThread::ThreadFunc()
{
    RETURN_IF_POINTER_NULL(m_pParent);
    return m_pParent->Receive();
}

/**
 *   Contructor. The cFilter contructor needs to be called !!
 *   The SetProperty in the constructor is necessary if somebody wants to deal with 
 *   the default values of the properties before init 
 *
 */
UdpRemote::UdpRemote(const tChar* __info) : cTimeTriggeredFilter(__info)
{
    m_alive_received = false;
    SetPropertyInt("ReceivePort", 8952);
}

/**
 *  Destructor. A Filter implementation needs always to have a virtual Destructor
 *              because the IFilter extends an IObject
 *
 */
UdpRemote::~UdpRemote()
{
}

/**
 *   The Filter Init Function. 
 *    eInitStage ... StageFirst ... should be used for creating and registering Pins 
 *               ... StageNormal .. should be used for reading the properies and initalizing 
 *                                  everything before pin connections are made 
 *   see {@link IFilter#Init IFilter::Init}.
 *
 */
tResult UdpRemote::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
      cObjectPtr<IMediaDescriptionManager> pDescManager;
      RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

      //output descriptor
      tChar const * strDescSignalValueOutput = pDescManager->GetMediaDescription("tSignalValue");
      RETURN_IF_POINTER_NULL(strDescSignalValueOutput);
       cObjectPtr<IMediaType> pTypeSignalValueOutput = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValueOutput,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
      RETURN_IF_FAILED(pTypeSignalValueOutput->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescSignalOutput));

      // Speed Output
      RETURN_IF_FAILED(m_output_speed.Create("accelerate", pTypeSignalValueOutput, static_cast<IPinEventSink*> (this)));
      RETURN_IF_FAILED(RegisterPin(&m_output_speed));

      // Steer Angle Output
      RETURN_IF_FAILED(m_output_steering.Create("steerAngle", pTypeSignalValueOutput, static_cast<IPinEventSink*> (this)));
      RETURN_IF_FAILED(RegisterPin(&m_output_steering));

      // Trigger TEST
      RETURN_IF_FAILED(m_opin_trigger_test.Create("test_trigger", new adtf::cMediaType(MEDIA_TYPE_STRUCTURED_DATA, MEDIA_SUBTYPE_STRUCT_STRUCTURED), static_cast<IPinEventSink*> (this)));
      RETURN_IF_FAILED(RegisterPin(&m_opin_trigger_test));

    }
    else if (eStage == StageNormal)
    {
        m_pBuffer = new int8_t[MAX_PACKET_SIZE];
        tInt nLocalPort = GetPropertyInt("ReceivePort");
        RETURN_IF_FAILED(m_oRecvSocket.Open(nLocalPort, 0));
        if (!m_oRecvSocket.SetTimeout(100000))
        {
            RETURN_ERROR(ERR_UNEXPECTED);
        }
        m_oReceiveThread.SetParent(this);
        m_oReceiveThread.Create();

        // Cycle interval
        RETURN_IF_FAILED(SetInterval(150*1000));
    }
    RETURN_NOERROR;
}

/**
 *   The Filters Start Function. see {@link IFilter#Start IFilter::Start}.
 *
 */
tResult UdpRemote::Start(__exception)
{
    m_oReceiveThread.Run();
    return cTimeTriggeredFilter::Start(__exception_ptr);
}

/**
 *   The Filters Stop Function. see {@link IFilter#Stop IFilter::Stop}.
 *
 */
tResult UdpRemote::Stop(__exception)
{
    m_oReceiveThread.Suspend();
    return cTimeTriggeredFilter::Stop(__exception_ptr);
}

/**
 *   The Filters Shutdown Function. see {@link IFilter#Shutdown IFilter::Shutdown}.
 *
 */
tResult UdpRemote::Shutdown(tInitStage eStage, __exception)
{
    if (eStage == StageNormal)
    {
        m_oReceiveThread.Release();
        m_oRecvSocket.Close();

        delete [] m_pBuffer;
        m_pBuffer = NULL;
    }
    return cTimeTriggeredFilter::Shutdown(eStage, __exception_ptr);
}

/**
 * This function gets called by the thread function.
 * \note Read must have a timeout, otherwise the thread would block forever.
 * @return Standard Result Code
 */
tResult UdpRemote::Receive()
{
    tInt nBytesReceived;
    // one read removes one packet from the receive queue (no matter of the size specified)
    if (IS_OK(m_oRecvSocket.Read(m_pBuffer, MAX_PACKET_SIZE, &nBytesReceived)))
    {
      if (nBytesReceived > 0 && nBytesReceived < 3)
      {
         if (m_pBuffer[0] == CMD_ALIVE)
           m_alive_received = true;
         else if (m_pBuffer[0] == CMD_TRIGGER && nBytesReceived == 2)
         {
           outputTrigger(m_pBuffer[1]);
         }
      }
      else if (nBytesReceived >= 3)
      {
        if (m_pBuffer[0] == CMD_CONTROL)
        {
          m_alive_received = true;
          outputSpeed(m_pBuffer[1]);
          outputSteering(m_pBuffer[2]);
        }
      }


        /*cObjectPtr<IMediaSample> pNewSample;
        if (IS_OK(_runtime->CreateInstance(OID_ADTF_MEDIA_SAMPLE, IID_ADTF_MEDIA_SAMPLE, (tVoid**) &pNewSample)))
        {
            cObjectPtr<ISerializable> pSerializable;
            if (IS_OK(pNewSample->GetInterface(IID_SERIALIZABLE, (tVoid**) &pSerializable)))
            {
                cStream oStream(cStream::INPUT_STREAM);
                oStream.AttachReference(m_pBuffer, nBytesReceived);
                if (IS_OK(pSerializable->Deserialize(&oStream)))
                {
                    m_oOutput.Transmit(pNewSample);
                }
                oStream.DetachReference();
            }
        }*/
    }

    RETURN_NOERROR;
}

void UdpRemote::outputSpeed(int8_t val)
{
  if (abs(val) > 100)
    return;

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
  cObjectPtr<IMediaCoder> pCoder;
  m_pCoderDescSignalOutput->WriteLock(pMediaSample, &pCoder);

  pCoder->Set("f32Value", (tVoid*)&(flValue));
  pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
  m_pCoderDescSignalOutput->Unlock(pCoder);

  //transmit media sample over output pin
  pMediaSample->SetTime(_clock->GetStreamTime());
  m_output_speed.Transmit(pMediaSample);
}

void UdpRemote::outputSteering(int8_t val)
{
  if (abs(val) > 90)
    return;

  tFloat32 flValue= ((tFloat32)val)*35.f/((tFloat32)100);
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
  cObjectPtr<IMediaCoder> pCoder;
  m_pCoderDescSignalOutput->WriteLock(pMediaSample, &pCoder);

  pCoder->Set("f32Value", (tVoid*)&(flValue));
  pCoder->Set("ui32ArduinoTimestamp", (tVoid*)&timeStamp);
  m_pCoderDescSignalOutput->Unlock(pCoder);

  //transmit media sample over output pin
  pMediaSample->SetTime(_clock->GetStreamTime());
  m_output_steering.Transmit(pMediaSample);
}

tResult UdpRemote::Cycle(__exception)
{
  if (m_alive_received)
  {
    m_alive_received = false;
    RETURN_NOERROR;
  }
  outputSpeed(0);
  RETURN_NOERROR;
}

tResult UdpRemote::outputTrigger(u_int8_t value)
{
  std::cout <<"RECEIVED Trigger signal " <<value <<std::endl;

  cObjectPtr<IMediaSample> pMediaSample;
  RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));
  RETURN_IF_FAILED(pMediaSample->AllocBuffer(sizeof(u_int8_t)));

  // write status byte
  pMediaSample->CopyBufferFrom(&value, sizeof(u_int8_t), 0, 0);

  // set stream time
  pMediaSample->SetTime(_clock->GetStreamTime());

  // transmit
  m_opin_trigger_test.Transmit(pMediaSample);

  RETURN_NOERROR;
}
