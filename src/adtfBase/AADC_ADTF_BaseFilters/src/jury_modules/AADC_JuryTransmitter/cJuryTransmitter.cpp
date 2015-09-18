/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.
 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: forchhe#$  $Date:: 2014-09-11 10:10:54#$ $Rev:: 25921   $
**********************************************************************/

#include "stdafx.h"
#include "cJuryTransmitter.h"



ADTF_FILTER_PLUGIN("AADC Jury Module", __guid, cJuryTransmitter);




cJuryTransmitter::cJuryTransmitter(const tChar* __info) : 
QObject(),
cBaseQtFilter(__info),
m_bDebugModeEnabled(false),
m_last_entryId(-1)
{
    m_last_timestamp=0,
    m_last_entryId=0,
    SetPropertyBool("Debug Output to Console",false);

    SetPropertyInt("Send time interval in sec",1);

    SetPropertyStr("ManeuverFile", "");
    SetPropertyBool("ManeuverFile" NSSUBPROP_FILENAME, tTrue); 
    SetPropertyStr("ManeuverFile" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
}

cJuryTransmitter::~cJuryTransmitter()
{
}

tHandle cJuryTransmitter::CreateView()
{
    QWidget* pWidget = (QWidget*)m_pViewport->VP_GetWindow();
        m_pWidget = new DisplayWidget(pWidget);
   
    connect(m_pWidget, SIGNAL(sendNotAus()), this, SLOT(OnNotAus()));
    connect(m_pWidget, SIGNAL(sendStart(tInt16)), this, SLOT(OnStart(tInt16)));
    connect(m_pWidget, SIGNAL(sendStop(tInt16)), this, SLOT(OnStop(tInt16)));
    connect(m_pWidget, SIGNAL(sendReadyRequest(tInt16)), this, SLOT(OnRequestReady(tInt16)));
    connect(this, SIGNAL(sendDriverState(int, int)), m_pWidget, SLOT(OnDriverState(int, int)));    
    connect(this, SIGNAL(sendMessage(QString)), m_pWidget, SLOT(OnAppendText(QString)));
    
    
    return (tHandle)m_pWidget;
}

tResult cJuryTransmitter::ReleaseView()
{
    if (m_pWidget != NULL)
    {
        delete m_pWidget;
        m_pWidget = NULL;
    }
    RETURN_NOERROR;
}

tResult cJuryTransmitter::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Init(eStage, __exception_ptr));

    // pins need to be created at StageFirst
    if (eStage == StageFirst)
    {
        
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                             IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                             (tVoid**)&pDescManager,
                                             __exception_ptr));
        /*
        * the MediaDescription for <struct name="tJuryNotAusFlag" .../> has to exist in a description file (e.g. in $ADTF_DIR\description\ or $ADTF_DIR\src\examples\src\description
        * before (!) you start adtf_devenv !! if not: the Filter-Plugin will not loaded because cPin.Create() and so ::Init() failes !
        */
        // Select Output
        tChar const * strDesc2 = pDescManager->GetMediaDescription("tJuryStruct");
        RETURN_IF_POINTER_NULL(strDesc2);
        cObjectPtr<IMediaType> pType2 = new cMediaType(0, 0, 0, "tJuryStruct", strDesc2, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_JuryStructOutputPin.Create("Jury_Struct", pType2, this));
        RETURN_IF_FAILED(RegisterPin(&m_JuryStructOutputPin));
        RETURN_IF_FAILED(pType2->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescJuryStruct));

        // NotAus Output
        tChar const * strDesc1 = pDescManager->GetMediaDescription("tJuryEmergencyStop");
        RETURN_IF_POINTER_NULL(strDesc1);
        cObjectPtr<IMediaType> pType1 = new cMediaType(0, 0, 0, "tJuryEmergencyStop", strDesc1,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_NotAusOutputPin.Create("Emergency_Stop", pType1, this));
        RETURN_IF_FAILED(RegisterPin(&m_NotAusOutputPin));
        RETURN_IF_FAILED(pType1->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescNotAus));      
        
        // input Pin
        tChar const * strDesc4 = pDescManager->GetMediaDescription("tDriverStruct");
        RETURN_IF_POINTER_NULL(strDesc4);
        cObjectPtr<IMediaType> pType4 = new cMediaType(0, 0, 0, "tDriverStruct", strDesc4, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_DriverStructInputPin.Create("Driver_Struct", pType4, this));
        RETURN_IF_FAILED(RegisterPin(&m_DriverStructInputPin));
        RETURN_IF_FAILED(pType4->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescDriverStruct));      
        
        m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");
    }
    else if(eStage == StageNormal)
    {
  
    }
    else if(eStage == StageGraphReady)
    {
        loadManeuverList();
        m_pWidget->SetManeuverList(m_sectorList);
        m_pWidget->FillComboBox();
    }
    RETURN_NOERROR;
}

tResult cJuryTransmitter::Start(__exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Start(__exception_ptr));

    RETURN_NOERROR;
}

tResult cJuryTransmitter::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr/* =NULL */)
{
    if (nActivationCode == IRunnable::RUN_TIMER)
    {
        
//not aus timer
        if (pvUserData==&m_hTimerNotAusCounter)
        {
            m_hTimerNotAusCounter++;
            RETURN_IF_FAILED(sendEmergencyStop());

            // the signal was transmitted three times
            if (m_hTimerNotAusCounter >= 3 && m_hTimerNotAus)
                {
                    __synchronized_obj(m_oCriticalSectionTimerNotAus);
                    // Destroy the timer
                    tResult nResult = _kernel->TimerDestroy(m_hTimerNotAus);
                    if (IS_FAILED(nResult)) THROW_ERROR(nResult);
                }
        }
        
//start event timer
        if (pvUserData==&m_hTimerStartCounter)
        {
            m_hTimerStartCounter++;
            RETURN_IF_FAILED(sendJuryStruct(action_START,m_Start_entry_id));
                
            // the signal was transmitted three times
            if (m_hTimerStartCounter >= 3 && m_hTimerStart)
                {
                    __synchronized_obj(m_oCriticalSectionTimerStart);
                    // Destroy the timer
                    tResult nResult = _kernel->TimerDestroy(m_hTimerStart);
                    if (IS_FAILED(nResult)) THROW_ERROR(nResult);
                }
        }
//request ready event timer
        if (pvUserData==&m_hTimerRequestReadyCounter)
        {
            m_hTimerRequestReadyCounter++;
            RETURN_IF_FAILED(sendJuryStruct(action_GETREADY,m_Request_Ready_entry_id));
                
            // the signal was transmitted three times
            if (m_hTimerRequestReadyCounter >= 3 && m_hTimerRequestReady)
                {
                    __synchronized_obj(m_oCriticalSectionTimerRequestReady);
                    // Destroy the timer
                    tResult nResult = _kernel->TimerDestroy(m_hTimerRequestReady);
                    if (IS_FAILED(nResult)) THROW_ERROR(nResult);
                }
        }
//stop event timer
        if (pvUserData==&m_hTimerStopCounter)
        {
            m_hTimerStopCounter++;
            RETURN_IF_FAILED(sendJuryStruct(action_STOP,m_Stop_entry_id));
                
            // the signal was transmitted three times
            if (m_hTimerStopCounter >= 3 && m_hTimerStop)
                {
                    __synchronized_obj(m_oCriticalSectionTimerStop);
                    // Destroy the timer
                    tResult nResult = _kernel->TimerDestroy(m_hTimerStop);
                    if (IS_FAILED(nResult)) THROW_ERROR(nResult);
                }
        }

    }
    return cBaseQtFilter::Run(nActivationCode, pvUserData, szUserDataSize, __exception_ptr);
}

tResult cJuryTransmitter::Stop(__exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Stop(__exception_ptr));

    if (m_hTimerStop) 
        {       
            __synchronized_obj(m_oCriticalSectionTimerStop);
            _kernel->TimerDestroy(m_hTimerStop);
            m_hTimerStop = NULL;
        }
    
    if (m_hTimerNotAus) 
        {
            __synchronized_obj(m_oCriticalSectionTimerNotAus);
            _kernel->TimerDestroy(m_hTimerNotAus);
            m_hTimerNotAus = NULL;
        }

    if (m_hTimerStart)
        {
            __synchronized_obj(m_oCriticalSectionTimerStart);
            _kernel->TimerDestroy(m_hTimerStart);
            m_hTimerStart = NULL;
        }

    if (m_hTimerRequestReady)
        {
            __synchronized_obj(m_oCriticalSectionTimerRequestReady);
            _kernel->TimerDestroy(m_hTimerRequestReady);
            m_hTimerRequestReady = NULL;
        }

    RETURN_NOERROR;
}

tResult cJuryTransmitter::Shutdown(tInitStage eStage, __exception)
{ 
    RETURN_IF_FAILED(cBaseQtFilter::Shutdown(eStage, __exception_ptr));
    RETURN_NOERROR;
}

tResult cJuryTransmitter::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);

    if (nEventCode == IPinEventSink::PE_MediaSampleReceived && pSource == &m_DriverStructInputPin)
    {     
        tInt8 stateDriver = 0; 
        tInt16 entry = -1;  
        
        {   // focus for sample read lock  
            // read-out the incoming Media Sample
            __adtf_sample_read_lock_mediadescription(m_pCoderDescDriverStruct,pMediaSample,pCoder);           
        
            pCoder->Get("i8StateID", (tVoid*)&stateDriver);     
            pCoder->Get("i16ManeuverEntry", (tVoid*)&entry);  
        }
               
        //update the gui
        emit sendDriverState(stateDriver, entry);
        
        // set the first entry manually
        if (m_last_timestamp==0)
            {
             m_last_timestamp = pMediaSample->GetTime();
             m_last_entryId = entry;
            }

        //update the timeline
        if (m_last_entryId < entry || m_last_entryId == -1 || stateDriver == -1 || stateDriver == 2)
        {
            tTimeStamp timeDiff = (pMediaSample->GetTime() - m_last_timestamp)/1000;  //time in milliseconds
            QString Message;
            switch (stateCar(stateDriver))
            {
            case stateCar_READY:
                Message = "from " + QString::number(m_last_entryId) + " to " + QString::number(entry) + ": " + QString::number(timeDiff) + " msec: Ready";
                break;
            case stateCar_RUNNING:
                Message = "from " + QString::number(m_last_entryId) + " to " + QString::number(entry) + ": " + QString::number(timeDiff) + " msec: OK";
                break;
            case stateCar_COMPLETE:
                Message = "from " + QString::number(m_last_entryId) + " to " + QString::number(entry) + ": " + QString::number(timeDiff) + " msec: Complete";
                break;
            case stateCar_ERROR:
                Message = "from " + QString::number(m_last_entryId) + " to " + QString::number(entry) + ": " + QString::number(timeDiff) + " msec: Error";
                break;
            case stateCar_STARTUP:
                break;
            }
            m_last_timestamp = pMediaSample->GetTime();             
            m_last_entryId = entry;
            //update the gui
            sendMessage(Message);
        }
        if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Jury Module: State value: %i with index %i",stateDriver,entry));

    }
    RETURN_NOERROR;

}

tResult cJuryTransmitter::OnNotAus()
{
    m_hTimerNotAusCounter = 0;
    __synchronized_obj(m_oCriticalSectionTimerNotAus);
    m_hTimerNotAus = _kernel->TimerCreate(GetPropertyInt("Send time interval in sec")*1000000, 0, static_cast<IRunnable*>(this),
                                    NULL, &m_hTimerNotAusCounter, sizeof(m_hTimerNotAusCounter), 0, adtf_util::cString::Format("%s.timerNotAus", OIGetInstanceName()));

    RETURN_NOERROR;
}

tResult cJuryTransmitter::OnRequestReady(tInt16 entryId)
{    
    m_Request_Ready_entry_id = entryId;
    m_hTimerRequestReadyCounter= 0;
    __synchronized_obj(m_oCriticalSectionTimerRequestReady);
    // Create a timer that will trigger the signal generation every 2 sec.
    m_hTimerRequestReady = _kernel->TimerCreate(GetPropertyInt("Send time interval in sec")*1000000, 0, static_cast<IRunnable*>(this),
                                    NULL, &m_hTimerRequestReadyCounter, sizeof(m_hTimerRequestReadyCounter), 0, adtf_util::cString::Format("%s.timerRequestReady", OIGetInstanceName()));
    RETURN_NOERROR;
}

tResult cJuryTransmitter::OnStop(tInt16 entryId)
{
    m_Stop_entry_id = entryId;
    m_hTimerStopCounter = 0;
    __synchronized_obj(m_oCriticalSectionTimerStop);
    m_hTimerStop = _kernel->TimerCreate(GetPropertyInt("Send time interval in sec")*1000000, 0, static_cast<IRunnable*>(this),
                                    NULL, &m_hTimerStopCounter, sizeof(m_hTimerStopCounter), 0, adtf_util::cString::Format("%s.timerStop", OIGetInstanceName()));
    RETURN_NOERROR;
}

tResult cJuryTransmitter::OnStart(tInt16 entryId)
{
    m_Start_entry_id = entryId;
    m_hTimerStartCounter = 0;
    __synchronized_obj(m_oCriticalSectionTimerStart);
    m_hTimerStart = _kernel->TimerCreate(GetPropertyInt("Send time interval in sec")*1000000, 0, static_cast<IRunnable*>(this),
                                    NULL, &m_hTimerStartCounter, sizeof(m_hTimerStartCounter), 0, adtf_util::cString::Format("%s.timerStart", OIGetInstanceName()));

    RETURN_NOERROR;
}


tResult cJuryTransmitter::sendJuryStruct(juryActions actionID, tInt16 maneuverEntry)
{
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pCoderDescJuryStruct->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();

    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
    {   // focus for sample write lock
        __adtf_sample_write_lock_mediadescription(m_pCoderDescJuryStruct,pMediaSample,pCoder);

    
        pCoder->Set("i8ActionID", (tVoid*)&actionID);
        pCoder->Set("i16ManeuverEntry", (tVoid*)&maneuverEntry);    
    }
        
    RETURN_IF_FAILED(pMediaSample->SetTime(_clock->GetStreamTime()));
        
    RETURN_IF_FAILED(m_JuryStructOutputPin.Transmit(pMediaSample));

    if(m_bDebugModeEnabled) LOG_INFO(cString::Format("Jury Module: Send: Action %d, Maneuver %d", actionID, maneuverEntry));

    RETURN_NOERROR;
}

tResult cJuryTransmitter::loadManeuverList()
{

    m_maneuverListFile = GetPropertyStr("ManeuverFile");
    
    if (m_maneuverListFile.IsEmpty())
    {
        LOG_ERROR("Jury Module: Maneuver file not found");
        RETURN_ERROR(ERR_INVALID_FILE);
    }    
    
    ADTF_GET_CONFIG_FILENAME(m_maneuverListFile);
    
    m_maneuverListFile = m_maneuverListFile.CreateAbsolutePath(".");

    //Load file, parse configuration, print the data
   
    if (cFileSystem::Exists(m_maneuverListFile))
    {
        cDOM oDOM;
        oDOM.Load(m_maneuverListFile);        
        cDOMElementRefList oSectorElems;
        cDOMElementRefList oManeuverElems;

        //read first Sector Elem
        if(IS_OK(oDOM.FindNodes("AADC-Maneuver-List/AADC-Sector", oSectorElems)))
        {                
            //iterate through sectors
            for (cDOMElementRefList::iterator itSectorElem = oSectorElems.begin(); itSectorElem != oSectorElems.end(); ++itSectorElem)
            {
                //if sector found
                tSector sector;
                sector.id = (*itSectorElem)->GetAttributeUInt32("id");
                
                if(IS_OK((*itSectorElem)->FindNodes("AADC-Maneuver", oManeuverElems)))
                {
                    //iterate through maneuvers
                    for(cDOMElementRefList::iterator itManeuverElem = oManeuverElems.begin(); itManeuverElem != oManeuverElems.end(); ++itManeuverElem)
                    {
                        tAADC_Maneuver man;
                        man.id = (*itManeuverElem)->GetAttributeUInt32("id");
                        man.action = (*itManeuverElem)->GetAttribute("action");
                        sector.maneuverList.push_back(man);
                    }
                }
    
                m_sectorList.push_back(sector);
            }
        }
        if (oSectorElems.size() > 0)
        {
            LOG_INFO("Jury Module: Loaded Maneuver file successfully.");
        }
        else
        {
            LOG_ERROR("Jury Moduler: no valid Maneuver Data found!");
            RETURN_ERROR(ERR_INVALID_FILE);
        }
    }
    else
    {
        LOG_ERROR("Jury Module: no valid Maneuver File found!");
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    //OUTPUT DATA TO LOG
    //for(int i = 0; i < m_sectorList.size(); i++)
    //{
    //    LOG_INFO(cString::Format("Driver Module: Sector ID %d",m_sectorList[i].id));
    //    for(int j = 0; j < m_sectorList[i].maneuverList.size(); j++)
    //    {
    //        LOG_INFO(cString::Format("\tManeuver ID: %d", m_sectorList[i].maneuverList.at(j).id));
    //        LOG_INFO(cString::Format("\tManeuver action: %s", m_sectorList[i].maneuverList.at(j).action.GetPtr()));
    //    }
    //}
   
    RETURN_NOERROR;
}


tResult cJuryTransmitter::sendEmergencyStop()
{
    if(m_bDebugModeEnabled)  LOG_INFO("Jury Module: Emergency_Stop requested");
        cObjectPtr<IMediaSample> pMediaSample;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

        cObjectPtr<IMediaSerializer> pSerializer;
        m_pCoderDescNotAus->GetMediaSampleSerializer(&pSerializer);
        tInt nSize = pSerializer->GetDeserializedSize();

        RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
        tBool boolValue = true; 

        {   // focus for sample write lock
            __adtf_sample_write_lock_mediadescription(m_pCoderDescNotAus,pMediaSample,pCoder);

            pCoder->Set("bEmergencyStop", (tVoid*)&boolValue);  
        }
        pMediaSample->SetTime(_clock->GetStreamTime()); 

        m_NotAusOutputPin.Transmit(pMediaSample);
        RETURN_NOERROR;
}



