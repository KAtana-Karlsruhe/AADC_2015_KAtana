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
#include "cDriverFilter.h"


ADTF_FILTER_PLUGIN("AADC Jury Module", __guid, cDriverFilter);




cDriverFilter::cDriverFilter(const tChar* __info) : 
QObject(),
cBaseQtFilter(__info),
m_bDebugModeEnabled(false)
{
    SetPropertyBool("Debug Output to Console",false);

    SetPropertyStr("ManeuverFile", "");
    SetPropertyBool("ManeuverFile" NSSUBPROP_FILENAME, tTrue); 
    SetPropertyStr("ManeuverFile" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)"); 
}

cDriverFilter::~cDriverFilter()
{
}

tHandle cDriverFilter::CreateView()
{
    QWidget* pWidget = (QWidget*)m_pViewport->VP_GetWindow();
    m_pWidget = new DisplayWidgetDriver(pWidget);        

    connect(m_pWidget, SIGNAL(sendStruct(stateCar, tInt16)), this, SLOT(OnSendState(stateCar, tInt16)));
    connect(this, SIGNAL(sendRun(int)), m_pWidget, SLOT(OnDriverGo(int)));
    connect(this, SIGNAL(sendStop(int)), m_pWidget, SLOT(OnDriverStop(int)));
    connect(this, SIGNAL(sendRequestReady(int)), m_pWidget, SLOT(OnDriverRequestReady(int)));

    return (tHandle)m_pWidget;
}

tResult cDriverFilter::ReleaseView()
{
    if (m_pWidget != NULL)
    {
        delete m_pWidget;
        m_pWidget = NULL;
    }
    RETURN_NOERROR;
}

tResult cDriverFilter::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Init(eStage, __exception_ptr));

    // pins need to be created at StageFirst
    if (eStage == StageFirst)    {
        
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                             IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                             (tVoid**)&pDescManager,
                                             __exception_ptr));

	// input jury struct
        tChar const * strDesc1 = pDescManager->GetMediaDescription("tJuryStruct");
        RETURN_IF_POINTER_NULL(strDesc1);
        cObjectPtr<IMediaType> pType1 = new cMediaType(0, 0, 0, "tJuryStruct", strDesc1, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_JuryStructInputPin.Create("Jury_Struct", pType1, this));
        RETURN_IF_FAILED(RegisterPin(&m_JuryStructInputPin));
        RETURN_IF_FAILED(pType1->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescJuryStruct));
        
        // output driver struct
        tChar const * strDesc2 = pDescManager->GetMediaDescription("tDriverStruct");
        RETURN_IF_POINTER_NULL(strDesc2);
        cObjectPtr<IMediaType> pType2 = new cMediaType(0, 0, 0, "tDriverStruct", strDesc2, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_DriverStructOutputPin.Create("Driver_Struct", pType2, this));
        RETURN_IF_FAILED(RegisterPin(&m_DriverStructOutputPin));
        RETURN_IF_FAILED(pType2->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pCoderDescDriverStruct));
        
        m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");
    }
    else if(eStage == StageGraphReady)
    {
        loadManeuverList();
        m_pWidget->SetManeuverList(m_sectorList);
        m_pWidget->ShowManeuverList();
        m_pWidget->FillComboBox();
    }
    RETURN_NOERROR;
}

tResult cDriverFilter::Start(__exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Start(__exception_ptr));
    

    RETURN_NOERROR;
}

tResult cDriverFilter::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr/* =NULL */)
{

    return cBaseQtFilter::Run(nActivationCode, pvUserData, szUserDataSize, __exception_ptr);
}

tResult cDriverFilter::Stop(__exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Stop(__exception_ptr));
    
    RETURN_NOERROR;
}

tResult cDriverFilter::Shutdown(tInitStage eStage, __exception)
{ 
    RETURN_IF_FAILED(cBaseQtFilter::Shutdown(eStage, __exception_ptr));
    RETURN_NOERROR;
}

tResult cDriverFilter::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

   RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);



    if (nEventCode == IPinEventSink::PE_MediaSampleReceived  )
    {      
	
        if (pSource == &m_JuryStructInputPin && m_pCoderDescJuryStruct != NULL) 
        {
            tInt8 i8ActionID = -2;
            tInt16 i16entry = -1;
            
            {   // focus for sample read lock
                __adtf_sample_read_lock_mediadescription(m_pCoderDescJuryStruct,pMediaSample,pCoder);

                pCoder->Get("i8ActionID", (tVoid*)&i8ActionID);
                pCoder->Get("i16ManeuverEntry", (tVoid*)&i16entry);              
            }

            switch (juryActions(i8ActionID))
            {
            case action_GETREADY:
                if(m_bDebugModeEnabled)  LOG_INFO(cString::Format("Driver Module: Received Request Ready with maneuver ID %d",i16entry));
                emit sendRequestReady((int)i16entry);
                break;
            case action_START:
                if(m_bDebugModeEnabled)  LOG_INFO(cString::Format("Driver Module: Received Run with maneuver ID %d",i16entry));
                emit sendRun((int)i16entry);
                break;
            case action_STOP:
                if(m_bDebugModeEnabled)  LOG_INFO(cString::Format("Driver Module: Received Stop with maneuver ID %d",i16entry));
                emit sendStop((int)i16entry);
                break;
            }

        }
    }
    RETURN_NOERROR;

}
tResult cDriverFilter::OnSendState(stateCar stateID, tInt16 i16ManeuverEntry)
{            
        cObjectPtr<IMediaSample> pMediaSample;
        RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

        cObjectPtr<IMediaSerializer> pSerializer;
        m_pCoderDescDriverStruct->GetMediaSampleSerializer(&pSerializer);
        tInt nSize = pSerializer->GetDeserializedSize();

        tInt8 value = tInt8(stateID);

        RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
        {   // focus for sample write lock
            __adtf_sample_write_lock_mediadescription(m_pCoderDescDriverStruct,pMediaSample,pCoder);


            pCoder->Set("i8StateID", (tVoid*)&value);
            pCoder->Set("i16ManeuverEntry", (tVoid*)&i16ManeuverEntry);
        }      
        
        pMediaSample->SetTime(_clock->GetStreamTime());
        m_DriverStructOutputPin.Transmit(pMediaSample);
        if(m_bDebugModeEnabled) 
            {
                switch (stateID)
                {
                case stateCar_READY:
                    LOG_INFO(cString::Format("Driver Module: Send state: READY, Maneuver ID %d",i16ManeuverEntry));
                    break;
                case stateCar_RUNNING:
                    LOG_INFO(cString::Format("Driver Module: Send state: RUNNING, Maneuver ID %d",i16ManeuverEntry));
                    break;
                case stateCar_COMPLETE:
                    LOG_INFO(cString::Format("Driver Module: Send state: COMPLETE, Maneuver ID %d",i16ManeuverEntry));
                    break;
                case stateCar_ERROR:
                    LOG_INFO(cString::Format("Driver Module: Send state: ERROR, Maneuver ID %d",i16ManeuverEntry));
                    break;
                case stateCar_STARTUP:
                    break;
                }
            }
        RETURN_NOERROR;
}

tResult cDriverFilter::loadManeuverList()
{

    m_maneuverListFile = GetPropertyStr("ManeuverFile");
    
    if (m_maneuverListFile.IsEmpty())
    {
        LOG_ERROR("DriverFilter: Maneuver file not found");
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
            LOG_INFO("DriverFilter: Loaded Maneuver file successfully.");
        }
        else
        {
            LOG_ERROR("DriverFilter: no valid Maneuver Data found!");
            RETURN_ERROR(ERR_INVALID_FILE);
        }
    }
    else
    {
        LOG_ERROR("DriverFilter: no valid Maneuver File found!");
        RETURN_ERROR(ERR_INVALID_FILE);
    }
       
    RETURN_NOERROR;
}




