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

/*! \brief Car Visualization
 *         
 *  Dieser Filter dient zum Remote Steuern des Fahrzeugs mittels GUI auf einem Notebook. Achtung: wurde bisher nur unter Windows 7 getestet.
 */

#ifndef _JURY_TRANSMITTER_FILTER_HEADER
#define _JURY_TRANSMITTER_FILTER_HEADER

#define __guid "adtf.aadc.jury_transmitter"

#include "stdafx.h"

#include "juryEnums.h"

#include "displaywidget.h"

//! class of filter for the jury
    /*!
    with this filter the state of a vehicle can be supervised an the action of it controlled
    */
class cJuryTransmitter : public QObject, public cBaseQtFilter
{
    ADTF_DECLARE_FILTER_VERSION(__guid, "AADC Jury Module", OBJCAT_Tool, "Jury Transmitter", 1, 2, 0,"");    

    Q_OBJECT
    
signals:


public: // construction
    cJuryTransmitter(const tChar *);
    virtual ~cJuryTransmitter();

    // overrides cFilter
    virtual tResult Init(tInitStage eStage, __exception = NULL);
    virtual tResult Start(__exception = NULL);
    virtual tResult Stop(__exception = NULL);
    virtual tResult Shutdown(tInitStage eStage, __exception = NULL);

    /*! */
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

    /*! */
    tResult Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr=NULL);
private:
    /*! this functions transmits the jury struct with the given data over the output pin
    @param actionID the actionID of the struct; -1: stop, 0: getReady, 1: Run
    @param maneuverEntry number of the maneuver of which the action is requested
    */
    tResult sendJuryStruct(juryActions actionID, tInt16 maneuverEntry);
protected: // Implement cBaseQtFilter

    /*! Creates the widget instance*/
    tHandle CreateView();

    /*! Destroys the widget instance*/
    tResult ReleaseView();
    
public slots:
    /*! send the sample for the Not Aus*/
    tResult OnNotAus();
    
    /*! send the sample for stop in section
    @param entryId the current id of the maneuver
    */
    tResult OnStop(tInt16 entryId);
    
    /*! send the sample for start section
    @param entryId the current id of the maneuver
    */
    tResult OnStart(tInt16 entryId);
    
    /*! send the sample for request ready 
    @param entryId the current id of the maneuver
    */
    tResult OnRequestReady(tInt16 entryId);
    
signals:
    /*! send new state to widget 
    @param state the state of the input
    @param entryId the id of the current maneuver
    */
    void sendDriverState(int state, int entryId); 
    
    /*! send new message to text field 
    @param text the text which should be plotted to gui*/
    void sendMessage(QString text);

private:
    /*! this functions loads the maneuver list given in the properties*/
    tResult loadManeuverList();

    /*! send emergency stop*/
    tResult sendEmergencyStop();

protected:

    /*! The displayed widget*/
    DisplayWidget *m_pWidget;
    /*! Coder Descriptor */
    cObjectPtr<IMediaTypeDescription> m_pCoderDescNotAus;
    /*! Coder Descriptor */
    cObjectPtr<IMediaTypeDescription> m_pCoderDescJuryStruct;
    /*! Coder Descriptor */
    cObjectPtr<IMediaTypeDescription> m_pCoderDescDriverStruct;
    /*! output pin for the Not Aus*/
    cOutputPin            m_NotAusOutputPin;
    /*! output pin for the stop command*/
    cOutputPin            m_JuryStructOutputPin;
    /*! input pin for state from driver*/
    cInputPin            m_DriverStructInputPin;
    /*! whether output to console is enabled or not*/
    tBool m_bDebugModeEnabled;    
    /*! last maneuver id*/
    tInt16 m_last_entryId;
    /*! timestamp of last maneuver id */
    tTimeStamp m_last_timestamp;
    /*! this is the filename of the maneuver list*/
    cFilename m_maneuverListFile;
    /*! this is the list with all the loaded sections from the maneuver list*/
    std::vector<tSector> m_sectorList;

    /*! handle for timer for not aus*/
    tHandle m_hTimerNotAus;
    /*! counter for not aus */
    tInt m_hTimerNotAusCounter;
     /*! the critical section of the timer setup */
    cCriticalSection m_oCriticalSectionTimerNotAus;

    /*! timer for stop event*/
    tHandle m_hTimerStop;    
    /*! counter for stop event*/
    tInt m_hTimerStopCounter;
    /*! id for current stop id*/
    tInt m_Stop_entry_id;
     /*! the critical section of the timer setup */
    cCriticalSection m_oCriticalSectionTimerStop;
    
    /*! timer for start event*/
    tHandle m_hTimerStart;    
    /*! counter for start event*/
    tInt m_hTimerStartCounter;
    /*! id for current start event*/
    tInt m_Start_entry_id;
     /*! the critical section of the timer setup */
    cCriticalSection m_oCriticalSectionTimerStart;
    
    /*! timer for request ready event*/
    tHandle m_hTimerRequestReady;
    /*! counter for request ready events */
    tInt m_hTimerRequestReadyCounter;
    /*! id for current request ready*/
    tInt m_Request_Ready_entry_id;
     /*! the critical section of the timer setup */
    cCriticalSection m_oCriticalSectionTimerRequestReady;


    
    
};

#endif
