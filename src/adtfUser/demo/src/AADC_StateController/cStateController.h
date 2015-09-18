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


#ifndef _STATE_CONTROLLER_HEADER
#define _STATE_CONTROLLER_HEADER

#define __guid "adtf.aadc.stateController"

#include "stdafx.h"

#include "juryEnums.h"

//! class of filter for the communication with the jury
    /*!
    Der Inhalt bzw. die StateMachine dieses Filters sollte von jedem Team in den eigenen Code übernommen werden.
    */
class cStateController: public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(__guid, "AADC State Controller", OBJCAT_Tool, "State Controller", 1, 0, 0, "Beta Version");    


public: // construction
    cStateController(const tChar *);
    virtual ~cStateController();

    /*! overrides cFilter */
    virtual tResult Init(tInitStage eStage, __exception = NULL);
    
    /*! overrides cFilter */
    virtual tResult Start(__exception = NULL);
    
    /*! overrides cFilter */
    virtual tResult Stop(__exception = NULL);
    
    /*! overrides cFilter */
    virtual tResult Shutdown(tInitStage eStage, __exception = NULL);
    
    /*! overrides cFilter */
    tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);

    /*! overrides cFilter */
    tResult Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr=NULL);

    /*! signal for sending the state
    @param i8StateID state to be sent; -1: error, 0: Ready, 1: Running
    @param i16ManeuverEntry current entry to be sent
    */
    tResult SendState(stateCar state, tInt16 i16ManeuverEntry);
    
    /*! creates the timer for the cyclic transmits*/
    tResult createTimer();

   /*! destroys the timer for the cyclic transmits*/
    tResult destroyTimer(__exception = NULL);

    /*! increments the id of the maneuver id by one and updates the list indexes*/
    tResult incrementManeuverID();

    /*! resets the counters to the start of the current section*/
    tResult resetSection();

   /*! changes the state of the car
    @param newState the new state of the car
    */
    tResult changeState(stateCar newState);
        
    /*! set the maneuver id and find the correct indexes
    @param maneuverId the id of the maneuver which has to be set*/
    tResult setManeuverID(tInt maneuverId);

    /*! this functions loads the maneuver list given in the properties*/
    tResult loadManeuverList();

    /*! Coder Descriptor for input jury struct */
    cObjectPtr<IMediaTypeDescription> m_pCoderDescJuryStruct;

    /*! Coder Descriptor for output driver struct*/
    cObjectPtr<IMediaTypeDescription> m_pCoderDescDriverStruct;

   /*! Coder Descriptor for input set state error */
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSetStateError;
    
    /*! Coder Descriptor for input set state running*/
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSetStateRunning;
    
    /*! Coder Descriptor for input set state stop*/
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSetStateComplete;
    
    /*! Coder Descriptor for input set state ready*/
    cObjectPtr<IMediaTypeDescription> m_pCoderDescSetStateReady;

    /*! Coder Descriptor for input restart section*/
    cObjectPtr<IMediaTypeDescription> m_pCoderDescRestartSection;

    /*! Coder Descriptor for input restart section*/
    cObjectPtr<IMediaTypeDescription> m_pCoderDescIncrementManeuver;

    /*! input pin for the run command */
    cInputPin        m_JuryStructInputPin;
    
    /*! input pin for the set state error command */
    cInputPin        m_StateErrorInputPin;
    
    /*! input pin for the set state running command */
    cInputPin        m_StateRunningInputPin;
    
    /*! input pin for the set state stop command */
    cInputPin        m_StateCompleteInputPin;
    
    /*! input pin for the set state ready command */
    cInputPin        m_StateReadyInputPin;

    /*! input pin for the set state ready command */
    cInputPin        m_StateRestartSectionInputPin;

    /*! input pin for the set state ready command */
    cInputPin        m_StateIncrementManeuverInputPin;

    /*! output pin for state from driver */
    cOutputPin        m_DriverStructOutputPin;
    
    /*! whether output to console is enabled or not*/
    tBool m_bDebugModeEnabled;

    /*! this is the filename of the maneuver list*/
    cFilename m_maneuverListFile;

    /*! this is the list with all the loaded sections from the maneuver list*/
    std::vector<tSector> m_sectorList;

    /*! holds the current state of the car */
    stateCar m_state;

    /*! holds the current maneuver id of the car*/
    tInt16 m_i16CurrentManeuverID;

    /*! holds the current index of the maneuvers in the list in the section */
    tInt16 m_i16ManeuverListIndex;

    /*! holds the current index in the lists of sections */
    tInt16 m_i16SectionListIndex;

    /*! handle for the timer */
    tHandle m_hTimer;

    /*! the critical section of the transmit */
    cCriticalSection m_oCriticalSectionTransmit;

    /*! the critical section of the timer setup */
    cCriticalSection m_oCriticalSectionTimerSetup;

};

#endif
