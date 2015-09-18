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


#ifndef _DRIVER_FILTER_HEADER
#define _DRIVER_FILTER_HEADER

#define __guid "adtf.aadc.driver_filter"

#include "stdafx.h"

#include "juryEnums.h"

#include "displaywidget.h"


//! class of filter for the communication with the jury
    /*!
    This filter is a prototyp for the receiver of the commands from the jury. It also shows which commands has to be sent to the jury.
    */
class cDriverFilter : public QObject, public cBaseQtFilter
{
    ADTF_DECLARE_FILTER_VERSION(__guid, "AADC Driver Module", OBJCAT_Tool, "Driver filter", 1, 1, 0, "Beta Version");    

    Q_OBJECT
    
signals:


public: // construction
    cDriverFilter(const tChar *);
    virtual ~cDriverFilter();

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


protected: // Implement cBaseQtFilter

    /*! Creates the widget instance*/
    tHandle CreateView();

    /*! Destroys the widget instance*/
    tResult ReleaseView();
    
public slots:    
    /*! function which transmits the state      
    @param i8StateID state to be sent; -1: error, 0: Ready, 1: Running
    @param i16ManeuverEntry current entry to be sent
    */
    tResult OnSendState(stateCar stateID, tInt16 i16ManeuverEntry);    

signals:
    /*! signal to the gui to show the command "run" from the jury
    @param entryId current entry to be sent 
    */
    void sendRun(int entryId); 
    
    /*! signal to the gui to show the command "stop" from the jury 
    @param entryId current entry to be sent 
    */
    void sendStop(int entryId);

    /*! signal to the gui to show the command "request ready" from the jury 
    @param entryId current entry to be sent 
    */
    void sendRequestReady(int entryId);
private:

    /*! The displayed widget*/    
    DisplayWidgetDriver *m_pWidget;

    /*! Coder Descriptor */
    cObjectPtr<IMediaTypeDescription> m_pCoderDescJuryStruct;
    /*! Coder Descriptor */
    cObjectPtr<IMediaTypeDescription> m_pCoderDescDriverStruct;

    /*! input pin for the run command*/
    cInputPin        m_JuryStructInputPin;
    /*! output pin for state from driver*/
    cOutputPin        m_DriverStructOutputPin;
    
    /*! whether output to console is enabled or not*/
    tBool m_bDebugModeEnabled;

    /*! this is the filename of the maneuver list*/
    cFilename m_maneuverListFile;
    /*! this functions loads the maneuver list given in the properties*/
    tResult loadManeuverList();
    /*! this is the list with all the loaded sections from the maneuver list*/
    std::vector<tSector> m_sectorList;

};

#endif
