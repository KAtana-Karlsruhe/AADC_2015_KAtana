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
#ifndef ROADSIGNEVALUATOR_H
#define ROADSIGNEVALUATOR_H

#include "stdafx.h"
#include "displaywidget.h"
#define __guid "adtf.aadc.road_sign_evaluator"

class RoadSignEvaluator : public QObject, public cBaseQtFilter
{
    ADTF_DECLARE_FILTER_VERSION(__guid, "AADC Road Sign Evaluator", adtf::OBJCAT_Tool, "AADC RoadSignEvaluator", 1, 0, 0, "");    

    Q_OBJECT
    
public:
    RoadSignEvaluator(const tChar* __info);
    virtual ~RoadSignEvaluator();
    
    virtual tResult Init(tInitStage eStage, __exception = NULL);
    virtual tResult Start(__exception = NULL);
    virtual tResult Stop(__exception = NULL);    
    virtual tResult Shutdown(tInitStage eStage, __exception = NULL);

    virtual tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
    
private:
    /*! the input pin for the struct from the detector */
    cInputPin m_InputPin;
    /*! the descriptor coder for the input pin*/
    cObjectPtr<IMediaTypeDescription> m_pCoderDescInput;
    /*! the gui of this filter */
    DisplayWidget* m_pWidget;
    /*! this function creates the input pins */
    tResult CreateInputPins();
        
    // reimplemented from cBaseQtFilter
    tHandle CreateView();
    tResult ReleaseView();

    /*! indicates wheter information is printed to the console or not */
    tBool m_bDebugModeEnabled;
};

#endif//ROADSIGNEVALUATOR_H
