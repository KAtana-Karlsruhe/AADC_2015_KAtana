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

/*! \brief Unbundle
 *         
 *  We create a single ADTF plugin that provides several filter classes.
 These macros create the plugin entry and exit points and they define the plugin
 class factory which makes the contained classes available to the ADTF runtime.
 */
 
#include "stdafx.h"
#include "AbstractSensorBundle.h"
#include "GyroSensorBundle.h"
#include "USSSensorBundle.h"
#include "IRSensorBundle.h"
#include "SteerangleSensorBundle.h"
#include "AccelSensorBundle.h"
#include "WheelcounterSensorBundle.h"
#include "VoltageSensorBundle.h"

ADTF_PLUGIN_BEGIN(cGenericBundle, "AADC Generic Bundle", 0x00)
    ADTF_MAP_FILTER(OID_ADTF_GYROBUNDLE_FILTER, cGyroSensorBundle)
    ADTF_MAP_FILTER(OID_ADTF_USSBUNDLE_FILTER, cUSSSensorBundle)
    ADTF_MAP_FILTER(OID_ADTF_IRBUNDLE_FILTER, cIRSensorBundle)    
    ADTF_MAP_FILTER(OID_ADTF_STEERBUNDLE_FILTER, cSteerangleSensorBundle)    
    ADTF_MAP_FILTER(OID_ADTF_ACCELBUNDLE_FILTER, cAccelSensorBundle)
    ADTF_MAP_FILTER(OID_ADTF_WHEELCOUNTERBUNDLE_FILTER, cWheelcounterSensorBundle)
    ADTF_MAP_FILTER(OID_ADTF_VOLTAGEBUNDLE_FILTER, cVoltageSensorBundle)
ADTF_PLUGIN_END(cGenericBundle)

