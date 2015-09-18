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

/*! \brief Calibration Scaling Extended
 *         
 *  Dieser Filter dient zur Kalibrierung und Skalierung von Daten. Im Gegensatz zum Calibration Scaling Filter wird hier eine abschnittsweise definierte Funktion f(x) mit Stützstellen zur Interpolation der Messdaten verwendet.
 */

#ifndef _SENSORCALIBRATION_H_
#define _SENSORCALIBRATION_H_

#include "stdafx.h"

#define OID_ADTF_SENSOR_CALIBRATION "adtf.aadc.sensorCalibration"

/*!
This filter can be used to scale data with a loaded xml calibration table
*/
class SensorCalibration : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_SENSOR_CALIBRATION, "AADC Calibration Filter Extended", OBJCAT_DataFilter, "Sensor Calibration Filter Extended", 1, 0, 1, "Beta Version");    

        cInputPin m_oInput;                //input pin for the raw value
        cOutputPin m_oOutput;            //output pin for the calibrated value

    public:
        SensorCalibration(const tChar* __info);
        virtual ~SensorCalibration();
    
    protected: // overwrites cFilter
        tResult Init(tInitStage eStage, __exception = NULL);
        tResult Start(__exception = NULL);
        tResult Stop(__exception = NULL);
        tResult Shutdown(tInitStage eStage, __exception = NULL);
        
        tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
    
    private:
        /*! creates all the input Pins*/
        tResult CreateInputPins(__exception = NULL);
        /*! creates all the output Pins*/
        tResult CreateOutputPins(__exception = NULL);
        /*! reads the xml file which is set in the filter properties */
        tResult LoadConfigurationData();
        /*! checks the loaded configuration data; checks if the xvalues are in increasing order*/
        tResult CheckConfigurationData();

        /*! doing the linear interpolation
        @param inputValue the value which should be interpolated
        */
        tFloat32 getLinearInterpolatedValue(tFloat32 inputValue);

        /*! doing the cubic spline interpolation
        @param inputValue the value which should be interpolated
        */
        tFloat32 getCubicSplineInterpolatedValue(tFloat32 inputValue);
        
        /*! holds the xml file for the supporting points*/
        cFilename m_fileConfig;
        /*! holds the yValues for the supporting points*/
        vector<tFloat32> m_yValues;
        /*! holds the xValues for the supporting points*/
        vector<tFloat32> m_xValues;
        /*! the class for cubic interpolation*/
        Cubic *m_cubicInterpolation;
        
        /*! if debug console output is enabled */
        tBool m_bDebugModeEnabled;
        
        /*! indicates which calibration mode is used: 1: linear, 2: cubic, 3: none*/
        tInt m_calibrationMode;
        
        /*! Coder Descriptor for the pins*/
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;    
    
};



//*************************************************************************************************

#endif // _SENSORCALIBRATION_H_

