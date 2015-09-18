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


#ifndef _ARDUINO_COMM_FILTER_H_
#define _ARDUINO_COMM_FILTER_H_




#define OID_ADTF_ARDUINOCOM_FILTER "adtf.aadc.arduinoCOM"
/*!
*  The filter runs the communication with the arduino via the serial interface which uses the defined communication protocol. I sends the actuator values to the arduino and receives all the sensor values
*/
class cArduinoCOM : public adtf::cFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_ARDUINOCOM_FILTER, "AADC Arduino Communication", OBJCAT_BridgeDevice, "Arduino Communication", 3, 0, 0, "");    
 
    public:
        /*! constructor */
        cArduinoCOM(const tChar* __info);
        virtual ~cArduinoCOM();

    //Receiver Thread
    private:
        class cReceiveThread: public cKernelThread
            {
                protected:
                cArduinoCOM*    m_pParent; /**< the parent of the thread*/

                public:
                /*! the receiving thread */
                cReceiveThread();
                /*! function to set the parent of the thread
                @param pParent parent for thread
                */
                tResult SetParent(cArduinoCOM* pParent);

                protected: //overwrite ThreadFunc of cKernelThread
                /*! the thread function */ 
                tResult ThreadFunc();
            };

        cReceiveThread m_oReceiveThread;

     protected: // overwrites cFilter
        tResult Init(tInitStage eStage, __exception = NULL);
        tResult Start(__exception = NULL);
        tResult Stop(__exception = NULL);
        tResult Shutdown(tInitStage eStage, __exception = NULL);
        tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
 
    private:
        cOutputPin        m_oCOMOutputPin;            /**< Output Pin for sensor data*/ 
        cInputPin        m_oCOMInputPin;                /**< input Pin for actuator commands */

            cObjectPtr<IMediaTypeDescription> m_pCoderDescArduinoOutputData;        /*!< descriptor for output data*/
            cObjectPtr<IMediaTypeDescription> m_pCoderDescArduinoInputData;            /*!< descriptor for input data*/
        
        /*! creates the input pins
        @param __exception pointer to exception handler
        */
        tResult CreateInputPins(__exception = NULL);
        /*! creates the output pins
        @param __exception pointer to exception handler
        */        
        tResult CreateOutputPins(__exception = NULL);
        
        /*! receives the frames on the serial interface */
        tResult ReceiveFrameFromSerial();

        /*! sends the media samples with the data from the frames on the serial interface */
        tResult SendMediaSample();
        
        /*! receives the media sample on the input pin
        @param pMediaSample the incoming media sample
        */
        tResult ReceiveMediaSample(IMediaSample* pMediaSample);

        /*! sends the frame with the data received on the input pins to the serial interface
        @param frame the pointer to the frame
        @param bytesToWrite the number of bytes to write
        */
        tResult WriteFrameToSerial(tUInt8* frame, int bytesToWrite);
        
        /*! gets the current stream time*/
        tTimeStamp GetStreamTime();
        
        Serial::cRawSerialDevice m_oSerialDevice;    /**< the serial device */

        tArduinoFrame m_oReceivedFrame;            /**< the frame received from the arduino */

        tTimeStamp m_timeLastDataReceived;         /**< the timestamp of the last received frame from the arduino */

        tInt m_timeOutSeconds;                /**< the ammount of seconds to check whether data is received from arduino */
};

//*************************************************************************************************

#endif // _ARDUINO_COMM_FILTER_H_

