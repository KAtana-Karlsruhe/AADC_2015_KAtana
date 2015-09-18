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
* $Author:: spiesra $  $Date:: 2014-09-16 13:29:48#$ $Rev:: 26104   $
**********************************************************************/


namespace Serial
{
    /* class to read and write on the serial interface; serial interface is used in raw mode
    */
    class cRawSerialDevice : public cSerialDevice
    {
#ifndef WIN32
    public:

        /* constructor for serial device in raw mode */        
        cRawSerialDevice();
        /* destructor */
        ~cRawSerialDevice();

        /* Opens a serial device; reimplementation for device in raw mode
         * @param strDeviceName [in] The device name.
         * @param nBaudRate     [in] The baud rate. (not used, just here for conformity to parent class)
         * @param nParity       [in] The parity. (not used, just here for conformity to parent class)
         * @param nDataBits     [in] The amount of data bits. (not used, just here for conformity to parent class)
         * @param nStopBits     [in] The amount of stop bits. (not used, just here for conformity to parent class)
         * @param nRxQueue      [in] The receive queue size. (not used, just here for conformity to parent class)
         * @param nTxQueue      [in] The transmission queue size. (not used, just here for conformity to parent class)
         * @param nReadTimeout  [in] The timeout for read operations in microseconds. (not used, just here for conformity to parent class)
         * @return Standard result.
         * @remarks only win32: the strDeviceName "COM1" through "COM9" can be used. A general way to specify the COM-ports (inclusive the numbering greather than 9) is the "\\\\.\\COMx". Further information can be found at http://support.microsoft.com/kb/115831/en-us
        */
        tInt Open(const tChar* strDeviceName, tInt nBaudRate=9600, tInt nParity=SER_NOPARITY, tInt nDataBits=8, tInt nStopBits=SER_ONESTOPBIT, tInt nRxQueue=4096, tInt nTxQueue=4096, tTimeStamp nReadTimeout=0);
        /* closes the serial interface */
        tResult Close();
    
        /* writes byte of the frame to the serial interface
        @param frame pointer to the frame to be written
        @param bytesToWrite number of bytes to be written
        */
        tInt32 Write(unsigned char* frame, int bytesToWrite);
        
        /* reads bytes from the serial interface
        @param frame pointer to frame where data should be written
        @param bytesToRead number of bytes to be read
        */        
        tInt32 Read(unsigned char* frame, int bytesToRead);
        
    private:
        /* struct with old options */
        struct termios m_old_options;
        /* struct with current options */        
        struct termios m_options;
        /* file descriptor for serial interface*/
        size_t m_FileDescriptorSerial;
#endif
    public:
        /* checks if bytes are available at the serial interface, returns the number of bytes */
        tInt32 BytesAvailable();
    };

}