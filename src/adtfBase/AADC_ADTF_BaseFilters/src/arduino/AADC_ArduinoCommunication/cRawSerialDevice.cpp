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

#include "stdafx.h"
#include "cRawSerialDevice.h"


#ifndef WIN32
Serial::cRawSerialDevice::cRawSerialDevice()
{
    memset (&m_old_options, 0, sizeof (m_old_options));
    memset (&m_options, 0, sizeof (m_options));
}

Serial::cRawSerialDevice::~cRawSerialDevice()
{
    tcsetattr(m_FileDescriptorSerial, TCSANOW, &m_old_options);
}

tResult Serial::cRawSerialDevice::Open(const tChar* strDeviceName, tInt nBaudRate, tInt nParity, tInt nDataBits, tInt nStopBits, tInt nRxQueue, tInt nTxQueue, tTimeStamp nReadTimeout)
{
    // "/dev/ttyUSB0"
    //m_FileDescriptorSerial = open( path, O_RDWR | O_NOCTTY );

    if ( (m_FileDescriptorSerial = open( strDeviceName, O_RDWR | O_NOCTTY )) < 0 )
    {
        //std::cerr << "Could not open Serial Device specified..." << std::endl;
        return m_FileDescriptorSerial;        
        //exit(-1);
    }

    tcgetattr ( m_FileDescriptorSerial, &m_old_options ) ;
    

    bzero(&m_options, sizeof(m_options));
    m_options.c_cflag = B115200 | CRTSCTS | CS8 | CLOCAL | CREAD;
    m_options.c_iflag = IGNPAR;
    m_options.c_oflag = OPOST;

    m_options.c_lflag = 0;

    m_options.c_cc[VTIME] = 0 ;
    m_options.c_cc[VMIN] = 7; // blocking until 7 chars are received i.e. length of header of arduino frame

    tcflush( m_FileDescriptorSerial, TCIFLUSH );
    tcsetattr(m_FileDescriptorSerial, TCSANOW, &m_options);
    
    //std::cout << "Open Serial Port with FileDesc: " << m_FileDescriptorSerial <<std::endl;

    return m_FileDescriptorSerial;
}

tResult Serial::cRawSerialDevice::Close()
{
    //std::cout << "Closing Serial Port with FileDesc: " << m_FileDescriptorSerial <<std::endl;
    RETURN_IF_FAILED(close(m_FileDescriptorSerial));
    RETURN_NOERROR;
}

tInt32 Serial::cRawSerialDevice::Write(unsigned char* frame, int bytesToWrite)
{
    //std::cout << "Writing to Serial Port with FileDesc: " << m_FileDescriptorSerial << "\tBytes: " << bytesToWrite << std::endl;
    return write( m_FileDescriptorSerial, frame, bytesToWrite);
}

tInt32 Serial::cRawSerialDevice::Read(unsigned char* frame, int bytesToRead)
{
    //std::cout << "Reading from Serial Port with FileDesc: " << m_FileDescriptorSerial << "\tBytes: " << bytesToRead << std::endl;
    return read(m_FileDescriptorSerial, frame , bytesToRead);
}
#endif

tInt32 Serial::cRawSerialDevice::BytesAvailable()
{    
    int bytesAvailable = 0;
#ifndef     WIN32
    ioctl(m_FileDescriptorSerial, FIONREAD, &bytesAvailable);
#else
    DWORD status;
    COMSTAT comStat;
    ClearCommError((HWND)(m_hDevice),&status, &comStat);
    bytesAvailable = (tInt32)comStat.cbInQue;
#endif
    //std::cout << "Bytes available on FileDesc: " << FileDescriptorSerial << "\t Bytes: " << bytesAvailable << std::endl;
    return bytesAvailable;
}



