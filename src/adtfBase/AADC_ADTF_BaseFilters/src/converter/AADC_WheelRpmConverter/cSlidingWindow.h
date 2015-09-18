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

#ifndef _SLIDINGWINDOW_H_
#define _SLIDINGWINDOW_H_

/*!this class implements a sliding window filter. a window of the given length is defined and the get functions return the first and last sample of the window*/
class cSlidingWindow
{
public:
    /*! constructor of the class
    @param in_m_queue_length defines the length of the queue i.e. the length of the sliding window
    */
    cSlidingWindow(tUInt in_m_queue_length);
    /*! destructor */
    ~cSlidingWindow();
    /*! adds a new tFloat32 value to the internal queue
    @param inputValues    the value of the new sample
    @param inputTime    the timestamp of the new sample
    */
    tVoid addNewValue(tFloat32 inputValues, tTimeStamp inputTime);
    /*! gets the first sample of the defined sliding window*/
    tFloat32 getBeginValue();
    /*! gets the last sample of the defined sliding window*/
    tFloat32 getEndValue();
    /*! gets the timestamp of the first sample of the defined sliding window*/
    tTimeStamp getBeginTime();
    /*! gets the timestamp of the last sample of the defined sliding window*/
    tTimeStamp getEndTime();
private:
    /*! queue which holds all the samples with their timestamps as a pair*/
    queue< pair<tFloat32,tTimeStamp> > m_inputValues;
    /*! length of the queue, i.e. the length of the sliding window*/
    tUInt m_uiQueueLength;
    
};

#endif
