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
#include "stdafx.h"
#include "cSlidingWindow.h"


cSlidingWindow::cSlidingWindow(tUInt in_m_queue_length):
m_uiQueueLength(in_m_queue_length)
{
}

cSlidingWindow::~cSlidingWindow(){}

tVoid cSlidingWindow::addNewValue(tFloat32 inputValues, tTimeStamp inputTime)
{
    if (m_inputValues.size()<m_uiQueueLength)
        m_inputValues.push(make_pair<tFloat32,tTimeStamp>(inputValues, inputTime));
    else
        {
        m_inputValues.pop();
        m_inputValues.push(make_pair<tFloat32,tTimeStamp>(inputValues, inputTime));
        }
}
tFloat32 cSlidingWindow::getBeginValue()
{
    if (m_inputValues.size()==m_uiQueueLength)
        return m_inputValues.front().first;
    else
        return 0;
}
tFloat32 cSlidingWindow::getEndValue()
{
    if (m_inputValues.size()==m_uiQueueLength)        
        return m_inputValues.back().first;
    else
        return 0;
}
tTimeStamp cSlidingWindow::getBeginTime()
{
    if (m_inputValues.size()==m_uiQueueLength)        
        return m_inputValues.front().second;
    else
        return 0;
}
tTimeStamp cSlidingWindow::getEndTime()
{
    if (m_inputValues.size()==m_uiQueueLength)
        return m_inputValues.back().second;
    else
        return 0;
}
