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

#ifndef DISPWIDGET
#define DISPWIDGET


#include "stdafx.h"
#include "car.h"

class CarControlFilter;

namespace Ui {
class DisplayGUI;
}

class DisplayWidget : public QWidget
{
    Q_OBJECT

    public:
        DisplayWidget(QWidget* parent, CarControlFilter *filter_orig);
        ~DisplayWidget() {};
    public slots:
        void slotUpdateUS(const int& nPosition, const float& nDistance, const bool& bObjectDetected);
        void slotUpdateSteeringAngle(const float& steer_angle);
        void slotUpdateLuminosityValue(const float& luminosity_value);
        void slotUpdateACCIndividual(const int& sensor_def, const float& value);
        void slotUpdateGYROIndividual(const int& sensor_def, const float& value);
        void slotUpdateRPMIndividual(const int& sensor_def, const float& value);
        void slotUpdateDistanceIndividual(const int& sensor_def, const float& value);
        void slotUpdateVoltageIndividual(const int& sensor_def, const float& value);
        void slotUpdateStatus(int stat);
        void slotUpdateIRIndividual(const int& sensor_def, const double& value);

    private slots:
        void slotSteering(int val);
        void slotSpeed(int val);
        void slotBrake();
        void slotHeadlight();
        void slotBrakelight();
        void slotReverselight();
        void slotTurnSignal();        

    private:
        Ui::DisplayGUI *ui;
        QWidget* m_pWidget;
        CarControlFilter* filter;
};

#endif
