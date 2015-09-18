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

#include "stdafx.h"
#include "displaywidget.h"
#include "CarVisualizationFilter.h"

using namespace SensorDefinition;

DisplayWidget::DisplayWidget(QWidget* parent, CarControlFilter* filter_orig) :
QWidget(),
ui(new Ui::DisplayGUI),
filter(filter_orig)
{
    m_pWidget = parent;
    ui->setupUi(this);

    connect(ui->SteeringSlider, SIGNAL(valueChanged(int)), this, SLOT(slotSteering(int)));
    //Gearbox
    connect(ui->SpeedSlider, SIGNAL(valueChanged(int)), this, SLOT(slotSpeed(int)));
    connect(ui->pb_brake, SIGNAL(clicked(bool)), this, SLOT(slotBrake()));
    // Headlight signal
    connect(ui->HeadLightSwitch, SIGNAL(clicked()), this, SLOT(slotHeadlight()));
    // Brakelight signal
    connect(ui->rb_bl_on, SIGNAL(clicked(bool)), this, SLOT(slotBrakelight()));
    connect(ui->rb_bl_off, SIGNAL(clicked(bool)), this, SLOT(slotBrakelight()));
//    connect(ui->rb_bl_emergency, SIGNAL(clicked(bool)), this, SLOT(slotBrakelight()));
    // Reverselight signal
    connect(ui->RevLightSwitch, SIGNAL(clicked()), this, SLOT(slotReverselight()));
    // Turn signal
    connect(ui->rb_ts_l_on, SIGNAL(clicked(bool)), this, SLOT(slotTurnSignal()));
//    connect(ui->rb_ts_l_3, SIGNAL(clicked(bool)), this, SLOT(slotTurnSignal()));
    connect(ui->rb_ts_off, SIGNAL(clicked(bool)), this, SLOT(slotTurnSignal()));
//    connect(ui->rb_ts_r_3, SIGNAL(clicked(bool)), this, SLOT(slotTurnSignal()));
    connect(ui->rb_ts_r_on, SIGNAL(clicked(bool)), this, SLOT(slotTurnSignal()));
}

void DisplayWidget::slotSteering(int val)
{
    filter->slotSteering(val);
}

void DisplayWidget::slotSpeed(int val)
{
    ui->rb_backward->setEnabled(val == 0);
    ui->rb_forward->setEnabled(val == 0);
    if (ui->rb_forward->isChecked())
        filter->slotSpeed(val);
    else if (ui->rb_backward->isChecked())
        filter->slotSpeed(val*(-1));  //multiply with -1 if backward is enabled
}


void DisplayWidget::slotBrake()
{
    ui->rb_backward->setEnabled(true);
    ui->rb_forward->setEnabled(true);
    ui->SpeedSlider->setValue(0);
}

void DisplayWidget::slotHeadlight()
{
    QPalette pal;
    tHeadLight sHeadLight;
    if (ui->HeadLightSwitch->isChecked())
    {
        pal.setColor(QPalette::Button, QColor(0, 255, 0));
        sHeadLight.nState = 2;
    }
    else
    {
        pal.setColor(QPalette::Button, QColor(255, 0, 0));
        sHeadLight.nState = 0;
    }
    filter->slotHeadlight(sHeadLight);
}

void DisplayWidget::slotBrakelight()
{
    tBrakeLight sBrakeLight;
    if (this->sender() == ui->rb_bl_off)
    {
        sBrakeLight.nState = 0;
    }
    else if (this->sender() == ui->rb_bl_on)
    {
        sBrakeLight.nState = 1;
    }
    //else if (this->sender() == ui->rb_bl_emergency)
    //{
    //    sBrakeLight.nState = 2;
    //}
    filter->slotBrakelight(sBrakeLight);
}

void DisplayWidget::slotReverselight()
{
    tReverseLight sReverseLight;
    if (ui->RevLightSwitch->isChecked())
        sReverseLight.nState = 1;
    else
        sReverseLight.nState = 0;
    filter->slotReverselight(sReverseLight);
}

void DisplayWidget::slotTurnSignal()
{
    tTurnSignal sTurnSignal;
    if (this->sender() == ui->rb_ts_l_on)
    {
        sTurnSignal.nState = 1;
    }
    /*else if (this->sender() == ui->rb_ts_l_3)
    {
        sTurnSignal.nState = 2;
    }*/
    else if (this->sender() == ui->rb_ts_off)
    {
        sTurnSignal.nState = 0;
    }
    /*else if (this->sender() == ui->rb_ts_r_3)
    {
        sTurnSignal.nState = 4;
    }*/
    else if (this->sender() == ui->rb_ts_r_on)
    {
        sTurnSignal.nState = 3;
    }
    /*else if (this->sender() == ui->rb_ts_hazard)
    {
        sTurnSignal.nState = 5;
    }*/
    filter->slotTurnSignal(sTurnSignal);
}



void DisplayWidget::slotUpdateUS(const int& nPosition, const float& nDistance, const bool& bObjectDetected)
{
    QString dist = QString("%1 cm").arg(nDistance);
    if (bObjectDetected)
    {
        switch(nPosition)
        {
            case 1:
                ui->US_front_left->setText(dist);
                break;
            case 2:
                ui->US_front_right->setText(dist);
                break;
            case 3:
                ui->US_back_right->setText(dist);
                break;
            case 4:
                ui->US_back_left->setText(dist);
                break;
        }
    }
    else
    {
        switch(nPosition)
        {
            case 1:
                ui->US_front_left->setText("---");
                break;
            case 2:
                ui->US_front_right->setText("---");
                break;
            case 3:
                ui->US_back_right->setText("---");
                break;
            case 4:
                ui->US_back_left->setText("---");
                break;
        }
    }
//    update();
}

void DisplayWidget::slotUpdateSteeringAngle(const float& steer_angle)
{
    ui->lbl_servo_pos->setText(QString::number(steer_angle,'f',2));;
}

void DisplayWidget::slotUpdateLuminosityValue(const float& luminosity_value)
{
    //ui->lumi_value->setNum(luminosity_value);
    //ui->lumi_value->setText(QString("%1 lux").arg(luminosity_value));
}

void DisplayWidget::slotUpdateACCIndividual(const int& sensor_def, const float& value)
{
    switch((ACCSensorDefinition)sensor_def)
    {
    case ACC_X:
        {
            float fAcc = floorf(value*1000 + .5)/1000;
            ui->lbl_acc_x->setText(QString::number(fAcc,'f',2));
            break;
        }
    case ACC_Y:
        {
            float fAcc = floorf(value*1000 + .5)/1000;
            ui->lbl_acc_y->setText(QString::number(fAcc,'f',2));
            break;
        }
    case ACC_Z:
        {
            float fAcc = floorf(value*1000 + .5)/1000;
            ui->lbl_acc_z->setText(QString::number(fAcc,'f',2));
            break;
        }
    default:
        {
            break;
        }
    };
//    update();
}

void DisplayWidget::slotUpdateGYROIndividual(const int& sensor_def, const float& value)
{
    switch((GYROSensorDefinition)sensor_def)
    {
    case GYRO_YAW:
        {
            ui->lbl_gyro_x->setText(QString::number(value,'f',2));
            break;
        }
    case GYRO_PITCH:
        {
            ui->lbl_gyro_y->setText(QString::number(value,'f',2));
            break;
        }
    case GYRO_ROLL:
        {
            ui->lbl_gyro_z->setText(QString::number(value,'f',2));
            break;
        }
    case GYRO_TEMP:
        {
            //ui->lbl_gyro_temp->setText(QString::number(value,'f',2));
            break;
        }
    default:
        {
            break;
        }
    };

}


void DisplayWidget::slotUpdateRPMIndividual(const int& sensor_def, const float& value)
{
    switch((WheelSensorDefinition)sensor_def)
    {
    case WHEEL_LEFT:
        {
            ui->RadEncRPM_links->setText(QString("%1 rpm").arg(int(value)));
            break;
        }
    case WHEEL_RIGHT:
        {
            ui->RadEncRPM_rechts->setText(QString("%1 rpm").arg(int(value)));
            break;
        }
    default:
        {
            break;
        }
    };
}

void DisplayWidget::slotUpdateDistanceIndividual(const int& sensor_def, const float& value)
{
    switch((WheelSensorDefinition)sensor_def)
    {
    case DISTANCE_LEFT:
        {
            ui->RadEnc_Dist_links->setText(QString("%1 cm").arg(int(value)));
            break;
        }
    case DISTANCE_RIGHT:
        {
            ui->RadEnc_Dist_rechts->setText(QString("%1 cm").arg(int(value)));
            break;
        }
    default:
        {
            break;
        }
    };
}

void DisplayWidget::slotUpdateVoltageIndividual(const int& sensor_def, const float& value)
{
    switch((VoltageSensorDefinition)sensor_def)
    {
    case VOLTAGE_MEASUREMENT:
        {
            ui->lblVolt1->setNum(floorf(value*100.f + .5f)/100.f);
            break;
        }
    case VOLTAGE_ENGINE:
        {
            ui->lblVolt2->setNum(floorf(value*100.f + .5f)/100.f);
            break;
        }
    default:
        {
            break;
        }
    };
}

void DisplayWidget::slotUpdateStatus(int stat)
{
    if (stat == 0)
        ui->label_AnfahrW->setPixmap(QPixmap(":/images/AnfahrNoWarn.png"));
    else if (stat == 1)
        ui->label_AnfahrW->setPixmap(QPixmap(":/images/AnfahrWarn.png"));
    else if (stat == 2)
        ui->label_Notbremse->setPixmap(QPixmap(":/images/NotbrAn.png"));
    else if (stat == 3)
        ui->label_Notbremse->setPixmap(QPixmap(":/images/NotbrAus.png"));
}

void DisplayWidget::slotUpdateIRIndividual(const int& sensor_def, const double& value)
{
    switch ((IrSensorDefinition)sensor_def)
    {
        case IR_FRONT_CENTER_LONG:
        {
            ui->IRvL->setText(QString("%1 cm").arg(QString::number(value,'f',2)));
            break;
        }
        case IR_FRONT_CENTER_SHORT:
        {
            ui->IRvS->setText(QString("%1 cm").arg(QString::number(value,'f',2)));
            break;
        }
        case IR_FRONT_LEFT_LONG:
        {
            ui->IRfrontleftL->setText(QString("%1 cm").arg(QString::number(value,'f',2)));
            break;
        }
        case IR_FRONT_LEFT_SHORT:
        {
            ui->IRfrontleftS->setText(QString("%1 cm").arg(QString::number(value,'f',2)));
            break;
        }
        case IR_FRONT_RIGHT_LONG:
        {
            ui->IRfrontrightL->setText(QString("%1 cm").arg(QString::number(value,'f',2)));
            break;
        }
        case IR_FRONT_RIGHT_SHORT:
        {
            ui->IRfrontrightS->setText(QString("%1 cm").arg(QString::number(value,'f',2)));
            break;
        }
        case IR_REAR_CENTER_SHORT:
        {
            ui->IRbL->setText(QString("%1 cm").arg(QString::number(value,'f',2)));
            break;
        }
        case IR_REAR_LEFT_SHORT:
        {
            ui->IRbackleft->setText(QString("%1 cm").arg(QString::number(value,'f',2)));
            break;
        }
        case IR_REAR_RIGHT_SHORT:
        {
            ui->IRbackright->setText(QString("%1 cm").arg(QString::number(value,'f',2)));
            break;
        }        
    };
}
