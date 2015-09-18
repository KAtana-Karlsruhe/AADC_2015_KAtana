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
#include "qcustomplot.h"
#include <vector>
#include <deque>

#define NUM_ROADSIGNS 12
#define RANGE_DATASET 10

namespace Ui {
class DisplayWidget;
}

class DisplayWidget : public QWidget
{
    Q_OBJECT

public:
    /*! Default Constructor */
    DisplayWidget(QWidget* parent);
    virtual ~DisplayWidget();
    /*! this function pushes the data received by filter to the m_identiferBuffer*/
    void pushData(tInt8);

private slots:
    /*!setup all graphs for the road signs*/
    void setupGraph();
    /*! this function handles the realtime graph*/
    void realTimeData();
    /*! this function calculates the data in m_identiferBuffer*/
    void calcData();
    /*! this function sets the roadsign in the gui*/
    void setRoadSign(int sign);

private:
    /*!Pointer to the Gui*/
    Ui::DisplayWidget *ui;
    /*! this Timer is handles the update rate of the graph*/
    QTimer dataTimer;

    /*! graph to display values*/
    QCPGraph* m_graph_vorfahrtgew;
    /*! graph to display values*/
    QCPGraph* m_graph_vorfahrt;
    /*! graph to display values*/
    QCPGraph* m_graph_stop;
    /*! graph to display values*/
    QCPGraph* m_graph_parken;
    /*! graph to display values*/
    QCPGraph* m_graph_geradeaus;    
    /*! graph to display values*/
    QCPGraph* m_graph_kreuzung;    
    /*! graph to display values*/
    QCPGraph* m_graph_fussgaenger;    
    /*! graph to display values*/
    QCPGraph* m_graph_kreisverkehr;    
    /*! graph to display values*/
    QCPGraph* m_graph_ueberholverbot;    
    /*! graph to display values*/
    QCPGraph* m_graph_verboteinfahrt;    
    /*! graph to display values*/
    QCPGraph* m_graph_einbahnstrasse;

    /*! Array which holds the RoadSign Images*/
    QImage* m_roadSigns[NUM_ROADSIGNS];
    /*! Enum for the road signs, just as an identifier*/
    enum
    {
        NOMATCH = 0,
        VORFAHRTGEW,
        VORFAHRT,
        STOP,    
        PARKEN,
        GERADEAUS,
        KREUZUNG,
        FUSSGAENGER,
        KREISVERKEHR,
        UEBERHOLVERBOT,
        VERBOTEINFAHRT,
        EINBAHNSTRASSE,
    };

    /*! Buffer to hold the values received by the filter*/
    std::deque<tInt8> m_identiferBuffer;
};

#endif
