/**********************************************************************
* Copyright (c) 2013 BFFT Gesellschaft fuer Fahrzeugtechnik mbH.
* All rights reserved.
**********************************************************************
* $Author:: spiesra $  $Date:: 2014-09-16 13:29:48#$ $Rev:: 26104   $
**********************************************************************/

/*! \brief Watchdog Trigger
 *         
 *  Dieser Filter sendet periodisch ein Signal an den Watchdog im Arduino. Kommt diese Signal im Arduino nicht an, wird das Relais für die Antriebselektronik ausgeschaltet.
 */

#ifndef _AKTORS_FILTER_H_
#define _AKTORS_FILTER_H_

#define OID_ADTF_WATCHDOGGUARD_FILTER "adtf.aadc.watchdogGuard"
//! class of filter for arduino watchdog
    /*!
    this class keeps the watchdog on the arduino alive
    */
class cWatchdogGuard : public adtf::cTimeTriggeredFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_WATCHDOGGUARD_FILTER, "AADC Watchdog Trigger", OBJCAT_Tool,  "AADC Watchdog Trigger", 1, 0,0, "Beta Version");

    protected:
        /*!outputpin which sends periodically the value tTrue */
        cOutputPin        m_oOutputPin;

    public:
        cWatchdogGuard(const tChar* __info);
        virtual ~cWatchdogGuard();

    protected: // overwrites cFilter
        tResult Cycle(__exception=NULL);
        tResult Init(tInitStage eStage, __exception = NULL);


    private:
        inline tResult CreateOutputPins(__exception=NULL);
        
        /*! Coder Descriptor for the pins*/
        cObjectPtr<IMediaTypeDescription> m_pCoderDescOutput;
};

//*************************************************************************************************

#endif // _ARDUINO_FILTER_H_

