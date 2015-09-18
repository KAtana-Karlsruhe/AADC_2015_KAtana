/**
 * KATANA Demo Patch Generator Filter.
 * $Author: philipp $
 */
#ifndef _DEMO_PATCH_GENERATOR_FILTER_HEADER_
#define _DEMO_PATCH_GENERATOR_FILTER_HEADER_

#define OID_ADTF_DEMO_PATCH_GENERATOR_FILTER "adtf.aadc.demo.patchGenerator"

#define MIN_SIGNAL   2
#define MAX_SIGNAL 200

#include "katanaCommon/katanaCommon.h"
#include "PinConversions.h"
#include <Pose.h>

class cDemoPatchFilter : public adtf::cTimeTriggeredFilter
{
    ADTF_FILTER(_DEMO_PATCH_GENERATOR_FILTER_HEADER_, "KATANA Demo Patch Generator", adtf::OBJCAT_Tool)

    public:

    protected:
        cOutputPin m_oPinOutput; // output pin for signal data

        cInputPin m_ipin_pose;   // input pin for current pose

	tInt		m_nMaxSignal;
	tInt32		m_numberDataValues;
	tInt32          m_numberOfPatches;
	tInt32		m_cycle;
	katana::Point*	m_points;
	tFloat32*	m_thetas;
	tInt32*		m_types;

    public: // construction
        cDemoPatchFilter(const tChar* __info);
        virtual ~cDemoPatchFilter();

    public: // overrides cFilter
        tResult Init(tInitStage eStage, __exception);
	tResult Shutdown(tInitStage eStage, __exception);

	//! implements IPinEventSink
	tResult OnPinEvent(IPin* pSource,
			   tInt nEventCode,
			   tInt nParam1,
			   tInt nParam2,
			   IMediaSample* pMediaSample);

    protected: // implement cTimeTriggeredFilter
        tResult Cycle(__exception = NULL);

    private:
	tResult CreateOutputPins(__exception = NULL);
	tResult initializePatchArray();
	tResult calculateNumberOfPatches(cFilename fileConfig);

	//!
	bool m_enable_field_of_view;

	//! Current pose: transmit patches only in field of vision
	katana::Pose m_pose;

	//! Define simple rectangular field of view
	katana::Point m_upper_left;
	katana::Point m_down_right;
};

//*************************************************************************************************
#endif
