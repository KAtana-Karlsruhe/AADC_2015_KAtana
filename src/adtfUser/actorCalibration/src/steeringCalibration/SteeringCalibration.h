/**
 *
 * ADTF Steering Calibration Filter.
 *
 */
#ifndef _STEERING_CALIBRATION_FILTER_H_
#define _STEERING_CALIBRATION_FILTER_H_

#define OID_ADTF_TEMPLATE_FILTER "adtf.aadc.steeringCalibration_filter"


//*************************************************************************************************
class cSteeringCalibrationFilter : public adtf::cFilter
{
    ADTF_FILTER(OID_ADTF_TEMPLATE_FILTER, "KATANA Steering calibration Filter", adtf::OBJCAT_DataFilter);

protected:
    cInputPin    m_oInput;
    cOutputPin    m_oOutput;

public:
    cSteeringCalibrationFilter(const tChar* __info);
    virtual ~cSteeringCalibrationFilter();

protected:
    tResult Init(tInitStage eStage, __exception);
    tResult Shutdown(tInitStage eStage, __exception);

    // implements IPinEventSink
    tResult OnPinEvent(IPin* pSource,
                       tInt nEventCode,
                       tInt nParam1,
                       tInt nParam2,
                       IMediaSample* pMediaSample);
    private:
	tResult CreateInputPins(__exception = NULL);
	tResult CreateOutputPins(__exception = NULL);
	tFloat32 m_maxValue;
    	cObjectPtr<IMediaTypeDescription> m_pCoderDescSignal;
};

//*************************************************************************************************
#endif // _STEERING_CALIBRATION_FILTER_H_
