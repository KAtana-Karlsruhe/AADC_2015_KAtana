/**
 *
 * ADTF Demo Filter.
 *    This is only for demo a Filter application.
 *
 * @file
 * Copyright &copy; Audi Electronics Venture GmbH. All rights reserved
 *
 * $Author: ABOEMI9 $
 * $Date: 2013-08-27 16:36:26 +0200 (Tue, 27 Aug 2013) $
 * $Revision: 40298 $
 *
 * @remarks
 *
 */
#ifndef _DEMO_EMPTY_FILTER_HEADER_
#define _DEMO_EMPTY_FILTER_HEADER_


#define OID_ADTF_UDP_REMOTE  "adtf.aadc.udp.remote"

const u_int8_t CMD_CONTROL = 0x5e;
const u_int8_t CMD_ALIVE = 0x3f;
const u_int8_t CMD_TRIGGER = 0x4e;

class UdpRemote : public cTimeTriggeredFilter
{
    ADTF_FILTER(OID_ADTF_UDP_REMOTE, "Remote control via udp", OBJCAT_BridgeDevice)

    private: //private members
        class cReceiveThread : public cKernelThread
        {
            protected:
                UdpRemote*    m_pParent;

            public:
                cReceiveThread();
                tResult SetParent(UdpRemote* pParent);

            protected: //overwrite ThreadFunc of cKernelThread
                tResult ThreadFunc();
        };

        static const tInt           MAX_PACKET_SIZE = 512;

        cOutputPin                  m_output_steering;
        cOutputPin                  m_output_speed;
        cObjectPtr<IMediaTypeDescription>	m_pCoderDesc;

        //! TEST TRIGGER OUTPUT PIN
        cOutputPin        m_opin_trigger_test;


        cDatagramSocket             m_oRecvSocket;
        cReceiveThread              m_oReceiveThread;
        int8_t*                      m_pBuffer;


    private:
        tResult Cycle(__exception = NULL);
        bool m_alive_received;

    public: //common implementation
        UdpRemote(const tChar* __info);
        virtual ~UdpRemote();

    public: // overwrites cFilter
        tResult Init(tInitStage eStage, __exception = NULL);
        tResult Start(__exception = NULL);
        tResult Stop(__exception = NULL);
        tResult Shutdown(tInitStage eStage, __exception = NULL);

    public:
        tResult Receive();

    private:
        void outputSpeed(int8_t val);
        void outputSteering(int8_t val);

        /*! Coder Descriptor for the output pins*/
        cObjectPtr<IMediaTypeDescription> m_pCoderDescSignalOutput;

        //! output trigger byte
        tResult outputTrigger(u_int8_t value);

};

#endif // _DEMO_EMPTY_FILTER_HEADER_
