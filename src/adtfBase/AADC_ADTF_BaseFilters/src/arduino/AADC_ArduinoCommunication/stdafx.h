#ifndef _STDAFX_HEADER
#define _STDAFX_HEADER


#ifndef YOURHEADER_INCLUSION_GUARD
#define YOURHEADER_INCLUSION_GUARD

    #ifdef WIN32
    #    ifdef GetObject
    #        define MYPROJECT_MACRO_GETOBJECT_WAS_DEFINED
    #    endif
    #    undef GetObject
    #endif

    #ifndef WIN32
        #include <sys/ioctl.h>
        #include <unistd.h>     // UNIX standard function definitions
        #include <termios.h>    // POSIX terminal control definitions
        #include <strings.h>    //linuxlib for bzero
        #include <fcntl.h>      // File control definitions
        #define BAUDRATE B115200
        #define _POSIX_SOURCE 1
    #else
        #define WIN32_LEAN_AND_MEAN
        #include <windows.h>
        #include <windowsx.h>
        #include <commctrl.h>
        #include <stdlib.h>    //lib for exit command
        #define SPM_BYTESTOREAD        WM_USER + 0x11
    #endif

    #ifdef WIN32
    #    if defined(MYPROJECT_MACRO_GETOBJECT_WAS_DEFINED)
    #        undef MYPROJECT_MACRO_GETOBJECT_WAS_DEFINED
    #        define GetObject GetObjectA
    #    endif
    #endif

#endif //YOURHEADER_INCLUSION_GUARD

// ADTF includes
#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
using namespace adtf;

// AADC includes
#include "aadc.h"
#include "arduinoProtocol.h"

#endif // _STDAFX_HEADER

