
#ifndef __STD_INCLUDES_HEADER
#define __STD_INCLUDES_HEADER

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
using namespace adtf;

#include <adtf_graphics.h>
using namespace adtf_graphics;

#include "aadc.h"

#include <QtGui/QtGui>
#include <QtCore/QtCore>
#ifdef WIN32
#ifdef _DEBUG
    #pragma comment(lib, "qtmaind.lib")
    #pragma comment(lib, "qtcored4.lib")
    #pragma comment(lib, "qtguid4.lib")
#else // _DEBUG
    #pragma comment(lib, "qtmain.lib")
    #pragma comment(lib, "qtcore4.lib")
    #pragma comment(lib, "qtgui4.lib")
#endif
#endif
#endif // __STD_INCLUDES_HEADER
