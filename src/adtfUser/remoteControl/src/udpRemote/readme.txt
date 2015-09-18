 /**
 *
 * Demo UDP Filter
 *
 * @file
 * Copyright &copy; Audi Electronics Venture GmbH. All rights reserved.
 *
 * @author               $Author: voigtlpi $
 * @date                 $Date: 2009-07-16 15:36:37 +0200 (Do, 16 Jul 2009) $
 * @version              $Revision: 10093 $
 *
 * @remarks
 *
 */
 
/**
 * \page page_demo_udp Demo UDP Filter
 *
 * Implements a UDP filter
 * 
 * \par Location
 * \code
 *    ./src/examples/src/filters/demo_udp/
 * \endcode
 *
 * \par Build Environment
 * To see how to set up the build environment have a look at this page @ref page_cmake_overview
 *
 * \par This example shows:
 * \li how to implement a common ADTF Filter for processing data
 * \li how to deal with a received MediaSample to get the specific data
 * \li how to send or receive data over a socket
 *
 * \par Call Sequence
 * For a better understanding of the sequence of calls while using base device
 * classes of ADTF SDK, the ethernet device implementations are added to the
 * installation as source files.\n
 * So you can debug through the call sequence.
 * 
 * \par The Header for the Demo UDP Filter
 * \include demoudpfilter.h
 *
 * \par The Implementation the Demo UDP Filter
 * \include demoudpfilter.cpp
 * 
 */