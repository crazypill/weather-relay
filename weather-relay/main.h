//
//  main.h
//  weather-relay
//
//  Created by Alex Lelievre on 7/20/20.
//  Copyright © 2020 Far Out Labs. All rights reserved.
//

#ifndef _H_main
#define _H_main

#include <stdbool.h>
#include <stdint.h>


// define this to see incoming weather data from weather sensors...
//#define TRACE_INCOMING_WX
//#define TRACE_STATS
#define DEBUG_PERIODS     // shortens the time we wait to test the code




//#define PORT_DEVICE "/dev/cu.usbserial-0001"  // blue usb->serial adapter, use command line interface instead of changing this. this is here to remind me of the port name.
#define PORT_DEVICE   "/dev/serial0"
#define PORT_ERROR    -1
#define PROGRAM_NAME  "folabs-wx-relay"
#define VERSION       "100"


#define pascal2inchHg    0.0002953
#define millibar2inchHg  0.02953

#define c2f( a ) (((a) * 1.8000) + 32)
#define ms2mph( a ) ((a) * 2.23694)
#define inHg2millibars( a ) ((a) * 33.8639)

// https://www.daculaweather.com/stuff/CWOP_Guide.pdf has all the intervals, etc...
#define kTelemDelaySecs     15  // 15 seconds after weather tx - used to stagger messages so they don't pile up on each other
#define kStatusDelaySecs    30

#ifdef DEBUG_PERIODS
// for debugging otherwise we spend a lifetime waiting for data to debug with...
#define kSendInterval    30
#define kParamsInterval  60
#define kStatusInterval  60

#define kTempPeriod    15
#define kIntTempPeriod 15
#define kWindPeriod    20
#define kGustPeriod    30
#define kBaroPeriod    15
#define kHumiPeriod    15
#define kAirPeriod     15

#else

#define kSendInterval    60 * 5        // 5 minutes
#define kParamsInterval  60 * 60 * 2   // every two hours
#define kStatusInterval  60 * 10 + 15  // every ten minutes + 15 seconds offset

#define kTempPeriod    60 * 5   // 5 minute average
#define kIntTempPeriod 60 * 5   // 5 minute average
#define kWindPeriod    60 * 2   // every 2 minutes we reset the average wind speed and direction
#define kGustPeriod    60 * 10  // every 10 minutes we reset the max wind gust to 0
#define kBaroPeriod    60
#define kHumiPeriod    60
#define kAirPeriod     60

#endif

#define kLongestInterval kGustPeriod

#ifdef TRACE_INCOMING_WX
#define trace printf
#else
#define trace nullprint
#endif

#ifdef TRACE_STATS
#define stats printf
#else
#define stats nullprint
#endif

#ifndef BUFSIZE
#define BUFSIZE 1025
#endif

#define AX25_MAX_ADDRS 10    /* Destination, Source, 8 digipeaters. */
#define AX25_MAX_INFO_LEN 2048    /* Maximum size for APRS. */
#define AX25_MAX_PACKET_LEN ( AX25_MAX_ADDRS * 7 + 2 + 3 + AX25_MAX_INFO_LEN)


bool debug_mode( void );
void log_error( const char* format, ... );
void log_unix_error( const char* prefix );

#endif // !_H_main

// EOF
