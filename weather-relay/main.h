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



//#define PORT_DEVICE "/dev/cu.usbserial-0001"  // blue usb->serial adapter, use command line interface instead of changing this. this is here to remind me of the port name.
#define PORT_DEVICE   "/dev/serial0"
//#define RAIN_DEVICE   "/dev/ttyUSB0"   // this is how the ESP32 shows up on the RPi
#define RAIN_DEVICE   "/dev/ttyACM0"        // this is on the Feather M0+RFM69 board
#define PORT_ERROR    -1
#define PROGRAM_NAME  "folabs-wx-relay"
#define VERSION       "110"


#define pascal2inchHg    0.0002953
#define millibar2inchHg  0.02953

#define c2f( a )                 (((a) * 1.8000) + 32)
#define ms2mph( a )              ((a) * 2.23694)
#define inHg2millibars( a )      ((a) * 33.8639)
#define millimeter2inch( a )     ((a) * 0.0393700787402)
#define rawRainCount2mm( a )     ((a) * 0.5)
#define rawRainCount2inches( a ) ((a) * 0.02)

// https://www.daculaweather.com/stuff/CWOP_Guide.pdf has all the intervals, etc...
#define kTelemDelaySecs     0
#define kStatusDelaySecs    20
#define kWxDelaySecs        40

#define kTempLowBar         -60.0f
#define kTempHighBar        130.0f
#define kTempTemporalLimit  35.0f // °F -- !!@ these are supposed to be over an hour but we only really keep about 10 minutes of data
#define kHumidityLowBar     0
#define kHumidityHighBar    100
#define kWindLowBar         0
#define kWindHighBar        100
#define kWindTemporalLimit  17.39130 // this is 20 knots in mph -- !!@ see note above about temporal being 10 minutes instead of hour.
#define kRainLowBar         0
#define kRainHighBar        9999
#define kRainTemporalLimit        50.0     // this is in millimeters of rain collected, you can't just go from 1 mm to 50 mm ya know?
#define kRainTemporalLimitInches  2.0


//-----------------------------------------------------------------------------------------------------------------------------

// for debugging otherwise we spend a lifetime waiting for data to debug with...
#define kSendInterval_debug    30
#define kParamsInterval_debug  60
#define kStatusInterval_debug  60

#define kTempPeriod_debug       15
#define kIntTempPeriod_debug    15
#define kWindPeriod_debug       20
#define kGustPeriod_debug       30
#define kBaroPeriod_debug       15
#define kHumiPeriod_debug       15
#define kAirPeriod_debug        15
#define kAQIPeriod_debug        15
#define kRain24HrPeriod_debug   60 * 10 // 10 minutes
#define kRainLastHrPeriod_debug 60


#define kSendInterval    60 * 5        // 5 minutes
#define kParamsInterval  60 * 60 * 2   // every two hours
#define kStatusInterval  60 * 10 + 15  // every ten minutes + 15 seconds offset

#define kTempPeriod       60 * 5   // 5 minute average
#define kIntTempPeriod    60 * 5   // 5 minute average
#define kWindPeriod       60 * 2   // 2 minute average
#define kGustPeriod       60 * 10  // 10 minute max wind gust
#define kBaroPeriod       60       // low for the minute period
#define kHumiPeriod       60
#define kAirPeriod        60
#define kAQIPeriod        60 * 60 * 24
#define kRain24HrPeriod   60 * 60 * 24
#define kRainLastHrPeriod 60 * 60


//-----------------------------------------------------------------------------------------------------------------------------


#define kLongestInterval kAQIPeriod

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
int open_serial_port( const char* serial_port_device, int port_speed );

#endif // !_H_main

// EOF
