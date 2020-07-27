//
//  main.c
//  weather-relay
//
//  Created by Alex Lelievre on 6/16/20.
//  Copyright © 2020 Far Out Labs. All rights reserved.
//

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>

#include <strings.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/termios.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <assert.h>
#include <netdb.h>
#include <getopt.h>

#include "main.h"

#include "wx_thread.h"
#include "TXDecoderFrame.h"
#include "aprs-wx.h"
#include "aprs-is.h"

#include "ax25_pad.h"
#include "kiss_frame.h"


//#define TRACE_STATS
//#define TRACE_AIR_STATS

#define kCallSign "K6LOT-13"

static time_t s_lastSendTime    = 0;
static time_t s_lastWindTime    = 0;
static time_t s_lastGustTime    = 0;
static time_t s_lastBaroTime    = 0;
static time_t s_lastTempTime    = 0;
static time_t s_lastIntTempTime = 0;
static time_t s_lastHumiTime    = 0;
static time_t s_lastAirTime     = 0;
static time_t s_lastParamsTime  = 0;

static float s_localOffsetInHg = 0.33f;
static float s_localTempErrorC = 2.033333333333333;

static bool s_debug = false;

static const char* s_logFilePath = NULL;
static FILE*       s_logFile     = NULL;

static const char*  s_kiss_server  = "localhost";
static uint16_t     s_kiss_port    = 8001;
static uint8_t      s_num_retries  = 5;
static uint8_t      s_sequence_num = 0;

static wx_thread_return_t sendToRadio_thread_entry( void* args );
static wx_thread_return_t sendPacket_thread_entry( void* args );

static int  connectToDireWolf( void );
static int  sendToRadio( const char* p );
static int  send_to_kiss_tnc( int chan, int cmd, char *data, int dlen );
static void transmit_wx_data( const Frame* min, const Frame* max, const Frame* ave );
static void transmit_air_data( const Frame* min, const Frame* max, const Frame* ave );


#pragma mark -


void nullprint( const char* format, ... )
{
    
}


char* copy_string( const char* stringToCopy )
{
    if( !stringToCopy )
        return NULL;
    
    size_t bufSize = strlen( stringToCopy ) + 1;
    char* newString = (char*)malloc( bufSize );
    if( newString )
    {
        strcpy( newString, stringToCopy );
        newString[bufSize - 1] = '\0';
    }
    return newString;
}


void buffer_input_flush()
{
    int c;
     // This will eat up all other characters
    while( (c = getchar()) != EOF && c != '\n' )
        ;
}


void printTime( int printNewline )
{
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    if( printNewline )
        printf("%d-%02d-%02d %02d:%02d:%02d\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    else
        printf("%d-%02d-%02d %02d:%02d:%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
}


void printTimePlus5()
{
  time_t t = time(NULL);
  struct tm tm = *localtime(&t);
  printf("%d-%02d-%02d %02d:%02d:%02d\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min + 5, tm.tm_sec);
}


time_t timeGetTimeSec()
{
    time_t rawtime = 0;
    return time( &rawtime );
}


uint8_t imax( uint8_t a, uint8_t b )
{
    return a > b ? a : b;
}


uint8_t imin( uint8_t a, uint8_t b )
{
    return a < b ? a : b;
}




#pragma mark -


void updateStats( Frame* data, Frame* min, Frame* max, Frame* ave )
{
    if( data->flags & kDataFlag_temp )
    {
        if( timeGetTimeSec() > s_lastTempTime + kTempInterval )
        {
            ave->tempC = 0;
            s_lastTempTime = timeGetTimeSec();
        }

        // check for no data before calculating mean
        if( ave->tempC == 0.0 )
            ave->tempC = data->tempC;

        ave->tempC = (data->tempC + ave->tempC) * 0.5f;
#ifdef TRACE_STATS
        printTime( false );
        stats( " temp average: %0.2f°F, time left: %ld\n", c2f( ave->tempC ), kTempInterval - (timeGetTimeSec() - s_lastTempTime) );
#endif
    }

    if( data->flags & kDataFlag_intTemp )
    {
        if( timeGetTimeSec() > s_lastIntTempTime + kIntTempInterval )
        {
            ave->intTempC = 0;
            s_lastIntTempTime = timeGetTimeSec();
        }

        // check for no data before calculating mean
        if( ave->intTempC == 0.0 )
            ave->intTempC = data->intTempC;

        ave->intTempC = (data->intTempC + ave->intTempC) * 0.5f;
#ifdef TRACE_STATS
        printTime( false );
        stats( " int temp average: %0.2f°F, time left: %ld\n", c2f( ave->intTempC ), kIntTempInterval - (timeGetTimeSec() - s_lastIntTempTime) );
#endif
    }

    
    if( data->flags & kDataFlag_humidity )
    {
        if( timeGetTimeSec() > s_lastHumiTime + kHumiInterval )
        {
            ave->humidity = 0;
            s_lastHumiTime = timeGetTimeSec();
        }

        // check for no data before calculating mean
        if( ave->humidity == 0 )
            ave->humidity = data->humidity;
        ave->humidity = (data->humidity + ave->humidity) / 2;
#ifdef TRACE_STATS
        printTime( false );
        stats( " humidity average: %d%%, time left: %ld\n", ave->humidity, kHumiInterval - (timeGetTimeSec() - s_lastHumiTime) );
#endif
    }
    
    // for wind we want the ave over the interval period
    if( data->flags & kDataFlag_wind )
    {
        if( ms2mph( data->windSpeedMs ) > 100 )
        {
            // blow off this entire frame of data- it's probably all wrong (except for baro and int temp)
            printTime( false );
            printf( " wind speed too high[%0.2f°]: %0.2f mph, time left: %ld\n", data->windDirection, ms2mph( data->windSpeedMs ), kWindInterval - (timeGetTimeSec() - s_lastWindTime) );
            data->flags &= ~kDataFlag_wind;
            return;
        }

        if( data->windDirection < 0 || data->windDirection > 360 )
        {
            // blow off this entire frame of data- it's probably all wrong
            printTime( false );
            printf( " wind direction out of range [%0.2f°]: %0.2f mph, time left: %ld\n", data->windDirection, ms2mph( data->windSpeedMs ), kWindInterval - (timeGetTimeSec() - s_lastWindTime) );
            data->flags &= ~kDataFlag_wind;
            return;
        }

        
        if( timeGetTimeSec() > s_lastWindTime + kWindInterval )
        {
            ave->windSpeedMs = 0;
            ave->windDirection = 0;
            s_lastWindTime = timeGetTimeSec();
        }

        // check for no data before calculating mean
        if( ave->windSpeedMs == 0.0 )
            ave->windSpeedMs = data->windSpeedMs;

        if( ave->windDirection == 0.0 )
            ave->windDirection = data->windDirection;

        ave->windSpeedMs = (data->windSpeedMs + ave->windSpeedMs) * 0.5f;
        ave->windDirection = (data->windDirection + ave->windDirection) * 0.5f;
#ifdef TRACE_STATS
        printTime( false );
        stats( " wind average[%0.2f°]: %0.2f mph, time left: %ld\n", ave->windDirection, ms2mph( ave->windSpeedMs ), kWindInterval - (timeGetTimeSec() - s_lastWindTime) );
#endif
    }

    // for gusts we want the max instantaneous over the interval period
    if( data->flags & kDataFlag_gust )
    {
        // I saw a 700 MPH wind gust go by which seems nuts... so trap that error
        if( ms2mph( data->windGustMs ) > 100 )
        {
            // blow off this entire frame of data- it's probably all wrong (except for baro and int temp)
            printTime( false );
            printf( " wind gust too high[%0.2f°]: %0.2f mph, time left: %ld\n", ave->windDirection, ms2mph( ave->windGustMs ), kGustInterval - (timeGetTimeSec() - s_lastGustTime) );
            data->flags &= ~kDataFlag_gust;
            return;
        }
        
        // we create a 10 minute window of instantaneous gust measurements
        if( timeGetTimeSec() > s_lastGustTime + kGustInterval )
        {
            max->windGustMs = 0;
            s_lastGustTime = timeGetTimeSec();
        }

        max->windGustMs = fmax( data->windGustMs, max->windGustMs );
#ifdef TRACE_STATS
        printTime( false );
        stats( " gust max: %0.2f mph, time left: %ld\n", ms2mph( max->windGustMs ), kGustInterval- (timeGetTimeSec() - s_lastGustTime) );
#endif
    }


    if( data->flags & kDataFlag_pressure )
    {
        if( timeGetTimeSec() > s_lastBaroTime + kBaroInterval )
        {
            min->pressure = 0;
            s_lastBaroTime = timeGetTimeSec();
        }

        // check for no data before calculating min
        if( min->pressure == 0.0 )
            min->pressure = data->pressure;

        min->pressure = fmin( data->pressure, min->pressure );
#ifdef TRACE_STATS
        printTime( false );
        stats( " pressure min: %0.2f InHg, time left: %ld\n",(min->pressure * millibar2inchHg) + s_localOffsetInHg, kBaroInterval - (timeGetTimeSec() - s_lastBaroTime) );
#endif
    }


    if( data->flags & kDataFlag_airQuality )
    {
        if( timeGetTimeSec() > s_lastAirTime + kAirInterval )
        {
            ave->pm10_standard = 0;
            ave->pm25_standard = 0;
            ave->pm100_standard = 0;
            ave->pm10_env = 0;
            ave->pm25_env = 0;
            ave->pm100_env = 0;
            ave->particles_03um = 0;
            ave->particles_05um = 0;
            ave->particles_10um = 0;
            ave->particles_25um = 0;
            ave->particles_50um = 0;
            ave->particles_100um = 0;
            s_lastAirTime = timeGetTimeSec();
        }

        // check for no data before calculating mean
        if( ave->pm10_standard == 0 )
            ave->pm10_standard = data->pm10_standard;
        ave->pm10_standard = (data->pm10_standard + ave->pm10_standard) / 2;

        if( ave->pm25_standard == 0 )
            ave->pm25_standard = data->pm25_standard;
        ave->pm25_standard = (data->pm25_standard + ave->pm25_standard) / 2;

        if( ave->pm100_standard == 0 )
            ave->pm100_standard = data->pm100_standard;
        ave->pm100_standard = (data->pm100_standard + ave->pm100_standard) / 2;

        if( ave->pm10_env == 0 )
            ave->pm10_env = data->pm10_env;
        ave->pm10_env = (data->pm10_env + ave->pm10_env) / 2;

        if( ave->pm25_env == 0 )
            ave->pm25_env = data->pm25_env;
        ave->pm25_env = (data->pm25_env + ave->pm25_env) / 2;

        if( ave->pm100_env == 0 )
            ave->pm100_env = data->pm100_env;
        ave->pm100_env = (data->pm100_env + ave->pm100_env) / 2;

        if( ave->particles_03um == 0 )
            ave->particles_03um = data->particles_03um;
        ave->particles_03um = (data->particles_03um + ave->particles_03um) / 2;

        if( ave->particles_05um == 0 )
            ave->particles_05um = data->particles_05um;
        ave->particles_05um = (data->particles_05um + ave->particles_05um) / 2;

        if( ave->particles_10um == 0 )
            ave->particles_10um = data->particles_10um;
        ave->particles_10um = (data->particles_10um + ave->particles_10um) / 2;

        if( ave->particles_25um == 0 )
            ave->particles_25um = data->particles_25um;
        ave->particles_25um = (data->particles_25um + ave->particles_25um) / 2;

        if( ave->particles_50um == 0 )
            ave->particles_50um = data->particles_50um;
        ave->particles_50um = (data->particles_50um + ave->particles_50um) / 2;

        if( ave->particles_100um == 0 )
            ave->particles_100um = data->particles_100um;
        ave->particles_100um = (data->particles_100um + ave->particles_100um) / 2;

        
#ifdef TRACE_AIR_STATS
        log_error( "averages pm10: %03d (%03d), pm25: %03d (%03d), pm100: %03d (%03d), 3um: %03d, 5um: %03d, 10um: %03d, 25um: %03d, 50um: %03d, 100um: %03d\n", ave->pm10_standard, ave->pm10_env, ave->pm25_standard, ave->pm25_env, ave->pm100_standard, ave->pm100_env, ave->particles_03um, ave->particles_05um, ave->particles_10um, ave->particles_25um, ave->particles_50um, ave->particles_100um );
#endif
    }

    
//    if( data.flags & kDataFlag_rain )
//        printf( "rain:       %g\n", data.rain );


}


void printFullWeather( const Frame* inst, const Frame* min, const Frame* max, const Frame* ave )
{
    // only show this stuff if in debug mode
    if( !s_debug )
        return;
    
    log_error( "     wind[%06.2f°]: %0.2f mph,     gust: %0.2f mph --     temp: %0.2f°F,     humidity: %2d%%,     pressure: %0.3f InHg,     int temp: %0.2f°F, rain: %g\n", inst->windDirection, ms2mph( inst->windSpeedMs ), ms2mph( inst->windGustMs ), c2f( inst->tempC ), inst->humidity, (inst->pressure * millibar2inchHg) + s_localOffsetInHg, c2f( inst->intTempC - s_localTempErrorC ), 0.0 );
    log_error( " avg wind[%06.2f°]: %0.2f mph, max gust: %0.2f mph -- ave temp: %0.2f°F, ave humidity: %2d%%, min pressure: %0.3f InHg  ave int temp: %0.2f°F\n",            ave->windDirection, ms2mph( ave->windSpeedMs ), ms2mph( max->windGustMs ), c2f( ave->tempC ), ave->humidity, (min->pressure * millibar2inchHg) + s_localOffsetInHg, c2f( ave->intTempC - s_localTempErrorC ) );
    log_error( " pm10: %03d (%03d), pm25: %03d (%03d), pm100: %03d (%03d),  3um: %03d,  5um: %03d,  10um: %03d,  25um: %03d,  50um: %03d,  100um: %03d\n", inst->pm10_standard, inst->pm10_env, inst->pm25_standard, inst->pm25_env, inst->pm100_standard, inst->pm100_env, inst->particles_03um, inst->particles_05um, inst->particles_10um, inst->particles_25um, inst->particles_50um, inst->particles_100um );
}


void printCurrentWeather( const Frame* min, const Frame* max, const Frame* ave )
{
    // only show this stuff if in debug mode
    if( !s_debug )
        return;

    printf( "Wind[%06.2f°]: %0.2f mph, gust: %0.2f mph, temp: %0.2f°F, humidity: %2d%%, pressure: %0.3f InHg, int temp: %0.2f°F, rain: %g\n", ave->windDirection, ms2mph( ave->windSpeedMs ), ms2mph( max->windGustMs ), c2f( ave->tempC ), ave->humidity, (min->pressure * millibar2inchHg) + s_localOffsetInHg, c2f( ave->intTempC - s_localTempErrorC ), 0.0 );
}


void version( int argc, const char* argv[] )
{
    printf( "%s, version %s", PROGRAM_NAME, "1.0.0" );
#ifdef DEBUG
    fputs(", compiled with debugging output", stdout);
#endif
    puts(".\n\
        Copyright (c) 2020 Far Out Labs, LLC.\n\
        This program comes with ABSOLUTELY NO WARRANTY. This is free software, and you\n\
        are welcome to redistribute it under certain conditions.  See the GNU General\n\
        Public License (version 3.0) for more details.");
    return;
}


void usage( int argc, const char* argv[] )
{
    printf( "Typical usage: %s --kiss \"10.0.1.208\"\n", PROGRAM_NAME );
    return;
}



void help( int argc, const char* argv[] )
{
    version( argc, argv );
    puts("");
    usage( argc, argv );
    puts( "\n\
        Special parameters:\n\
            -H, --help                 Show this help and exit.\n\
            -v, --version              Show version and licensing information, and exit.\n\
            -d, --debug                Show the incoming radio data and packet sends.\n\
            -l, --log                  Log errors and debug info to this file.\n\
        Tuning parameters:\n\
            -b, --baro                 Set the barometric pressure offset in InHg.\n\
            -t, --temp                 Set the interior temperature offset in °C.\n\
        Override parameters:\n\
            -k, --kiss                 Set the server we want to use, defaults to localhost.\n\
            -p, --port                 Set the port we want to use, defaults to 8001.\n\
        " );
}


void log_error( const char* format, ... )
{
    char buf[2048] = {0};
    va_list vaList;
    va_start( vaList, format );
    vsprintf( buf, format, vaList );
    va_end( vaList );
    
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    
    if( s_logFile )
    {
        fprintf( s_logFile, "%d-%02d-%02d %02d:%02d:%02d: %s", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, buf );
        fflush( s_logFile );
    }
    
    // print to debug as well...
    if( s_debug )
        printf( "%d-%02d-%02d %02d:%02d:%02d: %s", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, buf );
}


void log_unix_error( const char* prefix )
{
    char buffer[512] = {0};
    strerror_r( errno, buffer, sizeof( buffer ) );
    
    char finalBuffer[1024] = {0};
    strcat( finalBuffer, prefix );
    strcat( finalBuffer, buffer );
    strcat( finalBuffer, "\n" );
    log_error( finalBuffer );
}



#pragma mark -

int main( int argc, const char * argv[] )
{
    // do some command processing...
    if( argc >= 2 )
    {
        int c = '\0';          /* for getopt_long() */
        int option_index = 0;  /* for getopt_long() */

        const static struct option long_options[] = {
            {"help",                    no_argument,       0, 'H'},
            {"version",                 no_argument,       0, 'v'},
            {"debug",                   no_argument,       0, 'd'},
            {"temp",                    required_argument, 0, 't'},
            {"baro",                    required_argument, 0, 'b'},
            {"log",                     required_argument, 0, 'l'},
            {"kiss",                    required_argument, 0, 'k'},
            {"port",                    required_argument, 0, 'p'},

            {0, 0, 0, 0}
            };

        while( (c = getopt_long( argc, (char* const*)argv, "Hvdt:b:l:k:p:", long_options, &option_index)) != -1 )
        {
            switch( c )
            {
                /* Complete help (-H | --help) */
                case 'H':
                    help( argc, argv );
                    return EXIT_SUCCESS;

                /* Version information (-v | --version) */
                case 'v':
                    version( argc, argv );
                    return EXIT_SUCCESS;
                    
                case 'b':
                    s_localOffsetInHg = atof( optarg );
                    break;

                case 't':
                    s_localTempErrorC = atof( optarg );
                    break;

                case 'd':
                    s_debug = true;
                    break;

                case 'l':
                    s_logFilePath = optarg;
                    break;

                case 'k':
                    s_kiss_server = optarg;
                    break;

                case 'p':
                    s_kiss_port = atoi( optarg );
                    break;
            }
        }
    }
    
    if( s_debug )
        printf( "%s, version %s -- pressure offset: %0.2f InHg, interior temp offset: %0.2f °C, kiss: %s:%d\n", PROGRAM_NAME, VERSION, s_localOffsetInHg, s_localTempErrorC, s_kiss_server, s_kiss_port );

    if( s_logFilePath && !s_logFile )
    {
        s_logFile = fopen( s_logFilePath, "a" );
        if( !s_logFile )
            log_error( "  failed to open log file: %s\n", s_logFilePath );
        if( s_debug )
        {
            printf( "logging errors to: %s\n", s_logFilePath );
            
            time_t t = time(NULL);
            struct tm tm = *localtime(&t);
            
            if( s_logFile )
                fprintf( s_logFile, "%d-%02d-%02d %02d:%02d:%02d: %s, version %s -- pressure offset: %0.2f InHg, interior temp offset: %0.2f °C, kiss: %s:%d\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, PROGRAM_NAME, VERSION, s_localOffsetInHg, s_localTempErrorC, s_kiss_server, s_kiss_port );
        }
    }
    

    // open the serial port
    bool blocking = false;

    // Settings structure old and new
    struct termios newtio;

    int fd = open( PORT_DEVICE, O_RDWR | O_NOCTTY | (blocking ? 0 : O_NDELAY) );
    if( fd < 0 )
    {
        log_error( "Failed to open serial port: %s\n", PORT_DEVICE );
        return PORT_ERROR;
    }

    bzero( &newtio, sizeof( newtio ) );

    if( cfsetispeed( &newtio, B9600 ) != 0 )
        return PORT_ERROR;
    if( cfsetospeed( &newtio, B9600 ) != 0 )
        return PORT_ERROR;

    newtio.c_cflag &= ~PARENB;
    newtio.c_cflag &= ~CSTOPB;
    newtio.c_cflag &= ~CSIZE;
    newtio.c_cflag |= CS8 | CLOCAL | CREAD;
    newtio.c_cflag &= ~CRTSCTS;

    // Hardware control of port
    newtio.c_cc[VTIME] = blocking ? 1 : 0; // Read-timout 100ms when blocking
    newtio.c_cc[VMIN] = 0;

    tcflush( fd, TCIFLUSH );

    // Acquire new port settings
    if( tcsetattr( fd, TCSANOW, &newtio ) != 0 )
        puts( strerror( errno ) );

    if( fd == -1 )
        return PORT_ERROR;

    trace( "%s: reading from serial port: %s...\n\n", PROGRAM_NAME, PORT_DEVICE );

    // this holds all the min/max/averages
    Frame minFrame = {};
    Frame maxFrame = {};
    Frame aveFrame = {};

    // primary weather frame that is used to create APRS message
    Frame wxFrame = {};

    uint8_t receivedFlags = 0;
    ssize_t result = 0;
    while( 1 )
    {
        Frame frame = {};

        result = read( fd, &frame, sizeof( frame ) );
        if( result == sizeof( frame ) )
        {
            // doing this first allows us to turn off flags for bad measurements so this code skips them too-
            updateStats( &frame, &minFrame, &maxFrame, &aveFrame );
            
#ifdef TRACE_INCOMING_WX
            printTime( false );
#endif
            trace( " station_id: 0x%x", frame.station_id );
            
            // read flags
            if( frame.flags & kDataFlag_temp )
            {
                trace( ", temp: %0.2f°F", c2f( frame.tempC ) );
                wxFrame.tempC = frame.tempC;
            }

            if( frame.flags & kDataFlag_humidity )
            {
                trace( ", humidity: %d%%", frame.humidity );
                wxFrame.humidity = frame.humidity;
            }

            if( frame.flags & kDataFlag_wind )
            {
                trace( ", wind: %0.2f mph, dir: %0.2f degrees", ms2mph( frame.windSpeedMs ), frame.windDirection );
                wxFrame.windSpeedMs = frame.windSpeedMs;
                wxFrame.windDirection = frame.windDirection;
            }

            if( frame.flags & kDataFlag_gust )
            {
                trace( ", gust: %0.2f mph", ms2mph( frame.windGustMs ) );
                wxFrame.windGustMs = frame.windGustMs;
            }

            if( frame.flags & kDataFlag_rain )
            {
                trace( ", rain: %g", frame.rain );
                wxFrame.rain = frame.rain;
            }

            if( frame.flags & kDataFlag_intTemp )
            {
                trace( ", int temp: %0.2f°F", c2f( frame.intTempC - s_localTempErrorC ) );
                wxFrame.intTempC = frame.intTempC;
            }

            if( frame.flags & kDataFlag_pressure )
            {
                trace( ", pressure: %g InHg", (frame.pressure * millibar2inchHg) + s_localOffsetInHg );
                wxFrame.pressure = frame.pressure;
            }
            
            if( frame.flags & kDataFlag_airQuality )
            {
                trace( "          pm10: %03d (%03d), pm25: %03d (%03d), pm100: %03d (%03d), 3um: %03d, 5um: %03d, 10um: %03d, 25um: %03d, 50um: %03d, 100um: %03d\n", frame.pm10_standard, frame.pm10_env, frame.pm25_standard, frame.pm25_env, frame.pm100_standard, frame.pm100_env, frame.particles_03um, frame.particles_05um, frame.particles_10um, frame.particles_25um, frame.particles_50um, frame.particles_100um );
                wxFrame.pm10_standard = frame.pm10_standard;
                wxFrame.pm10_env = frame.pm10_env;
                wxFrame.pm25_standard = frame.pm25_standard;
                wxFrame.pm25_env = frame.pm25_env;
                wxFrame.pm100_standard = frame.pm100_standard;
                wxFrame.pm100_env = frame.pm100_env;
                wxFrame.particles_03um = frame.particles_03um;
                wxFrame.particles_05um = frame.particles_05um;
                wxFrame.particles_10um = frame.particles_10um;
                wxFrame.particles_25um = frame.particles_25um;
                wxFrame.particles_50um = frame.particles_50um;
                wxFrame.particles_100um = frame.particles_100um;
            }

            trace( "\n" );
            printFullWeather( &wxFrame, &minFrame, &maxFrame, &aveFrame );

            // ok keep track of all the weather data we received, lets only send a packet once we have all the weather data
            // and at least 5 minutes has passed...  !!@ also need to change averaging code to use average up to the instant
            // using saved data...  right now have good code to deal with not having enough data for averages.
            receivedFlags |= frame.flags;
            if( (receivedFlags & 0x7F) == 0x7F )
            {
                if( timeGetTimeSec() > s_lastSendTime + kSendInterval )
                {
                    transmit_wx_data( &minFrame, &maxFrame, &aveFrame );
                    transmit_air_data( &minFrame, &maxFrame, &aveFrame );
                    s_lastSendTime = timeGetTimeSec();
                }
            }
        }
        sleep( 1 );
    }
    
    // loop never finishes so this code never executes...
    if( s_logFile )
        fclose( s_logFile );
    return EXIT_SUCCESS;
}



#pragma mark -


wx_thread_return_t sendPacket_thread_entry( void* args )
{
    const char* packetToSend = (const char*)args;
    if( !packetToSend )
        wx_thread_return();

    bool success = false;
    for( int i = 0; i < s_num_retries; i++ )
    {
        // send packet to APRS-IS directly...  oh btw, if you use this code, please get your own callsign and passcode!  PLEASE
        int err = sendPacket( "noam.aprs2.net", 10152, kCallSign, "8347", packetToSend );
        if( err == 0 )
        {
            log_error( "packet sent: %s\n", packetToSend );
            success = true;
            break;
        }
        else
            log_error( "packet failed to send to APRS-IS, error: %d... %d of %d retries.\n", err, i + 1, s_num_retries );
    }
    
    if( !success )
        log_error( "packet NOT sent to APRS-IS, error: %d...\n", errno );
        
    free( (void*)packetToSend );
    wx_thread_return();
}


wx_thread_return_t sendToRadio_thread_entry( void* args )
{
    char* packetToSend = (char*)args;
    if( !packetToSend )
        wx_thread_return();
    
    // also send a packet to Direwolf running locally to hit the radio path...
    int err = sendToRadio( packetToSend );
    if( err != 0 )
        log_error( "packet failed to send via Direwolf for radio path, error: %d...\n", err );
    
    free( packetToSend );
    wx_thread_return();
}


void transmit_wx_data( const Frame* minFrame, const Frame* maxFrame, const Frame* aveFrame )
{
    char packetToSend[BUFSIZE];
    char packetFormat = UNCOMPRESSED_PACKET;

    if( s_debug )
    {
        printf( "\n" );
        printTime( false );
        printf( " Sending weather info to APRS-IS...  next update @ " );
        printTimePlus5();   // total hack and will display times such as 13:64 ?! (which is really 14:04)
        printCurrentWeather( minFrame, maxFrame, aveFrame );
    }

    APRSPacket wx;
    packetConstructor( &wx );

    uncompressedPosition( wx.latitude,    34.108,     IS_LATITUDE );
    uncompressedPosition( wx.longitude, -118.3349371, IS_LONGITUDE );

    int formatTruncationCheck = snprintf( wx.callsign, 10, "K6LOT-13" );
    assert( formatTruncationCheck >= 0 );

    formatTruncationCheck = snprintf( wx.windDirection, 4, "%03d", (int)(round(aveFrame->windDirection)) );
    assert( formatTruncationCheck >= 0 );

    formatTruncationCheck = snprintf( wx.windSpeed, 4, "%03d", (int)(round(ms2mph(aveFrame->windSpeedMs))) );
    assert( formatTruncationCheck >= 0 );

    formatTruncationCheck = snprintf( wx.gust, 4, "%03d", (int)(round(ms2mph(maxFrame->windGustMs))) );
    assert( formatTruncationCheck >= 0 );

    formatTruncationCheck = snprintf( wx.temperature, 4, "%03d", (int)(round(c2f(aveFrame->tempC))) );
    assert( formatTruncationCheck >= 0 );

    unsigned short int h = aveFrame->humidity;
    // APRS only supports values 1-100. Round 0% up to 1%.
    if( h == 0 )
        h = 1;

    // APRS requires us to encode 100% as "00".
    else if( h >= 100 )
        h = 0;

    formatTruncationCheck = snprintf( wx.humidity, 3, "%.2d", h );
    assert( formatTruncationCheck >= 0 );

    // we are converting back from InHg because that's the offset we know based on airport data! (this means we go from millibars -> InHg + offset -> millibars)
    formatTruncationCheck = snprintf( wx.pressure, 6, "%.5d", (int)(round(inHg2millibars((minFrame->pressure * millibar2inchHg) + s_localOffsetInHg) * 10)) );
    assert( formatTruncationCheck >= 0 );

    memset( packetToSend, 0, sizeof( packetToSend ) );
    printAPRSPacket( &wx, packetToSend, packetFormat, 0, false );
    // add some additional info
    strcat( packetToSend, PROGRAM_NAME );
    strcat( packetToSend, VERSION );
    if( s_debug )
        printf( "%s\n\n", packetToSend );

    // we need to create copies of the packet buffer and send that instead as we don't know the life of those other threads we light off...
    wx_create_thread_detached( sendPacket_thread_entry, copy_string( packetToSend ) );
    wx_create_thread_detached( sendToRadio_thread_entry, copy_string( packetToSend ) );
}


void transmit_air_data( const Frame* minFrame, const Frame* maxFrame, const Frame* aveFrame )
{
    char packetToSend[BUFSIZE];

    if( s_debug )
    {
        printTime( false );
        printf( " Sending air quality info to APRS-IS...\n" );
        printf( "3um: %03d, 5um: %03d, 10um: %03d, 25um: %03d, 50um: %03d\n", aveFrame->particles_03um, aveFrame->particles_05um, aveFrame->particles_10um, aveFrame->particles_25um, aveFrame->particles_50um );
    }
    
    if( s_sequence_num >= 255 )
        s_sequence_num = 0;
    
    // we need to see if we ever sent the parameters, units and equations...
    if( timeGetTimeSec() > s_lastParamsTime + kParamsInterval )
    {
        sprintf( packetToSend, "%s>APRS,TCPIP*::%s :PARM.3um,5um,10um,25um,50um", kCallSign, kCallSign );
        if( s_debug )
            printf( "%s\n", packetToSend );
        wx_create_thread_detached( sendPacket_thread_entry, copy_string( packetToSend ) );
        wx_create_thread_detached( sendToRadio_thread_entry, copy_string( packetToSend ) );

        sprintf( packetToSend, "%s>APRS,TCPIP*::%s :UNIT.um,um,um,um,um", kCallSign, kCallSign );
        if( s_debug )
            printf( "%s\n", packetToSend );
        wx_create_thread_detached( sendPacket_thread_entry, copy_string( packetToSend ) );
        wx_create_thread_detached( sendToRadio_thread_entry, copy_string( packetToSend ) );

        sprintf( packetToSend, "%s>APRS,TCPIP*::%s :EQNS.0,256,0,0,256,0,0,256,0,0,256,0,0,256,0", kCallSign, kCallSign );
        if( s_debug )
            printf( "%s\n", packetToSend );
        wx_create_thread_detached( sendPacket_thread_entry, copy_string( packetToSend ) );
        wx_create_thread_detached( sendToRadio_thread_entry, copy_string( packetToSend ) );
        
        s_lastParamsTime = timeGetTimeSec();
    }
    
    sprintf( packetToSend, "%s>APRS,TCPIP*:T#%03d,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%d%d%d%d%d%d%d%d", kCallSign, s_sequence_num++,
              aveFrame->particles_03um / 256.0,
              aveFrame->particles_05um / 256.0,
              aveFrame->particles_10um / 256.0,
              aveFrame->particles_25um / 256.0,
              aveFrame->particles_50um / 256.0,
              1,0,1,0,1,0,1,0 );

    
    // add some additional info
//    strcat( packetToSend, PROGRAM_NAME );
//    strcat( packetToSend, VERSION );
    if( s_debug )
        printf( "%s\n\n", packetToSend );

    // we need to create copies of the packet buffer and send that instead as we don't know the life of those other threads we light off...
    wx_create_thread_detached( sendPacket_thread_entry, copy_string( packetToSend ) );
    wx_create_thread_detached( sendToRadio_thread_entry, copy_string( packetToSend ) );
}



#pragma mark -


int sendToRadio( const char* p )
{
    int result = 0;
    // Parse the "TNC2 monitor format" and convert to AX.25 frame.
    unsigned char frame_data[AX25_MAX_PACKET_LEN];
    packet_t pp = ax25_from_text( (char*)p, 1 );
    if( pp != NULL )
    {
        int frame_len = ax25_pack( pp, frame_data );
        result = send_to_kiss_tnc( 0, KISS_CMD_DATA_FRAME, (char*)frame_data, frame_len );
        ax25_delete (pp);
    }
    else
    {
        log_error( "ERROR! Could not convert to AX.25 frame: %s\n", p );
        return -1;
    }
    return result;
}



/*-------------------------------------------------------------------
 *
 * Name:        send_to_kiss_tnc
 *
 * Purpose:     Encapsulate the data/command, into a KISS frame, and send to the TNC.
 *
 * Inputs:    chan    - channel number.
 *
 *        cmd    - KISS_CMD_DATA_FRAME, KISS_CMD_SET_HARDWARE, etc.
 *
 *        data    - Information for KISS frame.
 *
 *        dlen    - Number of bytes in data.
 *
 * Description:    Encapsulate as KISS frame and send to TNC.
 *
 *--------------------------------------------------------------------*/

int send_to_kiss_tnc( int chan, int cmd, char* data, int dlen )
{
    unsigned char temp[1000];
    unsigned char kissed[2000];
    int klen;
    int err = 0;

    if( chan < 0 || chan > 15 ) {
      log_error( "invalid channel %d - must be in range 0 to 15.\n", chan );
      chan = 0;
    }
    if( cmd < 0 || cmd > 15 ) {
      log_error( "invalid command %d - must be in range 0 to 15.\n", cmd );
      cmd = 0;
    }
    if( dlen < 0 || dlen > (int)(sizeof( temp ) - 1) ) {
      log_error( "invalid data length %d - must be in range 0 to %d.\n", dlen, (int)(sizeof( temp ) - 1) );
      dlen = sizeof( temp ) - 1;
    }

    temp[0] = (chan << 4) | cmd;
    memcpy( temp + 1, data, dlen );

    klen = kiss_encapsulate( temp, dlen + 1, kissed );
    
    // connect to direwolf and send data
    int server_sock = connectToDireWolf();
    if( server_sock < 0 )
    {
        log_error( "can't connect to direwolf...\n" );
        err = -1;
        goto exit_gracefully;
    }
    
    ssize_t rc = send( server_sock, (char*)kissed, klen, 0 );
    if( rc != klen )
    {
        log_error( "error writing KISS frame to socket.\n" );
        err = -1;
    }

exit_gracefully:
    shutdown( server_sock, 2 );
    close( server_sock );
    return err;
}


// returns fd to use to communicate with
int connectToDireWolf( void )
{
    int              error              = 0;
    char             foundValidServerIP = 0;
    struct addrinfo* result             = NULL;
    struct addrinfo* results;
    int              socket_desc        = -1;

    error = getaddrinfo( s_kiss_server, NULL, NULL, &results );
    if( error != 0 )
    {
        if( error == EAI_SYSTEM )
        {
            log_unix_error( "connectToDireWolf:getaddrinfo: " );
        }
        else
        {
            log_error( "error in getaddrinfo: %s\n", s_kiss_server );
        }
        return error;
    }

    for( result = results; result != NULL; result = result->ai_next )
    {
        /* For readability later: */
        struct sockaddr* const addressinfo = result->ai_addr;

        socket_desc = socket( addressinfo->sa_family, SOCK_STREAM, IPPROTO_TCP );
        if( socket_desc < 0 )
        {
            log_unix_error( "connectToDireWolf:socket: " );
            continue; /* for loop */
        }

        /* Assign the port number. */
        switch (addressinfo->sa_family)
        {
            case AF_INET:
                ((struct sockaddr_in*)addressinfo)->sin_port   = htons( s_kiss_port );
                break;
            case AF_INET6:
                ((struct sockaddr_in6*)addressinfo)->sin6_port = htons( s_kiss_port );
                break;
        }

        if( connect( socket_desc, addressinfo, result->ai_addrlen ) >= 0 )
        {
            foundValidServerIP = 1;
            break; /* for loop */
        }
        else
        {
            log_unix_error( "connectToDireWolf:connect: " );
            shutdown( socket_desc, 2 );
            close( socket_desc );
        }
    }
    freeaddrinfo( results );
    if( foundValidServerIP == 0 )
    {
        log_error( "connectToDireWolf: could not connect to the server.\n" );
        if( error )
            return error;
        else
            return -1;
    }

    return socket_desc;
}
