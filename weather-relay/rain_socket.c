//
//  rain_socket.c
//  wx-relay
//
//  Created by Alex Lelievre on 12/21/20.
//  Copyright © 2020 Far Out Labs, LLC. All rights reserved.
//
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <sys/time.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <time.h>

#include "main.h"
#include "wx_thread.h"
#include "TXDecoderFrame.h"


#define PORT             5555
#define MAXMSG           8          // msgs from the rain sensor are usually 3 bytes long (24bit raw tip bucket count)


static sig_atomic_t s_quit = 0;
static sig_atomic_t s_raw_rain_count = 0;


int make_socket( uint16_t port )
{
    int sock;
    struct sockaddr_in name;

    /* Create the socket. */
    sock = socket( PF_INET, SOCK_STREAM, 0 );
    if( sock < 0)
    {
        perror( "socket" );
        exit( EXIT_FAILURE );       // thread exit instead !!@
    }

    /* Give the socket a name. */
    name.sin_family = AF_INET;
    name.sin_port = htons (port);
    name.sin_addr.s_addr = htonl (INADDR_ANY);
    if( bind( sock, (struct sockaddr *)&name, sizeof( name ) ) < 0 )
    {
        perror( "bind" );
        exit( EXIT_FAILURE );       // thread exit instead !!@
    }

    return sock;
}


int read_from_client( int filedes )
{
    RainFrame frame;
    ssize_t   nbytes = 0;

    memset( &frame, 0, sizeof( RainFrame ) );
    nbytes = read( filedes, &frame, sizeof( RainFrame ) );
    if( nbytes < 0 )
    {
        perror( "read" );
        return (int)nbytes;
    }
    else if( nbytes == 0 )
    {
        // End-of-file
        return -1;
    }

    s_raw_rain_count = frame.raw_rain_count;
    
#ifdef DEBUG
    fprintf( stderr, "rain_socket: raw rain message: %d\n\n", frame.raw_rain_count );
#endif
    
    return 0;
}


 
wx_thread_return_t rain_socket_thread( void* args )
{
    int       sock;
    fd_set    active_fd_set;
    fd_set    read_fd_set;
    socklen_t size = 0;
    struct sockaddr_in clientname = {};
    struct timeval timeout = { .tv_sec = 3, .tv_usec = 0 };
    
    // Create the socket and set it up to accept connections.
    sock = make_socket( PORT );
    if( listen( sock, 1 ) < 0 )
    {
        log_error( "rain_socket failed to listen: %d\n", errno );
        perror( "rain_socket listen" );
        exit( EXIT_FAILURE );
    }

    log_error( "rain_socket listening on port %u\n", PORT );

    // Initialize the set of active sockets
    FD_ZERO( &active_fd_set );
    FD_SET( sock, &active_fd_set );

    while( !s_quit )
    {
        // Block until input arrives on one or more active sockets
        read_fd_set = active_fd_set;
        if( select( FD_SETSIZE, &read_fd_set, NULL, NULL, &timeout ) < 0 )
        {
            log_error( "rain_socket failed select(): %d\n", errno );
            perror ("select");
            continue;
        }

        // Service all the sockets with input pending.
        for( int i = 0; i < FD_SETSIZE; ++i )
        {
            if( FD_ISSET( i, &read_fd_set ) )
            {
                if( i == sock )
                {
                    // Connection request on original socket
                    size = sizeof( clientname );
                    int new = accept( sock, (struct sockaddr *)&clientname, &size );
                    if( new < 0 )
                    {
                        log_error( "rain_socket failed accept(): %d\n", errno );
                        perror( "accept" );
                    }
                    else
                    {
#ifdef DEBUG
                        fprintf( stderr, "\nrain_socket: connection from %s, port %hu.\n", inet_ntoa( clientname.sin_addr ), ntohs( clientname.sin_port ) );
#endif
                    }
                    FD_SET( new, &active_fd_set );
                }
                else
                {
                    // Data arriving on an already-connected socket
                    if( read_from_client( i ) < 0 )
                    {
                        close( i );
                        FD_CLR( i, &active_fd_set );
                    }
                }
            }
        }
    }
    wx_thread_return();
}


void rain_socket_thread_quit( void )
{
    s_quit = 1;
}


int rain_socket_raw_count()
{
    return s_raw_rain_count;
}
