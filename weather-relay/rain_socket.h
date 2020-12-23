//
//  rain_socket.h
//  weather-relay
//
//  Created by Alex Lelievre on 7/20/20.
//  Copyright © 2020 Far Out Labs. All rights reserved.
//

#ifndef _H_rain_socket
#define _H_rain_socket

#include <stdbool.h>
#include <stdint.h>

wx_thread_return_t rain_socket_thread( void* args );
void               rain_socket_thread_quit( void );
int                rain_socket_raw_count();



#endif // !_H_rain_socket
