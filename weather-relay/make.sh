gcc -g main.c wx_thread.c rain_socket.c rain_sensor.c ../stubs.c ../ax25_pad.c ../kiss_frame.c ../fcs_calc.c ../aprs-weather-submit/src/aprs-is.c ../aprs-weather-submit/src/aprs-wx.c -D__insecure_redirect__ -DKISSUTIL -I. -I.. -I../../tx31u-receiver/ -I../aprs-weather-submit/src/ -lm -o wxrelay -pthread

