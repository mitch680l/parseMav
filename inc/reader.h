// reader.h
#ifndef READER_H
#define READER_H

#include <cstdint>
#include "mavlink_usart.h"



bool create_ipc();   
bool open_ipc();            
int  init_uart();          
void readerLoop();         


void getTelem(double* lat, double* lon, double* alt);
void getBearing(double* bearing);
void setCurrent(int current);

#endif 