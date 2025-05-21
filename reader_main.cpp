#include "reader.h"
#include <iostream>



int main() {
    if (!create_ipc())   return 1;
    if (init_uart() != 0) return 2;
    readerLoop();   // writes into shm_ptr
    return 0;
}