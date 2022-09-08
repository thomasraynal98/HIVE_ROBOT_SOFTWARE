#include <string.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <math.h>
#include <fstream>
#include <cstdlib>
#include <unistd.h>

#include <sio_client.h>

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

#define ASIO_STANDALONE 

LibSerial::SerialPort* com_manager = new LibSerial::SerialPort;

std::thread thread_w, thread_r;

void f_write()
{
    double ms_for_loop = 50;
    auto next = std::chrono::high_resolution_clock::now();

    com_manager->Open("/dev/ttyUSB0");
    com_manager->SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    // com_manager->Set

    while(true)
    {
        next += std::chrono::milliseconds((int)ms_for_loop);
        std::this_thread::sleep_until(next);

        com_manager->Write("M|0|0.0|0.0|");
    }
}

void f_read()
{

}

int main(int argc, char *argv[])
{
    sio::client h;
    h.set_logs_verbose();
    h.connect("https://api.hiverobotics.fr/");
    // thread_w    = std::thread(&f_write);
    // thread_r    = std::thread(&f_read);

    // thread_w.join();
    // thread_r.join();
}