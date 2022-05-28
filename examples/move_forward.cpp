#include "robotino4.hpp"
#include "inputparser.hpp"
#include <iostream>
#include <iomanip>

int main( int argc, char **argv )
{
    // Default IP addres Robotino 4
    std::string ip_addr = "192.168.0.1";

    unsigned int test_duration = 10000;

    // Process command line options
    if(argc > 1) {
        InputParser input(argc, argv);
        std::streamsize st_size = 25;
        // Help option
        if(input.cmdOptionExists("-h") || input.cmdOptionExists("-?") || input.cmdOptionExists("--help")) {
            std::cout << "usage: move_forward [options]\n\nOptions:\n"
                        << std::left << "  " << std::setw(st_size) << "-h/-?/--help" << "Display this information.\n"
                        << "  " << std::setw(st_size) << "-ip <address>" << "Set Robotino4 IP-<address>.\n"
                        << "  " << std::setw(st_size) << "-t <time>" << "Set duration of execution. <time> in ms\n";
                exit(0);
        }
        if(input.cmdOptionExists("-ip"))
        {
            ip_addr = input.getCmdOption("-ip");
        }
        if(input.cmdOptionExists("-t")) {
            test_duration = std::stoi(input.getCmdOption("-t"));
        }
    }
    Robotino4 robotino(ip_addr);
    
    unsigned int time = robotino.get_msec();
    std::cout << "| Time | M1 pos | M2 pos | M3 pos | M1 vel | M2 vel | M3 vel| M1 cur | M2 cur | M3 cur |" << std::endl;
    std::cout << "|--------------------------------------------------------------------------------------|" << std::endl;
    while (time <= test_duration)
    {
        robotino.set_robot_speed(0.1f, 0.0f, 0.0f);
        std::cout << " | " << robotino.get_msec() << " | " << robotino.get_actual_position(0) << " | " << robotino.get_actual_position(1) << " | ";
        std::cout << robotino.get_actual_position(2) << " | " << robotino.get_actual_velocity(0) << " | " << robotino.get_actual_velocity(1) << " | ";
        std::cout << robotino.get_actual_velocity(2) << " | " << robotino.get_actual_current(0) << " | " << robotino.get_actual_current(1) << " | ";
        std::cout << robotino.get_actual_current(2) << " | " << std::endl;
        robotino.sleep(20);
        time = robotino.get_msec();
    }
    return 0;
}