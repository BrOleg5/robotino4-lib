#include "robotino4.hpp"

int main( int argc, char **argv )
{
    //Default IP addres Robotino 4
    std::string ip_addr = "192.168.0.1";
    if (argc > 1)
    {
        ip_addr = argv[1];
    }
    Robotino4 robotino(ip_addr);
    
    unsigned int test_duration = 10000;
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