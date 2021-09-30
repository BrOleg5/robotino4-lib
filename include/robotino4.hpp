#ifndef ROBOTINO4_HPP
#   define ROBOTINO4_HPP 

#include <iostream>
#include "rec/robotino/api2/all.h"
using namespace rec::robotino::api2;

#define PI 3.14159265358979323846

class MyCom : public Com
{
public:
	MyCom() : Com() {}

	void errorEvent( const char* errorString );
	void connectedEvent();
	void connectionClosedEvent();
	void logEvent( const char* message, int level );
	void pingEvent( float timeMs );
};

class Robotino4
{
private:
    size_t motor_num = 3;
    MyCom com;
    OmniDrive omniDrive;
    Motor motor[3];
    MotorArray motorArray;

public:
    Robotino4(const std::string& ip_addr);
    ~Robotino4();

    unsigned int get_msec();
    void sleep(unsigned int ms);
    int get_actual_position(size_t num);
    std::vector<int> get_actual_positions();
    float get_actual_velocity(size_t num);
    std::vector<float> get_actual_velocities();
    float get_actual_current(size_t num);
    std::vector<float> get_actual_currents();
    void set_motor_speed(size_t num, float speed);
    void set_motors_speed(const std::vector<float>& speeds);
    void reset_motor_position(size_t num, int pos);
    void reset_motors_position(const std::vector<int>& pos);
    void set_motor_pid(size_t num, float kp, float ki, float kd);
    void set_motors_pid(const std::vector<float>& kp, const std::vector<float>& ki, const std::vector<float>& kd);
    void set_robot_speed(float vx, float vy, float omega);

};

#endif