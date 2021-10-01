#include "robotino4.hpp"

void MyCom::errorEvent( const char* errorString )
{
    std::cerr << "Error: " << errorString << std::endl;
}

void MyCom::connectedEvent()
{
    std::cout << "Connected." << std::endl;
}

void MyCom::connectionClosedEvent()
{
    std::cout << "Connection closed." << std::endl;
}

void MyCom::logEvent( const char* message, int level )
{
    std::cout << message << std::endl;
}

void MyCom::pingEvent( float timeMs )
{
    std::cout << "Ping: " << timeMs << "ms" << std::endl;
}



Robotino4::Robotino4(const std::string& ip_addr)
{
    omniDrive.setComId(com.id());
    motorArray.setComId(com.id());
    for (size_t i = 0; i < motor_num; i++)
    {
        motor[i].setComId(com.id());
        motor[i].setMotorNumber(i);
    }

	// Connect
    std::cout << "Connecting...";
	com.setAddress( ip_addr.c_str() );
	com.connectToServer( true );
	if( !com.isConnected() )
	{
		std::cout << std::endl << "Could not connect to " << com.address() << std::endl;
		rec::robotino::api2::shutdown();
		exit( 1 );
	}
	else
	{
		std::cout << "success" << std::endl;
	}
}

Robotino4::~Robotino4()
{
    com.disconnectFromServer();
    rec::robotino::api2::shutdown();
}

unsigned int Robotino4::get_msec()
{
    return com.msecsElapsed();
}

void Robotino4::sleep(unsigned int ms)
{
    rec::robotino::api2::msleep(ms);
}

int Robotino4::get_actual_position(size_t num){
    return motor[num].actualPosition();
}

std::vector<int> Robotino4::get_actual_positions()
{
    std::vector<int> pos = {0, 0, 0};
    motorArray.actualPositions(&pos[0]);
    return pos;
}

float Robotino4::get_actual_velocity(size_t num)
{
    return 2 * PI * motor[num].actualVelocity() / 60;
}

std::vector<float> Robotino4::get_actual_velocities()
{
    std::vector<float> vel = {0, 0, 0};
    motorArray.actualVelocities(&vel[0]);
    for (size_t i = 0; i < motor_num; i++)
    {
        vel[i] = 2 * PI * vel[i] / 60;
    }
    return vel;
}

float Robotino4::get_actual_current(size_t num)
{
    return motor[num].motorCurrent();
}

std::vector<float> Robotino4::get_actual_currents()
{
    std::vector<float> cur = {0, 0, 0};
    motorArray.actualVelocities(&cur[0]);
    return cur;
}

void Robotino4::set_motor_speed(size_t num, float speed)
{
    motor[num].setSpeedSetPoint(60 * speed / (2 * PI));
}

void Robotino4::set_motors_speed(const std::vector<float>& speeds)
{
    motorArray.setSpeedSetPoints(&speeds[0], motor_num);
}

void Robotino4::reset_motor_position(size_t num, int pos)
{
    motor[num].resetPosition(pos);
}

void Robotino4::reset_motors_position(const std::vector<int>& pos)
{
    for (size_t i = 0; i < motor_num; i++)
    {
        motor[i].resetPosition(pos[i]);
    }
    
}

void Robotino4::set_motor_pid(size_t num, float kp, float ki, float kd)
{
    motor[num].setPID(kp, ki, kd);
}

void Robotino4::set_motors_pid(const std::vector<float>& kp, const std::vector<float>& ki, const std::vector<float>& kd)
{
    for (size_t i = 0; i < motor_num; i++)
    {
        motor[i].setPID(kp[i], ki[i], kd[i]);
    }
    
}

void Robotino4::set_robot_speed(float vx, float vy, float omega)
{
    omniDrive.setVelocity(vx, vy, omega);
}