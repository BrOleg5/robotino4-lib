#include "robotino4.hpp"

//This function is called on errors.
void MyCom::errorEvent( const char* errorString )
{
    std::cerr << "Error: " << errorString << std::endl;
}

//This function is called if a connection to Robotino has been established.
void MyCom::connectedEvent()
{
    std::cout << "Connected." << std::endl;
}

//This function is called when a connection is closed.
void MyCom::connectionClosedEvent()
{
    std::cout << "Connection closed." << std::endl;
}

//This function is called when a log message is posted.
void MyCom::logEvent( const char* message, int level )
{
    std::cout << message << std::endl;
}

//This function is called when use ping to robot.
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

//Return milliseconds since connection to server had ben established.
unsigned int Robotino4::get_msec()
{
    return com.msecsElapsed();
}

//Pause the programm for ms milliseconds.
void Robotino4::sleep(unsigned int ms)
{
    rec::robotino::api2::msleep(ms);
}

/*Retrieves the actual position of this motor.
num - motor number.
Position in encoder ticks.*/
int Robotino4::get_actual_position(size_t num){
    return motor[num].actualPosition();
}

/*Retrieves the actual position of all motors.
Position in encoder ticks.*/
std::vector<int> Robotino4::get_actual_positions()
{
    std::vector<int> pos = {0, 0, 0};
    motorArray.actualPositions(&pos[0]);
    return pos;
}

/*Retrieves the actual velocity of this motor.
num - motor number.
Velocity in rad/s.*/
float Robotino4::get_actual_velocity(size_t num)
{
    return 2 * PI * motor[num].actualVelocity() / 60;
}

/*Retrieves the actual velocity of all motors.
Velocity in rad/s.*/
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

/*Retrieves the current of this motor.
num - motor number.
Current in A.*/
float Robotino4::get_actual_current(size_t num)
{
    return motor[num].motorCurrent();
}

/*Retrieves the current of all motors.
Current in A.*/
std::vector<float> Robotino4::get_actual_currents()
{
    std::vector<float> cur = {0, 0, 0};
    motorArray.actualVelocities(&cur[0]);
    return cur;
}

/*Sets the setpoint speed of this motor.
num - motor number.
speed - setpoint speed in rad/s.*/
void Robotino4::set_motor_speed(size_t num, float speed)
{
    motor[num].setSpeedSetPoint(60 * speed / (2 * PI));
}

/*Sets the setpoint speed of all motors.
speeds - setpoint speed for all motors in rad/s.*/
void Robotino4::set_motors_speed(const std::vector<float>& speeds)
{
    motorArray.setSpeedSetPoints(&speeds[0], motor_num);
}

/*Resets the position of this motor.
num - motor number.
pos - new position after reset in encoder ticks.*/
void Robotino4::reset_motor_position(size_t num, int pos)
{
    motor[num].resetPosition(pos);
}

/*Resets the position of all motors.
pos - new position after reset in encoder ticks.*/
void Robotino4::reset_motors_position(const std::vector<int>& pos)
{
    for (size_t i = 0; i < motor_num; i++)
    {
        motor[i].resetPosition(pos[i]);
    }
    
}

/*Sets the proportional, integral and differential constant of the PID controller. 
num - motor number.
kp - proportional constant of the motor's speed PID controller.
ki - integral constant of the motor's speed PID controller.
kd - differential constant of the motor's speed PID controller.
These values are scaled by the microcontroller firmware to match with the PID controller implementation.
If value is given, the microcontroller firmware uses its build in default value.
Robotino v3 Parameters are floating point values used by the microcontroller directly. If parameter is less than 0 the default parameter is used.*/
void Robotino4::set_motor_pid(size_t num, float kp, float ki, float kd)
{
    motor[num].setPID(kp, ki, kd);
}

/*Sets the proportional, integral and differential constant of the PID controller. 
kp - proportional constants of the motor's speed PID controllers.
ki - integral constants of the motor's speed PID controllers.
kd - differential constants of the motor's speed PID controllers.
These values are scaled by the microcontroller firmware to match with the PID controller implementation.
If value is given, the microcontroller firmware uses its build in default value.
Robotino v3 Parameters are floating point values used by the microcontroller directly. If parameter is less than 0 the default parameter is used.*/
void Robotino4::set_motors_pid(const std::vector<float>& kp, const std::vector<float>& ki, const std::vector<float>& kd)
{
    for (size_t i = 0; i < motor_num; i++)
    {
        motor[i].setPID(kp[i], ki[i], kd[i]);
    }
    
}

/*Set robot speed.
vx - speed along x axis of robot's local coordinate system in m/s.
vy - speed along y axis of robot's local coordinate system in m/s.
omega - angular velocity of rotation in rad/s.
This function is thread save. It should be called about every 100ms.*/
void Robotino4::set_robot_speed(float vx, float vy, float omega)
{
    omniDrive.setVelocity(vx, vy, omega);
}