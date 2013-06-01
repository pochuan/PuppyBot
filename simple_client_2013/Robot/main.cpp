
#include <iostream>
#include "RobotCom.h"
#include "PrVector.h"

int main(int argc, char** argv)
{
	std::cout<<"This program tests the network connectivity between the client simulator and the servo server"<<std::endl;

/****************************************/
//TEST 1: ESTABLISH CONNECTION
/****************************************/
	RobotCom *test_robot = new RobotCom();
	
	std::cout<<"Sucessfully connected!"<<std::endl;

/****************************************/
//TEST 2: SEND MESSAGE TO THE SERVO SERVER
/****************************************/
	
	float helloWorld[] = {0.1, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	test_robot->control( JTRACK, helloWorld, 8);
	

/****************************************/
//TEST 3: RECEIVE REPLIES FROM THE SERVO SERVER
/****************************************/
	float data_in[8];
	test_robot->getStatus(GET_JPOS, data_in);
	std::cout<<"Joint angles are "<<data_in[0]<<" "<<data_in[1]<<" "<<data_in[2]<<" "<<data_in[3]<<" "<<data_in[4]<<" "<<data_in[5]<<" "<<data_in[6]<<" "<<data_in[7]<<std::endl;

/****************************************/
//TEST 4: TEAR DOWN
/****************************************/

	delete test_robot;

	std::cout<<"Successfully disconnected!"<<std::endl;	

}
