
#define _POSIX_C_SOURCE 199309L // 타이머 관련 기능 사용을 위해 필요한 헤더

#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>

#include "b3RobotSimulatorClientAPI.h"
#include "../Utils/b3Clock.h"
#include "LinearMath/btVector3.h"
#include "btBulletDynamicsCommon.h"
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <stdio.h>
#include <assert.h>
#include <thread>

#include "Robot.h"
#include "LR_Control.h"

#define ASSERT_EQ(a, b) assert((a) == (b));
double CONTROL_RATE = 1000.0;
btScalar fixedTimeStep = 1. / CONTROL_RATE;
Robot *robot;
LR_Control *control;
double gt;
double dt = fixedTimeStep;
b3RobotSimulatorClientAPI* sim;
JVec q;
JVec q_dot;
JVec kMaxTorques=JVec::Ones()*1000;
JVec torques;
Vector3d eef_forces;
Vector3d eef_moments;

void physics_callback(union sigval sv){
	q = robot->get_q();
	q_dot = robot->get_q_dot();
	//JVec torques = lr::GravityForces(q,control->g,control->Mlist, control->Glist, control->Slist);
	torques = robot->calc_inverse_dynamics();
	robot->set_torques(torques,kMaxTorques);
	//eef_forces = robot->get_eef_forces();
	//eef_moments = robot->get_eef_moments();
	//robot->apply_ext_forces(Vector3d(10,10,10));
	
	gt+=dt;
	sim->stepSimulation();
}
int main(int argc, char* argv[])
{
	//--------------arg Setup-----------------------------
    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <urdf_file_path>" <<  " <with_gui>" << std::endl;
        return 1;
    }
    const char* urdfFilePath = argv[1]; // 첫 번째 인자를 URDF 파일 경로로 사용
	bool with_gui = atoi(argv[2]) != 0;
	//-----------------Sim Setup------------------------
	sim = new b3RobotSimulatorClientAPI();
	bool isConnected;
	if (with_gui==true)
		isConnected = sim->connect(eCONNECT_GUI);
	else
		isConnected = sim->connect(eCONNECT_DIRECT);

	if (!isConnected)
	{
		printf("Cannot connect\n");
		return -1;
	}
	sim->configureDebugVisualizer(COV_ENABLE_GUI, 0);
	sim->setTimeOut(10);
	sim->syncBodies();	
	sim->setTimeStep(fixedTimeStep);
	sim->setGravity(btVector3(0, 0, -9.8));
	b3RobotSimulatorSetPhysicsEngineParameters args;
	sim->getPhysicsEngineParameters(args);
	int robotId = sim->loadURDF(urdfFilePath);
	int planeId = sim->loadURDF("urdf/plane/plane.urdf");
	sim->setRealTimeSimulation(false);
	robot = new Robot(sim,robotId);	
	control = new LR_Control();
	control->LRSetup();
	gt =0;
	//-----------------Thread Setup------------------------
    timer_t timerid;
    struct sigevent sev;
    struct itimerspec its;
    // 타이머 설정을 초기화
    sev.sigev_notify = SIGEV_THREAD;   // 타이머 이벤트를 스레드로 처리
    sev.sigev_notify_function = physics_callback; // 타이머 콜백 함수 설정
    // 타이머 생성
    if (timer_create(CLOCK_REALTIME, &sev, &timerid) == -1) {
        perror("timer_create");
        return 1;
    }
    // 타이머 주기 설정 (1ms)
    its.it_value.tv_sec = 0;            // 초기 실행까지 대기할 초
    its.it_value.tv_nsec = 1000000;     // 초기 실행까지 대기할 나노초 (1ms)
    its.it_interval.tv_sec = 0;         // 주기적 실행 주기 (1ms)
    its.it_interval.tv_nsec = 1000000;  // 주기적 실행 주기의 나노초 (1ms)
    // 타이머 설정을 적용
    if (timer_settime(timerid, 0, &its, NULL) == -1) {
        perror("timer_settime");
        return 1;
    }


	//Print
	while(1){
		printf("gt : %.3f\n",gt);
		std::cout<<"q : "<<q.transpose()<<std::endl;
		std::cout<<"q_dot : "<<q_dot.transpose()<<std::endl;
		std::cout<<"eef_forces : "<<eef_forces.transpose()<<std::endl;
		std::cout<<"torques : "<<torques.transpose()<<std::endl;
		sleep(1);  // 1초 대기
	}
	printf("EXIT\n");
	delete sim;
}

