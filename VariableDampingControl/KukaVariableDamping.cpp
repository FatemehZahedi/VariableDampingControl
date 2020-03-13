#include <sys/time.h>
#include <iostream>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include "PositionControlClient.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"
#include <time.h>
#include <sys/shm.h>
#include <eigen3/Eigen/Dense>
#include "UdpServer.h"
#include "TrignoEmgClient.h"
#include "H5FunctionsNMCHRL.h"
#include <algorithm>
/* Boost filesystem */
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/path.hpp>
/* HDF5 */
#include "H5Cpp.h"


using namespace std;
using namespace KUKA::FRI;
using namespace Eigen;
using namespace boost::filesystem;
using namespace H5;


/* IP Address/Port for KONI Connection */
#define DEFAULT_PORTID 30200
#define DEFAULT_IP "192.170.10.2"

#define DEFAULT_GROUPNUMBER 0
#define DEFAULT_TRIALNUMBER 0


/* GUI UDP Server Address/Port */
const std::string 	udp_addr_gui("192.168.0.102");
const int 			udp_port_gui = 50000;


/* Shared Memory Function Prototype */
template<typename T>
T * InitSharedMemory(std::string shmAddr, int nElements);
void CreateOrOpenKukaDataFile(boost::filesystem::ofstream & ofs, path kukaDataFilePath);
H5File * CreateH5File(path emgDataFilePath);

enum class DampingMode{
	POSITIVE = 1,
	NEGATIVE,
	VARIABLE,
	ZERO
};

enum class TargetPosition{
	NEUTRAL = 0,
	LEFT,	// 1
	RIGHT,	// 2
	DOWN,  	// 3
	UP,		// 4
};

enum class MovementDirection{
	LEFT_RIGHT = 1,
	DOWN_UP
};

// Main
int main(int argc, char** argv)
{
	/* Setup damping modes */
	std::vector<int> groupDampingModes 	= {4,4,3,3,2,3,1,3,3,2,2,1,3,3,1,2,2,1,1};
	std::vector<int> groupDirectionSeq 	= {1,2,1,4,1,1,4,0,4,0,4,3,2,3,1,2,3,2,0};

	const int nTrialsPerGroup = 10;
	Matrix<int,5,nTrialsPerGroup> directionSequences;
	directionSequences << 1, 1, 0, 0, 0, 0, 1, 0, 1, 0,
											  0, 0, 1, 1, 1, 1, 0, 0, 1, 0,
											  0, 0, 1, 0, 1, 1, 0, 0, 1, 1,
											  1, 0, 1, 0, 1, 0, 0, 1, 1, 1,
											  1, 1, 0, 1, 0, 0, 0, 0, 1, 1;

	Matrix<int,1,nTrialsPerGroup> trialDirSeq;
	DampingMode groupDamping;

	/* Command line arguments */
	if (argc < 3){
		printf("Use: KukaVariableDamping <SubjectNumber> <GroupNumber> <optional: TrialNumber>\n");
		exit(1);
	}
	int subjectNumber 	= atoi(argv[1]);
	int groupNumber 	= atoi(argv[2]);

	/* Check Inputted Subject Number */
	if (subjectNumber < 0){
		printf("Subject Number Must Be Nonnegative\n");
		printf("Inputted Subject Number: %d\n", subjectNumber);
		exit(1);
	}
	else{
		printf("Subject: %d\n", subjectNumber);
	}

	/* Check Inputted Group Number */
	if (groupNumber >= (int) groupDampingModes.size() || groupNumber < 0){
		printf("Group Number Out of Acceptable Range\n");
		printf("Inputted Group Number: %d\n", groupNumber);
		printf("Min: 0\n");
		printf("Max: %d\n", (int) groupDampingModes.size());
		exit(1);
	}
	else{
		groupDamping = (DampingMode) groupDampingModes[groupNumber];
		printf("Group: %d\n", groupNumber);
	}

	/* Parse the rest of the command line args */
	int trialNumber;
	std::string emgIpAddr;
	const double DEFAULT_KP = 63.48;
	const double DEFAULT_KN = 32.81;
	double kp;
	double kn;
	MovementDirection moveDir = MovementDirection::LEFT_RIGHT;

	bool useEmg = false;
	bool kpInputted = false;
	bool knInputted = false;
	bool trialNumberInputted = false;

	std::string argKey;
	std::string argVal;
	for (int iArg=3; iArg<(argc-1); iArg+=2){
		/* get key and val */
		argKey = std::string(argv[iArg]);
		argVal = std::string(argv[iArg+1]);
		/* set key uppercase */
		std::transform(argKey.begin(), argKey.end(), argKey.begin(), ::toupper);

		if (argKey.compare("TN") == 0){
			trialNumberInputted = true;
			trialNumber = std::stoi(argVal);
		}
		else if (argKey.compare("EMG") == 0){
			useEmg = true;
			emgIpAddr = argVal;
		}
		else if (argKey.compare("KP") == 0){
			kpInputted = true;
			kp = std::stod(argVal);
		}
		else if (argKey.compare("KN") == 0){
			knInputted = true;
			kn = std::stod(argVal);
		}
		else if (argKey.compare("MD") == 0){
			std::transform(argVal.begin(), argVal.end(), argVal.begin(), ::toupper);
			if (argVal.compare("DU") == 0){
				moveDir = MovementDirection::DOWN_UP;
			}
		}
		else{
			printf("Key: %s not understood\n", argKey.c_str());
		}
	}

	/* Check kp and kn values */
	if (groupNumber > 1){
		if (!kpInputted || !kpInputted){
			printf("Kp AND Kn must be inputted as command line options if group number > 1\n");
			exit(1);
		}
	}
	else{
		if (!kpInputted){
			kp = DEFAULT_KP;
		}
		if (!knInputted){
			kn = DEFAULT_KN;
		}
	}

	/* Check Inputted Trial Number */
	if (!trialNumberInputted){
		trialNumber = (int) DEFAULT_TRIALNUMBER;
	}
	if (trialNumber >= nTrialsPerGroup || trialNumber < 0){
		printf("Trial Number Out of Acceptable Range\n");
		printf("Inputted Trial Number: %d\n", trialNumber);
		printf("Min: 0\n");
		printf("Max: %d\n", nTrialsPerGroup -1);
		exit(1);
	}

	/* Check EMG use */
	TrignoEmgClient emgClient;
	if (useEmg){
		emgClient.SetIpAddress(emgIpAddr);
		emgClient.ConnectDataPort();
		emgClient.ConnectCommPort();
		if (emgClient.IsCommPortConnected()){
			/* Check if sensors are paired */
			emgClient.IsSensorPaired(1);
			emgClient.IsSensorPaired(2);
			emgClient.IsSensorPaired(3);
			emgClient.IsSensorPaired(4);
			emgClient.IsSensorPaired(5);
			emgClient.IsSensorPaired(6);

			emgClient.SendCommand(1); // this command signals the emg server to send readings to Data Port
			std::thread emgReceiveThread(&TrignoEmgClient::ReceiveDataStream, &emgClient);
			emgReceiveThread.detach();
		}
	}

	/* Check Movement Direction */
	TargetPosition targetPos = TargetPosition::LEFT;
	if (moveDir == MovementDirection::LEFT_RIGHT){
			printf("Movement Direction: Left-Right\n");
	}
	else if (moveDir == MovementDirection::DOWN_UP){
			printf("Movement Direction: Down-Up\n");
	}
	else{
			printf("Movement direction had an error!");
			exit(1);
	}

	/* Define target positions */
	int trialDirSeqInd = groupDirectionSeq[groupNumber];
	trialDirSeq << directionSequences.row(trialDirSeqInd);

	// UDP Server address and port hardcoded now -- change later
	UDPServer udp_server(udp_addr_gui, udp_port_gui);

	/* Paths for data files */
	path p_base = current_path();
	// subject directory
	std::string subjectDir = "Subject" + std::to_string(subjectNumber);
	path p_subject = path(p_base.string()) /= path(subjectDir);
	create_directory(p_subject);

	// Movement direction directory
	std::string directionDir;
	if (moveDir == MovementDirection::LEFT_RIGHT){
		directionDir = "Left_Right";
	}
	else if (moveDir == MovementDirection::DOWN_UP){
		directionDir = "Down_Up";
	}
	else{
		printf("Movement Direction not set properly\n");
		exit(1);
	}
	path p_direction = path(p_subject.string()) /= path(directionDir);
	create_directory(p_direction);

	// group directory
	std::string groupDir = "Group" + std::to_string(groupNumber);
	path p_group = path(p_direction.string()) /= path(groupDir);
	create_directory(p_group);

	std::string trialDir;
	path p_trial;
	path p_kukadata;
	path p_emgdata;
	boost::filesystem::ofstream kukaDataFileStream;

	/* EMG File */
	// H5File * fileH5 = new H5File();
	// std::string emgfilename = "EmgData.h5";
	std::string emgfilename = "EmgData.txt";

	/* Kuka Data File */
	std::string kukafilename = "KukaData.txt";

	/* Measured Torque */
	double meas_torque[7];

	/* Force Related Varibles */
	double ftx;						// Force x-direction (filtered)
	double fty;						// Force y-direction (filtered)
	double ftx_un;					// Force x-direction (unfiltered)
	double fty_un;					// Force y-direction (unfiltered)
	double zerox = 0;				// Force x-baseline
	double zeroy = 0;				// Force y-baseline
	double ftx_0 = 0.0;				// Part of force filtering calc
	double fty_0 = 0.0;  			// Part of force filtering calc
	double al = 0.5;				// exponential moving average alpha level

	/* Variables related to variable dampings*/
	float dt = 0.001;
	double b_var;					// Variable damping
	double b_LB = -20;				// Lower bound of damping
	double b_UB = 60;				// Upper bound of damping
	double dampingDefaultMinorDir = 60;		// default damping value in direction that the trial is not going.
	
	if (moveDir == MovementDirection::LEFT_RIGHT){
	  b_LB = -15;
	  b_UB = 90;
	}
	else if (moveDir == MovementDirection::DOWN_UP){
	  b_LB = -20;
	  b_UB = 90;
	}
																				// ex: if moveDir == LEFT_RIGHT (along x-axis), the this is the DOWN_UP (y-axis) damping
	MatrixXd x_new_filt(6, 1); x_new_filt << 0, 0, 0, 0, 0, 0;
	MatrixXd x_new_filt_old(6, 1); x_new_filt_old << 0, 0, 0, 0, 0, 0;
	MatrixXd xdot_filt(6, 1); xdot_filt << 0, 0, 0, 0, 0, 0;
	MatrixXd xdot_filt_old(6, 1); xdot_filt_old << 0, 0, 0, 0, 0, 0;
	MatrixXd xdotdot_filt(6, 1); xdotdot_filt << 0, 0, 0, 0, 0, 0;
	MatrixXd xdotdot_filt_old(6, 1); xdot_filt_old << 0, 0, 0, 0, 0, 0;

	/* Variables related to filter of position, velocity and acceleration*/
	double CUTOFF = 20.0;
	double RC = (CUTOFF * 2 * 3.14);
	double df = 1.0 / dt;
	double alpha_filt = RC / (RC + df);

	/* Shared Memory Setup */
	std::string shmAddr("/home/justin/Desktop/FRI-Client-SDK_Cpp/example/PositionControl2/shmfile");
	int shmNumElements = 3;
	int * forceRaw = InitSharedMemory<int>(shmAddr, shmNumElements);


	/* Euler Angles */
	double phi_euler = 0;
	double theta_euler = 0;
	double psi_euler = 0;


	// ----------------------Initial DH Parameters------------------------
	MatrixXd alpha(1, 7); alpha << M_PI / 2, -M_PI / 2, -M_PI / 2, M_PI / 2, M_PI / 2, -M_PI / 2, 0;
	MatrixXd a(1, 7); a << 0, 0, 0, 0, 0, 0, 0;
	MatrixXd d(1, 7); d << 0.36, 0, 0.42, 0, 0.4, 0, 0.126;
	MatrixXd theta(1, 7); theta << 0, 0, 0, 0, 0, 0, 0;


	MatrixXd qc(6, 1); qc << 0, 0, 0, 0, 0, 0;					// calculated joint space parameter differences (between current and prev iteration)
	MatrixXd delta_q(6, 1); delta_q << 0, 0, 0, 0, 0, 0;		// (current - initial) joint space parameters
	MatrixXd q_init(6, 1); q_init << 0, 0, 0, 0, 0, 0;			// initial joint space parameter

	MatrixXd x_e(6, 1); x_e << 0, 0, 0, 0, 0, 0;				// end effector equilibrium position
	MatrixXd force(6, 1); force << 0, 0, 0, 0, 0, 0;			// force vector (filtered)
	MatrixXd q_new(6, 1); q_new << 0, 0, 0, 0, 0, 0;			// joint space parameters
	MatrixXd x_new(6, 1); x_new << 0, 0, 0, 0, 0, 0;			// calculated current position and pose
	MatrixXd xdot(6, 1);  xdot << 0, 0, 0, 0, 0, 0;				// first derative of x_new
	MatrixXd xdotdot(6, 1); xdotdot << 0, 0, 0, 0, 0, 0;		// second derativive of x_new

	MatrixXd x_old(6, 1); x_old << 0, 0, 0, 0, 0, 0;			// one iteration old position and pose
	MatrixXd x_oldold(6, 1); x_oldold << 0, 0, 0, 0, 0, 0;		// two iteration old position and pose


	/* GUI variables*/
	double gui_data[16];											// xy coordinates that are send to gui
	memset(gui_data, 0, sizeof(double) * 16);

	double radius_e = 0.005;
	double rangex_ = -0.18;
	double rangex = 0.18;
	double rangey_ = 0.58;
	double rangey = 0.94;
	double d_r = 0;
	double ex_r = (rangex * 2) / 25;
	double u_r = ex_r - radius_e;;


	/* Variables related to defining target*/
	int guiMode = 2;
	MatrixXd neutralXY(2, 1); neutralXY << 0, 0.76;
	MatrixXd endEffectorXY(2, 1); endEffectorXY << 0, 0;
	MatrixXd targetXY(2, 1); targetXY << 0, 0;
	bool withinErrorBound;
	double distFromNeutral = 0.10;  // 10 cm

	/* Variables related to defining new trial*/
	int trialSettleIterCount = 0;
	const int trialEndIterCount = 2000;
	bool trialRunning = false;
	bool targetReached = false;
	int trialWaitCounter = 0;
	int trialWaitTime = rand() % 1000 + 500;
	bool readyForTrials = false;

	/* Koni Connection */
	const char* hostname = DEFAULT_IP; //optional command line argument for ip address (default is for KONI)
	int port = DEFAULT_PORTID; //optional comand line argument for port

	int count = 0;					// iteration counter
	float sampletime = 0.001;
	double MJoint[7] = { 0 };		// measured joint position

	double MaxRadPerSec[7] = { 1.7104,1.7104,1.7453,2.2689,2.4435,3.14159,3.14159 }; //absolute max velocity (no load from KUKA manual for iiwa 800)																					 //double MaxRadPerSec[7]={1.0,1.0,1.0,1.0,1.0,1.0,1.0}; //more conservative velocity limit
	double MaxRadPerStep[7] = { 0 };	// will be calculated
	double MaxJointLimitRad[7] = { 2.9671,2.0944,2.9671,2.0944,2.9671,2.0944,3.0543 };			//Max joint limits in radians (can be changed to further restrict motion of robot)
	double MinJointLimitRad[7] = { -2.9671,-2.0944,-2.9671,-2.0944,-2.9671,-2.0944,-3.0543 }; 	//Min joint limits in radians (can be changed to further restrict motion of robot)


	//calculate max step value
	for (int i = 0; i<7; i++)
	{
		MaxRadPerStep[i] = sampletime*MaxRadPerSec[i]; //converts angular velocity to discrete time step
	}

	// create new joint position client
	PositionControlClient client;
	client.InitValues(MaxRadPerStep, MaxJointLimitRad, MinJointLimitRad);

	// create new udp connection for FRI
	UdpConnection connection;

	// pass connection and client to a new FRI client application
	ClientApplication app(connection, client);

	// connect client application to KUKA Sunrise controller
	app.connect(port, hostname);

	// Initialize stiffness, damping, and inertia matrices
	MatrixXd inertia(6, 6);
	MatrixXd stiffness(6, 6);
	MatrixXd damping(6, 6);



	// Initial Joint Angles
	client.NextJoint[0] = -1.5708;
	client.NextJoint[1] = 1.5708;
	client.NextJoint[2] = 0;
	client.NextJoint[3] = 1.5708;
	client.NextJoint[4] = 0;
	client.NextJoint[5] = -1.5708;
	client.NextJoint[6] = -0.958709;
	memcpy(client.LastJoint, client.NextJoint, 7 * sizeof(double));


	bool enough = false;

	while (!enough)
	{

		app.step();//step through program

		if (client.KukaState == 4)
		{
			count++; //count initialized at 0


			// Update measured joint angle values
			memcpy(MJoint, client.GetMeasJoint(), sizeof(double) * 7);

			// Forward Kinematic
			theta << MJoint[0], MJoint[1], MJoint[2], MJoint[3], MJoint[4], MJoint[5], MJoint[6];

			MatrixXd A1(4, 4); A1 << cos(theta(0, 0)), -sin(theta(0, 0))*cos(alpha(0, 0)), sin(theta(0, 0))*sin(alpha(0, 0)), a(0, 0)*cos(theta(0, 0)),
									 sin(theta(0, 0)), cos(theta(0, 0))*cos(alpha(0, 0)), -cos(theta(0, 0))*sin(alpha(0, 0)), a(0, 0)*sin(theta(0, 0)),
									 0, sin(alpha(0, 0)), cos(alpha(0, 0)), d(0, 0),
									 0, 0, 0, 1;
			MatrixXd A2(4, 4); A2 << cos(theta(0, 1)), -sin(theta(0, 1))*cos(alpha(0, 1)), sin(theta(0, 1))*sin(alpha(0, 1)), a(0, 1)*cos(theta(0, 1)),
									 sin(theta(0, 1)), cos(theta(0, 1))*cos(alpha(0, 1)), -cos(theta(0, 1))*sin(alpha(0, 1)), a(0, 1)*sin(theta(0, 1)),
									 0, sin(alpha(0, 1)), cos(alpha(0, 1)), d(0, 1),
									 0, 0, 0, 1;
			MatrixXd A3(4, 4); A3 << cos(theta(0, 2)), -sin(theta(0, 2))*cos(alpha(0, 2)), sin(theta(0, 2))*sin(alpha(0, 2)), a(0, 2)*cos(theta(0, 2)),
									 sin(theta(0, 2)), cos(theta(0, 2))*cos(alpha(0, 2)), -cos(theta(0, 2))*sin(alpha(0, 2)), a(0, 2)*sin(theta(0, 2)),
									 0, sin(alpha(0, 2)), cos(alpha(0, 2)), d(0, 2),
									 0, 0, 0, 1;
			MatrixXd A4(4, 4); A4 << cos(theta(0, 3)), -sin(theta(0, 3))*cos(alpha(0, 3)), sin(theta(0, 3))*sin(alpha(0, 3)), a(0, 3)*cos(theta(0, 3)),
									 sin(theta(0, 3)), cos(theta(0, 3))*cos(alpha(0, 3)), -cos(theta(0, 3))*sin(alpha(0, 3)), a(0, 3)*sin(theta(0, 3)),
									 0, sin(alpha(0, 3)), cos(alpha(0, 3)), d(0, 3),
									 0, 0, 0, 1;
			MatrixXd A5(4, 4); A5 << cos(theta(0, 4)), -sin(theta(0, 4))*cos(alpha(0, 4)), sin(theta(0, 4))*sin(alpha(0, 4)), a(0, 4)*cos(theta(0, 4)),
									 sin(theta(0, 4)), cos(theta(0, 4))*cos(alpha(0, 4)), -cos(theta(0, 4))*sin(alpha(0, 4)), a(0, 4)*sin(theta(0, 4)),
									 0, sin(alpha(0, 4)), cos(alpha(0, 4)), d(0, 4),
									 0, 0, 0, 1;
			MatrixXd A6(4, 4); A6 << cos(theta(0, 5)), -sin(theta(0, 5))*cos(alpha(0, 5)), sin(theta(0, 5))*sin(alpha(0, 5)), a(0, 5)*cos(theta(0, 5)),
									 sin(theta(0, 5)), cos(theta(0, 5))*cos(alpha(0, 5)), -cos(theta(0, 5))*sin(alpha(0, 5)), a(0, 5)*sin(theta(0, 5)),
									 0, sin(alpha(0, 5)), cos(alpha(0, 5)), d(0, 5),
									 0, 0, 0, 1;

			MatrixXd T01(4, 4); T01 << A1;
			MatrixXd T02(4, 4); T02 << T01*A2;
			MatrixXd T03(4, 4); T03 << T02*A3;
			MatrixXd T04(4, 4); T04 << T03*A4;
			MatrixXd T05(4, 4); T05 << T04*A5;
			MatrixXd T06(4, 4); T06 << T05*A6;

			// Inverse Kinematic
			phi_euler = atan2(T06(1, 2), T06(0, 2));
			theta_euler = atan2(sqrt(pow(T06(1, 2), 2) + pow(T06(0, 2), 2)), T06(2, 2));
			psi_euler = atan2(T06(2, 1), -T06(2, 0));

			MatrixXd z0(3, 1); z0 << 0, 0, 1;
			MatrixXd z1(3, 1); z1 << T01(0, 2), T01(1, 2), T01(2, 2);
			MatrixXd z2(3, 1); z2 << T02(0, 2), T02(1, 2), T02(2, 2);
			MatrixXd z3(3, 1); z3 << T03(0, 2), T03(1, 2), T03(2, 2);
			MatrixXd z4(3, 1); z4 << T04(0, 2), T04(1, 2), T04(2, 2);
			MatrixXd z5(3, 1); z5 << T05(0, 2), T05(1, 2), T05(2, 2);
			MatrixXd z6(3, 1); z6 << T06(0, 2), T06(1, 2), T06(2, 2);

			MatrixXd p0(3, 1); p0 << 0, 0, 0;
			MatrixXd p1(3, 1); p1 << T01(0, 3), T01(1, 3), T01(2, 3);
			MatrixXd p2(3, 1); p2 << T02(0, 3), T02(1, 3), T02(2, 3);
			MatrixXd p3(3, 1); p3 << T03(0, 3), T03(1, 3), T03(2, 3);
			MatrixXd p4(3, 1); p4 << T04(0, 3), T04(1, 3), T04(2, 3);
			MatrixXd p5(3, 1); p5 << T05(0, 3), T05(1, 3), T05(2, 3);
			MatrixXd p6(3, 1); p6 << T06(0, 3), T06(1, 3), T06(2, 3);

			MatrixXd J1(6, 1); J1 << z0(1, 0)*(p6(2, 0) - p0(2, 0)) - z0(2, 0)*(p6(1, 0) - p0(1, 0)),
									-z0(0, 0)*(p6(2, 0) - p0(2, 0)) + z0(2, 0)*(p6(0, 0) - p0(0, 0)),
									 z0(0, 0)*(p6(1, 0) - p0(1, 0)) - z0(1, 0)*(p6(0, 0) - p0(0, 0)),
									 z0(0, 0), z0(1, 0), z0(2, 0);
			MatrixXd J2(6, 1); J2 << z1(1, 0)*(p6(2, 0) - p1(2, 0)) - z1(2, 0)*(p6(1, 0) - p1(1, 0)),
									-z1(0, 0)*(p6(2, 0) - p1(2, 0)) + z1(2, 0)*(p6(0, 0) - p1(0, 0)),
									 z1(0, 0)*(p6(1, 0) - p1(1, 0)) - z1(1, 0)*(p6(0, 0) - p1(0, 0)),
									 z1(0, 0), z1(1, 0), z1(2, 0);
			MatrixXd J3(6, 1); J3 << z2(1, 0)*(p6(2, 0) - p2(2, 0)) - z2(2, 0)*(p6(1, 0) - p2(1, 0)),
									-z2(0, 0)*(p6(2, 0) - p2(2, 0)) + z2(2, 0)*(p6(0, 0) - p2(0, 0)),
									 z2(0, 0)*(p6(1, 0) - p2(1, 0)) - z2(1, 0)*(p6(0, 0) - p2(0, 0)),
									 z2(0, 0), z2(1, 0), z2(2, 0);
			MatrixXd J4(6, 1); J4 << z3(1, 0)*(p6(2, 0) - p3(2, 0)) - z3(2, 0)*(p6(1, 0) - p3(1, 0)),
									-z3(0, 0)*(p6(2, 0) - p3(2, 0)) + z3(2, 0)*(p6(0, 0) - p3(0, 0)),
									 z3(0, 0)*(p6(1, 0) - p3(1, 0)) - z3(1, 0)*(p6(0, 0) - p3(0, 0)),
									 z3(0, 0), z3(1, 0), z3(2, 0);
			MatrixXd J5(6, 1); J5 << z4(1, 0)*(p6(2, 0) - p4(2, 0)) - z4(2, 0)*(p6(1, 0) - p4(1, 0)),
									-z4(0, 0)*(p6(2, 0) - p4(2, 0)) + z4(2, 0)*(p6(0, 0) - p4(0, 0)),
									 z4(0, 0)*(p6(1, 0) - p4(1, 0)) - z4(1, 0)*(p6(0, 0) - p4(0, 0)),
									 z4(0, 0), z4(1, 0), z4(2, 0);
			MatrixXd J6(6, 1); J6 << z5(1, 0)*(p6(2, 0) - p5(2, 0)) - z5(2, 0)*(p6(1, 0) - p5(1, 0)),
									-z5(0, 0)*(p6(2, 0) - p5(2, 0)) + z5(2, 0)*(p6(0, 0) - p5(0, 0)),
									 z5(0, 0)*(p6(1, 0) - p5(1, 0)) - z5(1, 0)*(p6(0, 0) - p5(0, 0)),
									 z5(0, 0), z5(1, 0), z5(2, 0);


			MatrixXd Jg(6, 6); Jg << J1, J2, J3, J4, J5, J6; 	// Geometric Jacobian
			MatrixXd Tphi(6, 6); Tphi << 1, 0, 0, 0, 0, 0,
										 0, 1, 0, 0, 0, 0,
										 0, 0, 1, 0, 0, 0,
										 0, 0, 0, 0, -sin(phi_euler), cos(phi_euler)*sin(theta_euler),
										 0, 0, 0, 0, cos(phi_euler), sin(phi_euler)*sin(theta_euler),
										 0, 0, 0, 1, 0, cos(theta_euler);

			MatrixXd Ja(6, 6); Ja << Tphi.inverse()*Jg;			// Analytical Jacobian

			/* Set end effector position */
			endEffectorXY(0) = T06(0,3);
			endEffectorXY(1) = T06(2,3);

			/* For first 2 seconds, apply exponential moving average to force
			measurements, this will be the bias value */
			if (count < 2000){
				if (count == 1){	//first time inside
					sampletime = client.GetTimeStep();
					//calculate max step value
					for (int i = 0; i < 7; i++)
					{
						client.MaxRadPerStep[i] = sampletime*MaxRadPerSec[i]; //converts angular velocity to discrete time step
					}

					// Initialize equilibrium position and pose
					x_e << T06(0, 3), T06(1, 3), T06(2, 3), phi_euler, theta_euler, psi_euler;
					x_old << x_e;
					x_oldold << x_e;
					x_new << x_e;

					// Initializing Stiffness Damping and Inertia
					stiffness << 0, 0, 0, 0, 0, 0, //toward varun desk
											 0, 10000000, 0, 0, 0, 0, //up
											 0, 0, 0, 0, 0, 0, //out toward workshop
											 0, 0, 0, 1000000, 0, 0,
											 0, 0, 0, 0, 1000000, 0,
											 0, 0, 0, 0, 0, 1000000;

					damping << dampingDefaultMinorDir,   0, 										 0,   0,   0,   0,
																					0, 100, 										 0,   0,   0,   0,
																					0,   0, dampingDefaultMinorDir,   0,   0,   0,
																					0, 	 0, 										 0, 0.5,   0,   0,
																					0,   0, 										 0,   0, 0.5,   0,
																					0, 	 0, 										 0, 	0, 	 0, 0.5;

					inertia << 10, 0, 0, 0, 0, 0,
										 0, 0.000001, 0, 0, 0, 0,
										 0, 0, 10, 0, 0, 0,
										 0, 0, 0, 0.0001, 0, 0,
										 0, 0, 0, 0, 0.0001, 0,
										 0, 0, 0, 0, 0, 0.0001;
				}
				/* zero force vector, so the end effector pos does not change */
				force << 0, 0, 0, 0, 0, 0;

				/* average force readings to get a zero level */
				zerox = al*(double)forceRaw[0] / 1000000 + (1 - al)*zerox;
				zeroy = al*(double)forceRaw[1] / 1000000 + (1 - al)*zeroy;

				/* set joint positions to the default */
				q_new(0) = -1.5708;
				q_new(1) = 1.5708;
				q_new(2) = 0;
				q_new(3) = 1.5708;
				q_new(4) = 0;
				q_new(5) = -1.5708;
				q_init << q_new;
			}
			else{  // after 2 seconds

				 /* Set target placement */
	 			if (targetPos == TargetPosition::LEFT)
	 			{
	 				targetXY(0) = neutralXY(0) + distFromNeutral;
	 				targetXY(1) = neutralXY(1);
	 			}
	 			else if (targetPos == TargetPosition::RIGHT)
	 			{
	 				targetXY(0) = neutralXY(0) - distFromNeutral;
	 				targetXY(1) = neutralXY(1);
	 			}
	 			else if (targetPos == TargetPosition::NEUTRAL)
	 			{
	 				targetXY(0) = neutralXY(0);
	 				targetXY(1) = neutralXY(1);
	 			}
				else if (targetPos == TargetPosition::DOWN)
	 			{
	 				targetXY(0) = neutralXY(0);
	 				targetXY(1) = neutralXY(1) + distFromNeutral;
	 			}
				else if (targetPos == TargetPosition::UP)
				{
					targetXY(0) = neutralXY(0);
					targetXY(1) = neutralXY(1) - distFromNeutral;
				}

				withinErrorBound = (pow((endEffectorXY(0) - targetXY(0)),2) + pow((endEffectorXY(1) - targetXY(1)),2) <= pow(radius_e,2));
				/*
				 * Move to left position before trials start. Match left position, then move to neutral.
				 * At this point we are ready for trials.
				 */
				if (!readyForTrials){
					if (withinErrorBound){
						readyForTrials = true;
						targetPos = TargetPosition::NEUTRAL;
					}
				}
				else{ // ready for trials
					/* Move through trial specifics */
					if (trialRunning){		// trial running
						/* Write To Kuka Data File */
						memcpy(meas_torque, client.GetMeasTorque(), sizeof(double)*7);
						kukaDataFileStream 	<< MJoint[0] << ","
																<< MJoint[1] << ","
																<< MJoint[2] << ","
																<< MJoint[3] << ","
																<< MJoint[4] << ","
																<< MJoint[5] << ","
																<< MJoint[6] << ","
																<< force(0)  << ","
																<< force(2)  << ","
																<< x_new(0)  << ","
																<< x_new(1)  << ","
																<< x_new(2)  << ","
																<< x_new(3)  << ","
																<< x_new(4)  << ","
																<< x_new(5)  << ",";
						if (moveDir == MovementDirection::DOWN_UP){
							kukaDataFileStream << (int) targetPos + 2 << ",";
						}
						else if (moveDir == MovementDirection::LEFT_RIGHT){
							kukaDataFileStream << (int) targetPos << ",";
						}
						kukaDataFileStream  << damping(0,0) << ","
																<< xdot_filt(0) << ","
																<< xdotdot_filt(0) << ","
																<< meas_torque[0] << ","
																<< meas_torque[1] << ","
																<< meas_torque[2] << ","
																<< meas_torque[3] << ","
																<< meas_torque[4] << ","
																<< meas_torque[5] << ","
																<< meas_torque[6] << std::endl;


						/*
						 * Trials are designed as the following.  Before the trial, a target
						 * is position is set, the trial is then started.  The subject must
						 * travel to the target position, within it's error bounds. As soon as
						 * the target is reached, the trial will last for only the next two seconds.
						 * Once those two seconds are up, the trial is ended.
						 */

						/* Triggered when trial is running and target is first reached */
						if (!targetReached && withinErrorBound){
			 				targetReached = true;
							trialSettleIterCount = 0;
			 			}

						/* Keep iteration count after target is first reached */
						if (targetReached){
							trialSettleIterCount++;

							/* End Trial --- This occurs 2 secs after the target was reached */
							if (trialSettleIterCount >= trialEndIterCount){
								targetReached = false;
								trialRunning = false;
								targetPos = TargetPosition::NEUTRAL;

								/* Stop emg recording if necessary */
								if (useEmg){
									emgClient.StopWritingFileStream();
								}
							}
						}
					}
					else{					// trial not running
						/*
						 * Time between trial flows in the following manner. The subject
						 * must move within the neutral position error bounds.  Once this occurs,
						 * a random amount of time, between 0.5-1.5 seconds passes, then the trial
						 * will start.
						 */

						 /* Triggered when trial is not running and target (neutral position) is first reached */
						 if (!targetReached && withinErrorBound){
							 targetReached = true;
							 trialWaitCounter = 0;
							 trialWaitTime = rand() % 1000 + 500;
						 }

						 /* Keep iteration count after target is first reached */
						 if (targetReached){
							 trialWaitCounter++;

							 /* Start Trial --- This occurs 0.5-1.5 seconds after the neutral position is first reached */
							 if (trialWaitCounter >= trialWaitTime){
								 if (trialNumber < nTrialsPerGroup){
									targetReached = false;
									trialRunning = true;
								 	targetPos = (TargetPosition) (trialDirSeq(trialNumber) + 1);
									printf("Starting trial %d\n", trialNumber);
									/* Make trial directory */
									trialDir = std::string("Trial") + std::to_string(trialNumber);
									p_trial = path(p_group.string()) /= path(trialDir);
									create_directory(p_trial);

									/* Create kuka data trial files */
									p_kukadata = path(p_trial.string()) /= path(kukafilename);
									CreateOrOpenKukaDataFile(kukaDataFileStream, p_kukadata);

									/* Create emg hdf5 file */
									if (useEmg){
										p_emgdata = path(p_trial.string()) /= path(emgfilename);
										emgClient.StartWritingFileStream(p_emgdata);
									}
								}
								 else{
									 	enough = true;		// ends group of trials
								 }
								 trialNumber++;
							 }
						 }
					 }
				 }

				 // Get force data from shared memory, convert to Newtons
 				ftx 	= (double)forceRaw[0] / 1000000 - zerox; //toward varun desk
 				ftx_un 	= (double)forceRaw[0] / 1000000 - zerox;

 				fty 	= (double)forceRaw[1] / 1000000 - zeroy;
 				fty_un 	= (double)forceRaw[1] / 1000000 - zeroy;

 				// Filter force data with exponential moving average
 				ftx = al*ftx + (1 - al)*ftx_0;
 				ftx_0 = ftx;
 				fty = al*fty + (1 - al)*fty_0;
 				fty_0 = fty;

 				/* Copy filtered force data into force matrix */
 				force << ftx,0,fty,0,0,0;

				/* Calculate damping */
				if (groupDamping == DampingMode::POSITIVE){
						damping(0,0) = b_UB;
				}
				else if (groupDamping == DampingMode::NEGATIVE){
						damping(0,0) = b_LB;
				}
				else if (groupDamping == DampingMode::VARIABLE){
						damping(0,0) = b_var;
				}
				else if (groupDamping == DampingMode::ZERO){
						damping(0,0) = 0;
					}



				// Shift old position/pose vectors, calculate new position
				x_oldold << x_old;
				x_old << x_new;
				x_new << (inertia/(0.000001) + damping/(0.001) + stiffness).inverse()*(force + (inertia/(0.000001))*(x_old - x_oldold) + stiffness*(x_e - x_old)) + x_old;

				// Implement virtual wall - enforces boundaries on new position
			    if (x_new(2) >= 0.94){
			      x_new(2) = 0.94;
			    }

			    if (x_new(2) <= 0.58){
			      x_new(2) = 0.58;
			    }

			    if (x_new(0) >= 0.18){
			      x_new(0) = 0.18;
			    }

			    if (x_new(0) <= -0.18){
			      x_new(0) = -0.18;
			    }

				/* Calculate new joint velocities/angles */
				qc << Ja.inverse()*(x_new - x_old);
				delta_q << delta_q +qc;
				q_new << delta_q +q_init;

				/* Filter */
				x_new_filt_old 		= x_new_filt;
				x_new_filt 			= x_new_filt_old + (alpha_filt*(x_new - x_new_filt_old));

				xdot_filt_old 		= xdot_filt;
				xdot 				= (x_new_filt - x_new_filt_old) / dt;
				xdot_filt 			= xdot_filt_old + (alpha_filt*(xdot - xdot_filt_old));

				xdotdot_filt_old 	= xdotdot_filt;
				xdotdot 			= (xdot_filt - xdot_filt_old) / dt;
				xdotdot_filt 		= xdotdot_filt_old + (alpha_filt*(xdotdot - xdotdot_filt_old));

				// calculating variable damping
				if (xdot_filt(0)*xdotdot_filt(0) >= 0)					{
					b_var = (2 * b_LB / (1 + exp(-kp * xdot_filt(0)*xdotdot_filt(0))) - b_LB);
				}
				else					{
					b_var = -(2 * b_UB / (1 + exp(-kn * xdot_filt(0)*xdotdot_filt(0))) - b_UB);
				}

			} // end if (count > 2000)

			// Register new joint angles with KUKA
			client.NextJoint[0] = q_new(0);
			client.NextJoint[1] = q_new(1);
			client.NextJoint[2] = q_new(2);
			client.NextJoint[3] = q_new(3);
			client.NextJoint[4] = q_new(4);
			client.NextJoint[5] = q_new(5);
			client.NextJoint[6] = -0.958709;

			// Send data to visualizer gui
			if (moveDir == MovementDirection::LEFT_RIGHT){
				gui_data[0] = (double) guiMode;
				gui_data[1] = neutralXY(0);
				gui_data[2] = neutralXY(1);
				gui_data[3] = d_r;
				gui_data[4] = endEffectorXY(0);
				gui_data[5] = endEffectorXY(1);
				gui_data[6] = u_r;
				gui_data[7] = targetXY(0);
				gui_data[8] = targetXY(1);
				gui_data[9] = ex_r;
				gui_data[10] = damping(0, 0);
				gui_data[11] = (float) (trialNumber);
				gui_data[12] = (float) nTrialsPerGroup;

			}
			else if (moveDir == MovementDirection::DOWN_UP){
				gui_data[0] = (double) guiMode;
				gui_data[1] = neutralXY(1);
				gui_data[2] = neutralXY(0);
				gui_data[3] = d_r;
				gui_data[4] = endEffectorXY(1);
				gui_data[5] = endEffectorXY(0);
				gui_data[6] = u_r;
				gui_data[7] = targetXY(1);
				gui_data[8] = targetXY(0);
				gui_data[9] = ex_r;
				gui_data[10] = damping(0, 0);
				gui_data[11] = (float) (trialNumber);
				gui_data[12] = (float) nTrialsPerGroup;
			}

			udp_server.Send(gui_data, 16);

		}
	}

	// disconnect from controller
	app.disconnect();
	sleep(0.5);
	if (useEmg){
		/* Check if sensors are paired at the end of the trial*/
		emgClient.StopReceiveDataStream();
	}
	sleep(0.5);

	printf("Group %d trials finished\n", groupNumber);

	return 1;
}


void CreateOrOpenKukaDataFile(boost::filesystem::ofstream & ofs, path kukaDataFilePath){
	/* deconstruct kuka file path into path, filename, extension */
	path pp = kukaDataFilePath.parent_path();
	path fname_stem = kukaDataFilePath.stem();
	path fname_ext = kukaDataFilePath.extension();

	/* Make a path to rename old file with same path, and rename if necessary */
	path p_unsuc = path(kukaDataFilePath.string());
	int unsuc_count = 1;
	std::string fname_unsuc;
	if (is_regular_file(p_unsuc)){
		while (is_regular_file(p_unsuc)){
			fname_unsuc = fname_stem.string() + std::string("_unsuccessful_") + std::to_string(unsuc_count) + fname_ext.string();
			p_unsuc = path(pp.string()) /= path(fname_unsuc);
			unsuc_count++;
		}
		rename(kukaDataFilePath, p_unsuc);
	}

	/* Make file stream */
	ofs.close();
	ofs.open(kukaDataFilePath);
}

H5File * CreateH5File(path emgDataFilePath){
	/* deconstruct kuka file path into path, filename, extension */
	path pp = emgDataFilePath.parent_path();
	path fname_stem = emgDataFilePath.stem();
	path fname_ext = emgDataFilePath.extension();

	/* Make a path to rename old file with same path, and rename if necessary */
	path p_unsuc = path(emgDataFilePath.string());
	int unsuc_count = 1;
	std::string fname_unsuc;
	if (is_regular_file(p_unsuc)){
		while (is_regular_file(p_unsuc)){
			fname_unsuc = fname_stem.string() + std::string("_unsuccessful_") + std::to_string(unsuc_count) + fname_ext.string();
			p_unsuc = path(pp.string()) /= path(fname_unsuc);
			unsuc_count++;
		}
		rename(emgDataFilePath, p_unsuc);

	}

	std::string filepathStr = emgDataFilePath.string();

	H5File * file = new H5File(filepathStr.c_str(), H5F_ACC_TRUNC);
	return file;
}

// Shared Memory-------------------------------------------------------
template<typename T>
T * InitSharedMemory(std::string shmAddr, int nElements){
	key_t key;
	int shmid;
	size_t shmSize = nElements*sizeof(T);
	T * shm = (T *) malloc(shmSize);
	/* make the key */
	if ((key = ftok(shmAddr.c_str(), 'R')) == -1)
	{
		perror("ftok-->");
		exit(1);
	}

	if ((shmid = shmget(key, shmSize, 0666 | IPC_CREAT)) == -1)
	{
		perror("shmget");
		exit(1);
	}

	shm = (T *) shmat(shmid, (void *)0, 0);

	if (shm == (T *)(-1))
	{
		perror("shmat");
		exit(1);
	}

	for (int i = 0; i<nElements; i++)
	{
		shm[i] = 0.0;
	}

	return shm;
}
