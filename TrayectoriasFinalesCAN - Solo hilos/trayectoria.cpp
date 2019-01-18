#include <iostream>
#include "Definitions.h"
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdio.h>
#include <list>
#include <math.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/times.h>
#include <sys/time.h>
#include <time.h>
#include <ctime>
#include <pthread.h>
#include <fstream>
#include <thread>

using namespace std;

//====================================================
//============ Configure EPOS4 parameters ============
//====================================================

void* g_pKeyHandle = 0;
string g_deviceName = "EPOS4";

string g_protocolStackName = "CANopen";
string g_interfaceName = "CAN_mcp251x 0";
string g_portName = "CAN0";
unsigned int p_pErrorCode = 0;
bool Absolute = true;
bool Immediately = true;


//====================================================
//======== Declaracion de variables iniciales ========
//====================================================

unsigned short g_usNodeId = 1;	// ID Gateway (Hip)
unsigned short g_usNodeId2 = 2; // ID CAN node 2 (Knee)
unsigned short g_usNodeId3 = 3; // ID CAN node 3 (Ankle)



long TargetPosition[] = {30, -15, 35, 30};           // Position Data (grades hip)
long TargetPosition2[] = {-3, -13, -3, -65, -3};	 // Position Data (grades knee)
long TargetPosition3[] = {4, 18, -8, 24, 2, 8, 4};	 // Position Data (grades ankle)


double CyclePercent[] = {0, 0.52, 0.87, 1};                      // Percent of hike hip
double CyclePercent2[] = {0, 0.18, 0.4, 0.69, 1};                // Percent of hike knee
double CyclePercent3[] = {0, 0.08, 0.38, 0.67, 0.79, 0.92, 1};   // Percent of hike ankle

int HikingTime = 10;        // Hiking Time


// Initial condition for MCUV

double Vf = 0;
double Vf2 = 0;
double Vf3 = 0;
double Tf = 0;
double Tf2 = 0;
double Tf3 = 0;

// Position measurements
int pPositionIs = 0;
int pPositionIs2 = 0;
int pPositionIs3 = 0;

short pCurrentIs = 0;
short pCurrentIs2 = 0;
short pCurrentIs3 = 0;

int pVelocityIs = 0;
int pVelocityIs2 = 0;
int pVelocityIs3 = 0;

int pDemandIs = 0;
int pDemandIs2 = 0;
int pDemandIs3 = 0;

unsigned int pNbBytes = 0;



// To indicate the number of samles of each joint
int contador = 0;
int contador2 = 0;
int contador3 = 0;

// Help to start the data capture of each joint
int banCaptura = 0;



// Empty data of hip
unsigned int ProfileVelocity[3];
unsigned int ProfileAcceleration[3];
unsigned int ProfileDeceleration[3];

// Empty data of knee
unsigned int ProfileVelocity2[4];
unsigned int ProfileAcceleration2[4];
unsigned int ProfileDeceleration2[4];

// Empty data of ankle
unsigned int ProfileVelocity3[6];
unsigned int ProfileAcceleration3[6];
unsigned int ProfileDeceleration3[6];

const string g_programName = "Left Lower Limb";    // Indicates which limb has an error

//====================================================
//===================== Functions ====================
//====================================================

int   OpenDevice();
void  LogError(string functionName, unsigned int p_ulErrorCode);

//====================================================
//================ Detect EPOS4 error ================
//====================================================

void LogError(string functionName, unsigned int p_ulErrorCode)
{
    // When EPOS4 has an error, print the code error
	cerr << g_programName << ": " << functionName << " failed errorCode=0x" << std::hex << p_ulErrorCode << ")"<< endl;
}

//====================================================
//=================== Enable Device ==================
//====================================================

int OpenDevice()
{

    char* pDeviceName = new char[255];
    char* pProtocolStackName = new char[255];
    char* pInterfaceName = new char[255];
    char* pPortName = new char[255];


    strcpy(pDeviceName, g_deviceName.c_str());
    strcpy(pProtocolStackName, g_protocolStackName.c_str());
    strcpy(pInterfaceName, g_interfaceName.c_str());
    strcpy(pPortName, g_portName.c_str());

	g_pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, &p_pErrorCode);   // Open the port to comunicate with EPOS4

	// If the port open correctly, enable devices
	if (g_pKeyHandle > 0)
	{
        VCS_SetEnableState(g_pKeyHandle, g_usNodeId, &p_pErrorCode);     // Enable EPOS4 Hip
        VCS_SetEnableState(g_pKeyHandle, g_usNodeId2, &p_pErrorCode);    // Enable EPOS4 Knee
        //VCS_SetEnableState(g_pKeyHandle, g_usNodeId3, &p_pErrorCode);    // Enable EPOS4 Ankle
	}
	// Report the error
	else{
        cout << "error" << endl;
        LogError("VCS_OpenDevice", p_pErrorCode);
	}
	return 0;

}

//====================================================
//=========== Data capture - Hip (thread) ============
//====================================================

void* Data(void* data){

cout << "Capturando datos" << endl;

ofstream fa1("ActualValueHip.txt");             // Created a txt file to save hip data
ofstream fd1("DemandValueHip.txt");
ofstream fc1("ActualCurrentHip.txt");
ofstream fv1("ActualVelocityHip.txt");

ofstream fa2("ActualValueKnee.txt");             // Created a txt file to save hip data
ofstream fd2("DemandValueKnee.txt");
ofstream fc2("ActualCurrentKnee.txt");
ofstream fv2("ActualVelocityKnee.txt");

// Capturing hip data until finish one hiking
while (banCaptura == 0)
{
    usleep(2000);
    VCS_GetPositionIs(g_pKeyHandle, g_usNodeId, &pPositionIs, &p_pErrorCode);   // Get position of hip joint
    VCS_GetObject(g_pKeyHandle, g_usNodeId, 0x6062, 0x00, &pDemandIs, 4, &pNbBytes, &p_pErrorCode); // Get refence of hip joint
    VCS_GetCurrentIs(g_pKeyHandle, g_usNodeId, &pCurrentIs, & p_pErrorCode);
    VCS_GetVelocityIs(g_pKeyHandle, g_usNodeId, &pVelocityIs, &p_pErrorCode);



    VCS_GetPositionIs(g_pKeyHandle, g_usNodeId2, &pPositionIs2, &p_pErrorCode);   // Get position of hip joint
    VCS_GetObject(g_pKeyHandle, g_usNodeId2, 0x6062, 0x00, &pDemandIs2, 4, &pNbBytes, &p_pErrorCode); // Get refence of hip joint
    VCS_GetCurrentIs(g_pKeyHandle, g_usNodeId2, & pCurrentIs2, & p_pErrorCode);
    VCS_GetVelocityIs(g_pKeyHandle, g_usNodeId2, &pVelocityIs2, &p_pErrorCode);


    fa1 << float(pPositionIs)/2230 << endl;              // Save position of hip in txt file
    fd1 << dec << float(pDemandIs)/2230 << endl;
    fc1 << abs(float(pCurrentIs)) << endl;
    fv1 << abs(float(pVelocityIs)) << endl;



    fa2 << abs(float(pPositionIs2)/487) << endl;              // Save position of hip in txt file
    fd2 << dec << abs(float(pDemandIs2)/487) << endl;
    fc2 << abs(float(pCurrentIs2)) << endl;
    fv2 << abs(float(pVelocityIs2)) << endl;

    contador = contador + 1;
}

fa1.close();
fd1.close();
fc1.close();
fv1.close();

fa2.close();
fd2.close();
fc2.close();
fv2.close();
                              // Close txt file
cout << "muestras 1: " << contador << endl;     // print the number of samples
contador = -1;

}


//====================================================
//=============== Hip movement (thread) ==============
//====================================================

void* Cadera(void* data) {

cout << "Cadera" << endl;

for (int x = 1; x <= 3; x++){
    //auto start = std::chrono::high_resolution_clock::now();


    VCS_SetPositionProfile(g_pKeyHandle, g_usNodeId, ProfileVelocity[x-1], ProfileAcceleration[x-1], ProfileDeceleration[x-1], &p_pErrorCode);  // Load parameteres of each stage
    // If EPOS4 fail in his movement, report that error
    if (VCS_MoveToPosition(g_pKeyHandle, g_usNodeId, TargetPosition[x], Absolute, Immediately, &p_pErrorCode) == 0)
    {
        LogError("VCS_MoveToPosition - Hip", p_pErrorCode);     // Report the error
        break;

    }


    cout << "PASO Cadera" << endl;
    usleep((HikingTime * (CyclePercent[x] - CyclePercent[x - 1]))*1000000); // wait until finish each stage

    /*
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end-start;
    std::cout << "Waited " << elapsed.count() << " ms\n";
    */

    //cout << (HikingTime * (CyclePercent[x] - CyclePercent[x - 1]))*1000000 << endl;
}

banCaptura = 1;  // Indicate that the capture of information must finish
return NULL;
}


//====================================================
//============== Knee movement (thread) ==============
//====================================================

void* Rodilla(void* data) {

cout << "Rodilla" << endl;

for (int x = 1; x <= 4; x++){

    VCS_SetPositionProfile(g_pKeyHandle, g_usNodeId2, ProfileVelocity2[x-1], ProfileAcceleration2[x-1], ProfileDeceleration2[x-1], &p_pErrorCode);  // Load parameteres of each stage

    // If EPOS4 fail in his movement, report that error
    if (VCS_MoveToPosition(g_pKeyHandle, g_usNodeId2, TargetPosition2[x], Absolute, Immediately, &p_pErrorCode) == 0)
    {
        LogError("VCS_MoveToPosition - Knee", p_pErrorCode);    // Report the error
        break;
    }

    cout << "PASO Rodilla" << endl;
    usleep((HikingTime * (CyclePercent2[x] - CyclePercent2[x - 1]))*1000000);

}

//banRodilla = 1;     // Indicate that the capture of information must finish
return NULL;
}

//====================================================
//============== ankle movement (thread) =============
//====================================================

void* Tobillo(void* data) {
// Otro proceso
cout << "Tobillo" << endl;

// MOvimiento del tobillo
for (int x = 1; x <= 6; x++){

    // ankle
    VCS_SetPositionProfile(g_pKeyHandle, g_usNodeId3, ProfileVelocity3[x-1], ProfileAcceleration3[x-1], ProfileDeceleration3[x-1], &p_pErrorCode);  // Load parameteres of each stage

    // If EPOS4 fail in his movement, report that error
    if (VCS_MoveToPosition(g_pKeyHandle, g_usNodeId3, TargetPosition3[x], Absolute, Immediately, &p_pErrorCode) == 0)
    {
        LogError("VCS_MoveToPosition - Ankle", p_pErrorCode);       // Report the error
        break;
    }
    cout << "PASO" << endl;
    usleep((HikingTime * (CyclePercent3[x] - CyclePercent3[x - 1]))*1000000);
    cout << (HikingTime * (CyclePercent3[x] - CyclePercent3[x - 1]))*1000000 << endl;

}

//banTobillo = 1;     // Indicate that the capture of information must finish
return NULL;
}



//====================================================
//===================== Main code ====================
//====================================================

int main()
{

	OpenDevice();                   // Call the function that enable all EPOS4

    pthread_t thread1, thread2, thread3, thread4; // Create thread



	// Transformation of degrees to increments
	for (int y = 0; y <= 3; y++){

        TargetPosition[y] = TargetPosition[y]*2230;     // Save increments of the Hip
        }

	for (int y = 0; y <= 4; y++){

        TargetPosition2[y] = TargetPosition2[y]*487;    // Save increments of the Knee
        }

	for (int y = 0; y <= 6; y++){

        TargetPosition3[y] = TargetPosition3[y]*487;    // Save increments of the Ankle
	}

	// Calculation of speeds and accelerations of hip trajectory
    for (int i = 1; i <= 3; i++){

            Tf = HikingTime * (CyclePercent[i] - CyclePercent[i - 1]);      // Time of each section of the trajectory
            Vf = 2*(TargetPosition[i] - TargetPosition[i - 1]) / (Tf);      // Velocity of each section of the trajectory

            ProfileVelocity[i-1] = rint(abs(Vf*0.0145));                    // Save the velocity of each section in rpm
            ProfileAcceleration[i-1] = rint(2*ProfileVelocity[i-1]/Tf);
            ProfileDeceleration[i-1] = rint(2*ProfileVelocity[i-1]/Tf);
            cout << ProfileVelocity[i-1] << endl;                           // Show velocities
            cout << ProfileAcceleration[i-1] << endl;
            }

	// Calculation of speeds and accelerations of knee trajectory
    for (int i = 1; i <= 4; i++){

            Tf2 = HikingTime * (CyclePercent2[i] - CyclePercent2[i - 1]);      // Time of each section of the trajectory
            Vf2 = 2*(TargetPosition2[i] - TargetPosition2[i - 1]) / (Tf2);     // Velocity of each section of the trajectory

            ProfileVelocity2[i-1] = rint(abs(Vf2*0.0145));                    // Save the velocity of each section in rpm
            ProfileAcceleration2[i-1] = rint(2*ProfileVelocity2[i-1]/Tf2);
            ProfileDeceleration2[i-1] = rint(2*ProfileVelocity2[i-1]/Tf2);
            //cout << ProfileVelocity2[i-1] << endl;
            }


    // Calculation of speeds and accelerations of ankle trajectory
    for (int i = 1; i <= 6; i++){

            Tf3 = HikingTime * (CyclePercent3[i] - CyclePercent3[i - 1]);      // Time of each section of the trajectory
            Vf3 = 2*(TargetPosition3[i] - TargetPosition3[i - 1]) / (Tf3);     // Velocity of each section of the trajectory

            ProfileVelocity3[i-1] = rint(abs(Vf3*0.0145));                    // Save the velocity of each section in rpm
            ProfileAcceleration3[i-1] = rint(2*ProfileVelocity3[i-1]/Tf3);
            ProfileDeceleration3[i-1] = rint(2*ProfileVelocity3[i-1]/Tf3);
            }

    // Place all the joints in the initial position

    // Initial position Hip
    VCS_ActivateProfilePositionMode(g_pKeyHandle, g_usNodeId,  &p_pErrorCode);          // Activate the EPOS4 of the Hip with Profile Position Mode
	VCS_SetPositionProfile(g_pKeyHandle, g_usNodeId, 1000, 1000, 1000, &p_pErrorCode);  // Set the parameters to move the Hip to the initial position
	if (VCS_MoveToPosition(g_pKeyHandle, g_usNodeId, TargetPosition[0], Absolute, Immediately, &p_pErrorCode) == 0) // Move the Hip to initial position. If EPOS4 does not work, it will report an error
	{
        LogError("VCS_MoveToPosition - Hip", p_pErrorCode);
	}


    // Initial position Knee
    VCS_ActivateProfilePositionMode(g_pKeyHandle, g_usNodeId2,  &p_pErrorCode);         // Activate the EPOS4 of the Knee with Profile Position Mode
	VCS_SetPositionProfile(g_pKeyHandle, g_usNodeId2, 1000, 1000, 1000, &p_pErrorCode); // Set the parameters to move the Knee to the initial position
	if (VCS_MoveToPosition(g_pKeyHandle, g_usNodeId2, TargetPosition2[0], Absolute, Immediately, &p_pErrorCode) == 0) // Move the Knee to initial position. If EPOS4 does not work, it will report an error
    {

       LogError("VCS_MoveToPosition - Knee", p_pErrorCode);
	}

	// Initial position Ankle
	//VCS_ActivateProfilePositionMode(g_pKeyHandle, g_usNodeId3,  &p_pErrorCode);           // Activate the EPOS4 of the Ankle with Profile Position Mode
	//VCS_SetPositionProfile(g_pKeyHandle, g_usNodeId3, 1000, 1000, 1000, &p_pErrorCode);   // Set the parameters to move the Knee to the initial position
	//if (VCS_MoveToPosition(g_pKeyHandle, g_usNodeId3, TargetPosition3[0], Absolute, Immediately, &p_pErrorCode) == 0) // Move the Knee to initial position. If EPOS4 does not work, it will report an error
	//{
        //LogError("VCS_MoveToPosition - Ankle", p_pErrorCode);
	//}


	usleep(5000000);    // Give five seconds for the joints to stabilize

    cout << "Empezamos..." << endl;
    //VCS_SetProtocolStackSettings(g_pKeyHandle, 1000000, 50, &p_pErrorCode);

    //cout << "rate: " << pBaudrate << "Time: " << pTimeout << endl;
    int c;

    // Press ENTER to move the exoskeleton one time according to the hiking time set before
    // Press ESC and then ENTER to quit the program

    while ((c = getchar()) != 27){

        // These three variables let that the threads that capturing information work during the hiking
        banCaptura = 0;


        // Create three threads to run each joint according the before calculations
        pthread_create(&thread1, NULL, Cadera, NULL);
        pthread_create(&thread2, NULL, Rodilla, NULL);
        //pthread_create(&thread3, NULL, Tobillo, NULL);

        // Create three threads to capture the important information about the exoskeleton when it is working
        //pthread_create(&thread4, NULL, Data, NULL);


        // Run all the threads
        pthread_join(thread1, NULL);    // Thread that moves the Hip joint
        pthread_join(thread2, NULL);    // Thread that moves the Knee joint
        //pthread_join(thread3, NULL);  // Thread that moves the Ankle joint

        //pthread_join(thread4, NULL);    // Thread that captures information of Hip


    }

    // When the program ends, all joints return to the original position (0 degrees in each joint)
    VCS_MoveToPosition(g_pKeyHandle, g_usNodeId, 0, Absolute, Immediately, &p_pErrorCode);
    VCS_MoveToPosition(g_pKeyHandle, g_usNodeId2, 0, Absolute, Immediately, &p_pErrorCode);
    //VCS_MoveToPosition(g_pKeyHandle, g_usNodeId3, 0, Absolute, Immediately, &p_pErrorCode);
    usleep(5000000);


    // Disable all EPOS4 and the program ends
	VCS_SetDisableState(g_pKeyHandle, g_usNodeId, &p_pErrorCode);		//disable driver
	VCS_SetDisableState(g_pKeyHandle, g_usNodeId2, &p_pErrorCode);		//disable driver
    //VCS_SetDisableState(g_pKeyHandle, g_usNodeId3, &p_pErrorCode);		//disable driver


	cout << "Se termino \n";

	return 0;
}
