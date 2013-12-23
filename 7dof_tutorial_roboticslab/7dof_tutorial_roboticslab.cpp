/* RoboticsLab, Copyright 2008-2011 SimLab Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF SimLab Co., LTD.
 */
#include "stdafx.h"

#include "rMath/rMath.h"
using namespace rMath;

#include "rCommand/rCmdDefine.h"
#include "rxSDK/rxSDK.h"

//#include "../controlXXXX/controlXXXXCmd.h"

#ifdef USING_NAMESPACE_RTERM
#include "rTerm/rTerm.h"
using namespace rTerm;
#endif

bool bContact = true;		// Enables/disables contact dynamics.
bool bQuit = false;			// Set this flag true to quit this program.
bool bRun = false;			// Set this flag true to activate the program.
							// See OnKeyRun() function for the details.

#ifdef USING_NAMESPACE_RTERM
// Defining a macro, RPLAYER_AUTO_START, makes your program start rPlayer automatically.
// If you don't want rPlayer to be started automatically, comment out the code below.
#define RPLAYER_AUTO_START
// Defining a macro, RPLOT_AUTO_START, makes your program start rPlot automatically.
// If you don't want rPlot to be started automatically, comment out the code below.
#define RPLOT_AUTO_START
#endif

void OnKeyQuit(void* data)
{
	rxControlInterface* control = reinterpret_cast<rxControlInterface*>(data);
	if (control) 
		control->command(RESERVED_CMD_SERVO_OFF);

	PHYSICS_WORLD->deactivateWorld();
	bQuit = true;
}

void OnKeyRun(void* data)
{
	static bool bServoOn = false;
	if (!bServoOn)
	{
		rxControlInterface* control = reinterpret_cast<rxControlInterface*>(data);
		if (control) 
			control->command(RESERVED_CMD_SERVO_ON);

		bServoOn = true;
	}

	bRun = !bRun;
	if(bRun)
		PHYSICS_WORLD->activateWorld();
	else
		PHYSICS_WORLD->deactivateWorld();
}

void OnCmdControlMode(int key, void* data)
{
	rxControlInterface* control = reinterpret_cast<rxControlInterface*>(data);
	if (!control) 
		return;

	switch (key)
	{
	case VK_H:
	case VK_Z:
		control->command(RESERVED_CMD_GO_HOME, key);
		break;

	default:
		break;
	}
}

void SetKeyboardHandler(rxSystem* sys, rxControlInterface* control)
{
	PHYSICS_WORLD->addKeyboardEvent(VK_Q, OnKeyQuit, control);
	PHYSICS_WORLD->addKeyboardEvent(VK_TAB, OnKeyRun, control);

	printf("To run/pause simulation, press TAB key.\n");
	printf("To quit simulation, press 'q' key.\n");
	PHYSICS_WORLD->printLogMessage(_T("To run/pause simulation, press TAB key."));
	PHYSICS_WORLD->printLogMessage(_T("To quit simulation, press 'q' key."));

	PHYSICS_WORLD->addKeyboardEvent(VK_H, OnCmdControlMode, control);
	PHYSICS_WORLD->addKeyboardEvent(VK_Z, OnCmdControlMode, control);
}

int _tmain(int argc, _TCHAR* argv[])
{
	const rTime delT = 0.005;						// Basic time increment.

	PHYSICS_WORLD->createWorld(bContact, delT);		// Create a world for simulation.
	if (!PHYSICS_WORLD->isWorldCreated())			// Check if a world is created successfully.
	{
		printf("Failed to create a physics world. Press any key to quit...");
		getchar();
		return -1;
	}

	PHYSICS_WORLD->setGravity(0, 0, -GRAV_ACC);		// Set gravitational vector.
													// It is only valid for simulation(virtual) world.
													// By default, it is already set as (0, 0, -GRAV_ACC).

	if (bContact)									// If contact dynamics is enabled, then create a infinite plane for the ground.
		PHYSICS_WORLD->createPlane(0, 0, 1, 0);

	printf("wait for connection..\n");
#ifdef RPLAYER_AUTO_START
	startPlayer("-ip 127.0.0.1 -port 5150");		// Start rPlayer.
#endif
#ifdef RPLOT_AUTO_START
	startPlot("-ip 127.0.0.1 -port 5150");			// Start rPlot.
#endif
	PHYSICS_WORLD->makeNetwork(1000);				// Create a network instance.
													// This function is blocked until a new client such as rPlayer is connected or timeout is occurred.
	
	//string_type aml = _T("models/Manipulator/WAM/wam7.aml");
	string_type aml = _T("D:/Projects/gitRepos/alexalspach/roboticslab/7dof_tutorial_roboticslab/models/wam7.aml");
													// Set AML file path to load including file extension, 'aml'.
													// It can be a relative path based on the program working directory or absolute file path.
													// Usually the working directory is set from the system environment variable, $(RLAB_BIN_PATH).
	string_type name = _T("SAP1");					// Set the name of your model. Each name should be unique.

	HTransform T0;									// Initial position and orientation of your robot. 
	dVector q0;										// Initial joint position, q values of robot.
	rxSystem* sys = NULL;							// Loaded system.

	sys = PHYSICS_WORLD->createSystem(aml, name, T0, q0); // Load a system and put it into the world.
	if (!sys)										// Check if the system is created successfully.
	{
		printf("Failed to load a robot model(AML). Press any key to quit...");
		getchar();
		return -1;
	}

	string_type eml_path = _T("models/default.eml");
													// Set EML file path to load including file extension, 'eml'.
													// It can be a relative path based on the program working directory or absolute file path.
	string_type eml_name = _T("evn");				// Set the name of your environment. Each name should be unique.
	HTransform eml_T0;								// Initial position and orientation of your environment. 
	rxEnvironment* env = NULL;
	env = PHYSICS_WORLD->createEnvironment(eml_path, eml_name, eml_T0); // Load an environment and put it into the world.
	if (!env)										// Check if the system is created successfully.
	{
		printf("Failed to load an environment model(EML). Press any key to quit...");
		getchar();
		return -1;
	}

	PHYSICS_WORLD->initialize();					// Initialize the world.
													// Every system should be created before this function call.

	rxControlInterface* control = NULL;
	if (sys)
	{
		int step = 1;								// Controller is updated once per 'step' simulation times.
													// When step is 0, the controller should be updated by user manually.
		control = PHYSICS_WORLD->createController(_T("controller_name"), sys, step);
		string_type control_plugin_path = _T("controls/rControlDummy.dll");
													// Set your control plugin DLL file path relative to working directory.
													// Usually the working directory is set from the system environment variable, $(RLAB_BIN_PATH).
		control->setAlgorithmDll(control_plugin_path);
		control->setPeriod(step*delT);
		control->setNominalSystem(aml, name, T0, q0);
		control->initAlgorithm();
	}

	SetKeyboardHandler(sys, control);				// Setup keyboard handler.
													// See the function implementation of 'SetKeyboardHandler()' for the details.

	while (!bQuit)									// Program main-loop.
	{
		PHYSICS_WORLD->update();
	}

	DESTROY_DATA_ACQUISITION();
	DESTROY_PHYSICS_WORLD();						// Destroy all instances created.

#ifdef RPLAYER_AUTO_START
	stopPlayer();									// Exit all the rPlayer instances.
#endif
#ifdef RPLOT_AUTO_START
	stopPlot();										// Exit all the rPlot instances.
#endif

	return 0;
}
