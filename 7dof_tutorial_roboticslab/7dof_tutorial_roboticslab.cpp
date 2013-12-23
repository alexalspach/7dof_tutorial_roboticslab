

	string_type aml = _T("D:/Projects/gitRepos/alexalspach/roboticslab/7dof_tutorial_roboticslab/models/wam7.aml");

		string_type control_plugin_path = _T("controls/7dof_tutorial_roboticslab_control_pd.dll");







/* RoboticsLab, Copyright 2008-2011 SimLab Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF SimLab Co., LTD.
 */
#include "stdafx.h"

#include "rMath/rMath.h"
using namespace rMath;

#include "rCommand/rCmdDefine.h"

//#include "../controlXXXX/controlXXXXCmd.h"
#include "../7dof_tutorial_roboticslab_control_pd/7dof_tutorial_roboticslab_control_pdCmd.h"

#include "rxSDK/rxSDK.h"
#include "rCommon/rCodeUtil.h"

bool bContact = false;		// Enables/disables contact dynamics.
bool bQuit = false;			// Set this flag true to quit this program.
bool bRun = false;			// Set this flag true to activate the program.
										// See OnKeyRun() function for the details.
// const rTime delT = 0.005;
const rTime delT = 0.002;
string_type aml_path = _T("D:/Projects/gitRepos/alexalspach/roboticslab/7dof_tutorial_roboticslab/models/wam7.aml");
string_type aml_name = _T("WAM");
HTransform aml_T0;
dVector aml_q0;
rxSystem* sys = NULL;
string_type eml_path = _T("models/default.eml");
string_type eml_name = _T("Environment");
HTransform eml_T0;
rxEnvironment* env = NULL;
rxControlInterface* control = NULL;
//string_type control_path = _T("controls/rControlDummy.dll");
// This path can target an XDL instead
string_type control_path = _T("controls/7dof_tutorial_roboticslab_control_pd.dll");
string_type control_name = _T("MyController");

void MyKeyboardHandler(int key, void* data);
void MyControlCallback(rTime time, void* data);
void SetupDAQ();

int _tmain(int argc, _TCHAR* argv[])
{
	rCreateWorld(bContact, delT);
	rSetGravity(0, 0, -GRAV_ACC);
	rCreatePlane(0, 0, 1, 0);

	aml_T0.r[0] = 0.0; aml_T0.r[1] = 0.0; aml_T0.r[2] = 0.0;
	sys = rCreateSystem(aml_path, aml_name, aml_T0, aml_q0);
	env = rCreateEnvironment(eml_path, eml_name, eml_T0);

	rInitializeEx(true, true);

	if (sys)
	{
		int step = 1;
		control = rCreateController(control_name, sys, step);
		control->setAlgorithmDll(control_path);
		control->setPeriod(step*delT);
		control->setNominalSystem(aml_path, aml_name, aml_T0, aml_q0);
		control->initAlgorithm();
	}

	SetupDAQ();
	
	rAddKeyboardHandler(MyKeyboardHandler, NULL);
	//rAddControlHandler(MyControlCallback, NULL);
	rRun(-1);

	return 0;
}

void MyKeyboardHandler(int key, void* data)
{
	switch (key)
	{
	case VK_TAB:
		{
			bRun = !bRun;
			if(bRun)
			{
				if (control) 
					control->command(RESERVED_CMD_SERVO_ON);
				rActivateWorld();
			}
			else
			{
				if (control)
					control->command(RESERVED_CMD_SERVO_OFF);
				rDeactivateWorld();
			}
		}
		break;
	
	case VK_Q:
		{
			if (control) 
				control->command(RESERVED_CMD_SERVO_ON);
			rDeactivateWorld();
			//bQuit = true;
			rQuit();
		}
		break;

	case VK_H:
	case VK_Z:
		{
			printf("Home CMD.\n");
			if (control)
				control->command(RESERVED_CMD_GO_HOME, key);
		}
		break;

	case VK_1:
		printf("1 pressed.\n");
		break;
	}
}


void SetupDAQ()
{
        rID pid_q = rdaqCreatePlot(_T("Encoders"), eDataPlotType_TimeLine);
        rdaqAddData(pid_q, control, 240);

        rID pid_q_des = rdaqCreatePlot(_T("Desired Pos"), eDataPlotType_TimeLine);
        rdaqAddData(pid_q_des, control, 241);

        rID pid_pd1 = rdaqCreatePlot(_T("PD Control"), eDataPlotType_TimeLine);
        rdaqAddData(pid_pd1, control, 250);
}


void MyControlCallback(rTime time, void* data)
{
	 printf("MyControlCallback: time=%.3f\n", time);
}
