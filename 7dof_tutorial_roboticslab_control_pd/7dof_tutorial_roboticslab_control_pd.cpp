/* RoboticsLab, Copyright 2008-2010 SimLab Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF SimLab Co., LTD.
 */
#include "7dof_tutorial_roboticslab_control_pd.h"
#include "7dof_tutorial_roboticslab_control_pdCmd.h"

My7dof_tutorial_roboticslab_control_pd::My7dof_tutorial_roboticslab_control_pd(rDC rdc) 
#ifdef _USE_RCONTROLALGORITHM_EX_
:rControlAlgorithmEx(rdc)
#else
:rControlAlgorithm(rdc)
#endif
, _jdof(0)
{
}

My7dof_tutorial_roboticslab_control_pd::~My7dof_tutorial_roboticslab_control_pd()
{
}

void My7dof_tutorial_roboticslab_control_pd::_servoOn()
{
}

void My7dof_tutorial_roboticslab_control_pd::_servoOff()
{
}

void My7dof_tutorial_roboticslab_control_pd::_arrangeJointDevices()
{
	 	for (int i = 0; i < _jdof; i++)
	 	{
	 		JDevices device;
	 
	 		TCHAR devname[32];
	 		_stprintf(devname, _T("motor%d"), i + 1);
	 		device.motor = findDevice(devname);
	 
	 		_stprintf(devname, _T("enc%d"), i + 1);
	 		device.enc = findDevice(devname);
	 
	 		_stprintf(devname, _T("tacho%d"), i + 1);
	 		device.tacho = findDevice(devname);
	 
	 		if (device.motor != INVALID_RHANDLE)
	 		{
	 			const TCHAR* reduction = getDeviceProperty(device.motor, _T("reduction"));
	 			if (reduction)
	 				device.motor_reduction = _tstof(reduction);
	 		}
	 
	 		if (device.enc != INVALID_RHANDLE)
	 		{
	 			const TCHAR* reduction = getDeviceProperty(device.enc, _T("reduction"));
	 			if (reduction)
	 				device.enc_reduction = _tstof(reduction);
	 		}
	 
	 		if (device.tacho != INVALID_RHANDLE)
	 		{
	 			const TCHAR* reduction = getDeviceProperty(device.tacho, _T("reduction"));
	 			if (reduction)
	 				device.tacho_reduction = _tstof(reduction);
	 		}
	 
	 		if (!device.isAllInvalid())
	 			_device.push_back(device);
	 	}
}

void My7dof_tutorial_roboticslab_control_pd::init(int mode)
{

	printf("\nController: 7dof_tutorial_roboticslab_control_pd\n\n");

	// if _USE_RCONTROLALGORITHM_EX_ is defined 
	// You can import the user-defined properties from its XDL file
	//const TCHAR* kp = getProperty(_T("kp"));
	//const TCHAR* kv = getProperty(_T("kv"));

	_jdof = 7;	// You should set the dimension.

	_arrangeJointDevices();
	
	_q.resize(_jdof);
	_qdot.resize(_jdof);
	_torque.resize(_jdof);

	_q.zero();
	_qdot.zero();
	_torque.zero();

	//  Example code for adding an interest frame..
	//	addInterestFrame(_T("desired frame"));
}


void My7dof_tutorial_roboticslab_control_pd::update(const rTime& t)
{
	rControlAlgorithm::update(t);
}

void My7dof_tutorial_roboticslab_control_pd::setNominalSystem(const TCHAR* path, const TCHAR* aml, const HTransform& T0, const dVector& q0)
{
}

void My7dof_tutorial_roboticslab_control_pd::setPeriod(const rTime& dT)
{
}

void My7dof_tutorial_roboticslab_control_pd::_readDevices()
{
	 	float value = 0;
	 	int i = 0;
	 	for (std::list<JDevices>::iterator dev = _device.begin(); dev != _device.end(); dev++, i++)
		{
	 		if (dev->enc != INVALID_RHANDLE)
	 		{
	 			readDeviceValue(dev->enc, &value, 1*sizeof(float));
	 			_q[i] = value / dev->enc_reduction;
				//printf("%d: %2.3f\t", i, _q[i]);
	 		}
			
	 		/*if (dev->tacho != INVALID_RHANDLE)
	 		{
	 			readDeviceValue(dev->tacho, &value, 1*sizeof(float));
	 			_qdot[i] = value / dev->tacho_reduction;
	 		}*/
	 	}
		//printf("\n");
}

void My7dof_tutorial_roboticslab_control_pd::_writeDevices()
{
	 	float value;
	 	int i = 0;
	 	for (std::list<JDevices>::iterator dev = _device.begin(); dev != _device.end(); dev++, i++)
	 	{
	 		if (dev->motor != INVALID_RHANDLE)
	 		{
	 			value = 0.0; // _torque[i] / dev->motor_reduction;
	 			writeDeviceValue(dev->motor, &value, 1*sizeof(float));
	 		}
	 	}
}

void My7dof_tutorial_roboticslab_control_pd::_reflect()
{
}

void My7dof_tutorial_roboticslab_control_pd::_compute(const double& t)
{
}

void My7dof_tutorial_roboticslab_control_pd::_estimate()
{
}

int My7dof_tutorial_roboticslab_control_pd::command(const short& cmd, const int& arg)
{
	switch (cmd)
	{
	case DEFAULT_CMD:
		break;

		// 	case USER_CMD_1:
		// 		{
		//			// For example, find a device and write a value.
		// 			rHANDLE device = findDevice(_T("YOUR_DEVICE_NAME")); 
		// 			writeDeviceValue(device, &value, 1*sizeof(float));
		// 		}
		// 
		// 		break;


	case RESERVED_CMD_SERVO_ON:
		_servoOn();

		break;

	case RESERVED_CMD_SERVO_OFF:
		_servoOff();

		break;

	default:
		break;
	}

	return 0;
}

void My7dof_tutorial_roboticslab_control_pd::datanames(vector<string_type>& names, int channel)
{

	switch (channel)
    {
            case 240:
                    {
                            TCHAR dname[64];
                            for (int i=0; i<_jdof; i++) {
                                    _stprintf(dname, _T("J%d"), i);
                                    names.push_back(dname);
                            }
                    }
                    break; 
    }

}

void My7dof_tutorial_roboticslab_control_pd::collect(vector<double>& data, int channel)
{
	switch (channel)
    {
			case 240:
            {
				for (int i=0; i<_jdof; i++) {
                    data.push_back(_q[i]);
				}

            }
            break;
    }
}

void My7dof_tutorial_roboticslab_control_pd::onSetInterestFrame(const TCHAR* name, const HTransform& T)
{
}

rControlAlgorithm* CreateControlAlgorithm(rDC& rdc)
{
	return new My7dof_tutorial_roboticslab_control_pd(rdc);
}
