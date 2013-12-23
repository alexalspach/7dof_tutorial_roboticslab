/* RoboticsLab, Copyright 2008-2010 SimLab Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF SimLab Co., LTD.
 */
#ifndef __MY7DOF_TUTORIAL_ROBOTICSLAB_CONTROL_PD_H__
#define __MY7DOF_TUTORIAL_ROBOTICSLAB_CONTROL_PD_H__

#include <list>
#include "rControlAlgorithm/rControlAlgorithm.h"

#define _USE_RCONTROLALGORITHM_EX_

 struct JDevices
 {
 	rHANDLE		motor;
 	rHANDLE		enc;
 	rHANDLE		tacho;
 	
 	double		motor_reduction;
 	double		enc_reduction;
 	double		tacho_reduction;
 
 	JDevices() : motor(INVALID_RHANDLE), enc(INVALID_RHANDLE), tacho(INVALID_RHANDLE), motor_reduction(1), enc_reduction(1), tacho_reduction(1) { }
 	bool isAllInvalid() const { return ((motor == INVALID_RHANDLE) && (enc == INVALID_RHANDLE) && (tacho == INVALID_RHANDLE));}
 };

#ifdef _USE_RCONTROLALGORITHM_EX_
class REXPORT My7dof_tutorial_roboticslab_control_pd : public rControlAlgorithmEx
#else
class REXPORT My7dof_tutorial_roboticslab_control_pd : public rControlAlgorithm
#endif
{
public:
	My7dof_tutorial_roboticslab_control_pd(rDC rdc);
	~My7dof_tutorial_roboticslab_control_pd();

	virtual void init(int mode = 0);
	virtual void update(const rTime& t);
	virtual void setNominalSystem(const TCHAR* path, const TCHAR* aml, const HTransform& T0, const dVector& q0);
	virtual void setPeriod(const rTime& dT);
	virtual int command(const short& cmd, const int& arg = 0);
	virtual void datanames(vector<string_type>& names, int channel = -1);
	virtual void collect(vector<double>& data, int channel = -1);
	virtual void onSetInterestFrame(const TCHAR* name, const HTransform& T);

private:
	virtual void _estimate();
	virtual void _readDevices();
	virtual void _writeDevices();

	virtual void _reflect();
	virtual void _compute(const rTime& t);

	void _arrangeJointDevices();

	void _servoOn();
	void _servoOff();


private:
	std::list<JDevices>	_device;

	float				_dT;

	dVector				_q;
	dVector				_qdot;
	dVector				_torque;

	dVector				_q_des;
	dVector				_q_pre;

	dVector				_kp;
	dVector				_kd;

	int					_jdof;



};
#endif