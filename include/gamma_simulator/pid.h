/*
 * pid.h
 *
 *  Created on: Dec 3, 2020
 *      Author: by
 */

#ifndef __H
#define __H
#define FREQUENCY 1000

class PID
{
private:
	//Controller parameters
	double P_ = 0;
	double I_ = 0;
	double D_ = 0;
	double F_ = 0;

	//Limits
	double maxIOutput_ = 0;
	double maxError_ = 0;
	double errorSum_ = 0;
	double prevError_ = 0;
	double deadTime_ = 0;
	double frequency_ = 50;

	double maxOutput_ = 0;
	double minOutput_ = 0;

	double setpoint_ = 0;

	double lastActual_ = 0;

	//Flags
	bool firstRun_ = true;
	bool reversed_ = false;

	//Ramping and descent limits
	double outputRampRate_ = 0;
	double outputDescentRate_ = 0;
	double lastOutput_ = 0;

	double outputFilter_ = 0;

	double setpointRange_ = 0;

	unsigned int prev_time_;

	double clamp(double value, double min, double max);
	bool bounded(double value, double min, double max);
	void checkSigns();

public:
	PID();
	void setPID(double p, double i, double d);
	void setPIDF(double p, double i, double d, double f);
	void setMaxIOutput(double maximum);
	void setOutputLimits(double min, double max);
	void setDirection(bool reversed);
	void setSetpoint(double setpoint);
	double skipCycle(unsigned int time);
	double getOutput(double actual, double setpoint, unsigned int time);
	double getOutputFast(unsigned int time);
	void reset();
	void setOutputRampRate(double rate);
	void setOutputDescentRate(double rate);
	void setSetpointRange(double range);
	void setOutputFilter(double strength);
	void setFrequency(double freq);
};

#endif /* INC_PID_H_ */
