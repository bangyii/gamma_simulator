/*
 * pid.c
 *
 *  Created on: Dec 3, 2020
 *      Author: by
 */

#include "gamma_simulator/pid.h"

PID::PID()
{
}

void PID::checkSigns()
{
    if (reversed_)
    {
        if (P_ > 0)
            P_ *= -1;
        if (I_ > 0)
            I_ *= -1;
        if (D_ > 0)
            D_ *= -1;
        if (F_ > 0)
            F_ *= -1;
    }

    else
    {
        if (P_ < 0)
            P_ *= -1;
        if (I_ < 0)
            I_ *= -1;
        if (D_ < 0)
            D_ *= -1;
        if (F_ < 0)
            F_ *= -1;
    }
}

double PID::clamp(double value, double min, double max)
{
    if (value > max)
        return max;

    if (value < min)
        return min;

    return value;
}

bool PID::bounded(double value, double min, double max)
{
    return (value > min) && (value < max);
}

void PID::setPID(double p, double i, double d)
{
    P_ = p;
    I_ = i;
    D_ = d;
    checkSigns();
}

void PID::setPIDF(double p, double i, double d, double f)
{
    P_ = p;
    I_ = i;
    D_ = d;
    F_ = f;
    checkSigns();
}

void PID::setMaxIOutput(double maximum)
{
    maxIOutput_ = maximum;
    if (I_ != 0)
        maxError_ = maxIOutput_ / I_;
}

void PID::setOutputLimits(double min, double max)
{
    if (max > min)
    {
        maxOutput_ = max;
        minOutput_ = min;
    }

    if (maxIOutput_ == 0 || maxIOutput_ > (max - min))
    {
        setMaxIOutput(max - min);
    }
}

void PID::setDirection(bool reversed)
{
    reversed_ = reversed;
}

void PID::setSetpoint(double setpoint)
{
    setpoint_ = setpoint;
}

double PID::skipCycle(unsigned int time)
{
    prev_time_ = time;
    return lastOutput_;
}

double PID::getOutput(double actual, double setpoint, unsigned int time)
{
    double output = 0;
    double Poutput = 0;
    double Ioutput = 0;
    double Doutput = 0;
    double Foutput = 0;

    //Remember old errorSum for use in reverting errorSum if any limits are reached later on
    double oldErrorSum = errorSum_;

    setpoint_ = setpoint;

    //Do the simple parts of the calculations
    double error = setpoint_ - actual;

    //If this is our first time running this  we don't actually _have_ a previous input or output.
    //For sensor, sanely assume it was exactly where it is now.
    //For last output, we can assume it's the current time-independent outputs.
    if (firstRun_)
    {
        lastActual_ = actual;
        prevError_ = error;
        lastOutput_ = Poutput + Foutput;
        prev_time_ = time;
        firstRun_ = false;
    }

    //Get time difference since last run
    double dt = (double)(time - prev_time_) / (double)frequency_;

    //Only run cycle when time passed is greater than 1/Hz
    if (dt < 1.0 / frequency_)
        return lastOutput_;

    //Ramp the setpoint used for calculations if user has opted to do so
    if (setpointRange_ != 0)
    {
        setpoint_ = clamp(setpoint_, actual - setpointRange_, actual + setpointRange_);
    }

    //Calculate F output. Notice, this depends only on the setpoint, and not the error.
    Foutput = F_ * setpoint_;

    //Calculate P term
    Poutput = P_ * error;

    //Calculate D Term
    //If rate of change of error is positive, then the derivative term should be positive to track
    //target setpoint, as system is lagging behind
    double error_rate = (error - prevError_) / dt;
    Doutput = D_ * error_rate;

    //The Iterm is more complex. There's several things to factor in to make it easier to deal with.
    // 1. maxIoutput restricts the amount of output contributed by the Iterm.
    // 2. prevent windup by not increasing errorSum if we're already running against our max Ioutput
    // 3. prevent windup by not increasing errorSum if output is output=maxOutput
    errorSum_ += error * dt;
    Ioutput = I_ * errorSum_;

    //Case 2: Clamp IOutput to max allowed integral output
    if (maxIOutput_ != 0 && !bounded(Ioutput, -maxIOutput_, maxIOutput_))
    {
        Ioutput = clamp(Ioutput, -maxIOutput_, maxIOutput_);

        //Max Ioutput reached, clamp errorSum
        errorSum_ = oldErrorSum;
    }

    //And, finally, we can just add the terms up
    output = Foutput + Poutput + Ioutput + Doutput;

    //Restrict output to our specified output and ramp limits
    //Output decent rate should be negative
    if (outputRampRate_ != 0 && outputDescentRate_ != 0)
    {
        //If output is positive, allow outputRampRate increase and outputDescentRate decrease
        if (lastOutput_ > 0)
        {
            if (!bounded(output, lastOutput_ + outputDescentRate_ * dt, lastOutput_ + outputRampRate_ * dt))
            {
                output = clamp(output, lastOutput_ + outputDescentRate_ * dt, lastOutput_ + outputRampRate_ * dt);
                errorSum_ = oldErrorSum;
            }
        }

        else
        {
            if (!bounded(output, lastOutput_ - outputRampRate_ * dt, lastOutput_ - outputDescentRate_ * dt))
            {
                output = clamp(output, lastOutput_ - outputRampRate_ * dt, lastOutput_ - outputDescentRate_ * dt);
                errorSum_ = oldErrorSum;
            }
        }
    }

    //Restrict output if output surpasses max and minimum values
    if (minOutput_ != maxOutput_ && !bounded(output, minOutput_, maxOutput_))
    {
        output = clamp(output, minOutput_, maxOutput_);

        //Prevent errorsum from increasing if max output is already capped
        errorSum_ = oldErrorSum;
    }

    if (outputFilter_ != 0)
    {
        output = lastOutput_ * outputFilter_ + output * (1 - outputFilter_);
    }

    lastOutput_ = output;
    prev_time_ = time;
    prevError_ = error;
    lastActual_ = actual;
    return output;
}

double PID::getOutputFast(unsigned int time)
{
    return getOutput(lastActual_, setpoint_, time);
}

void PID::reset()
{
    firstRun_ = true;
    errorSum_ = 0;
}

void PID::setOutputRampRate(double rate)
{
    outputRampRate_ = rate;
}

void PID::setOutputDescentRate(double rate)
{
    outputDescentRate_ = rate;
}

void PID::setSetpointRange(double range)
{
    setpointRange_ = range;
}

void PID::setOutputFilter(double strength)
{
    if (strength == 0 || bounded(strength, 0, 1))
        outputFilter_ = strength;
}

void PID::setFrequency(double freq)
{
    frequency_ = freq;
}
