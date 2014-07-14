//*********************************************************************************
// Arduino PID Library Version 1.0.1 Modified Version for C++
// Platform Independent
// 
// Revision: 1.1
// 
// Description: The PID Controller module originally meant for Arduino made
// platform independent. Some small bugs present in the original Arduino source
// have been rectified as well.
// 
// For a detailed explanation of the theory behind this library, go to:
// http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
// 
// Revisions can be found here:
// https://github.com/tcleg
// 
// Modified by: Trent Cleghorn , <trentoncleghorn@gmail.com>
// 
// Copyright (C) Brett Beauregard , <br3ttb@gmail.com>
// 
//                                 GPLv3 License
// 
// This program is free software: you can redistribute it and/or modify it under 
// the terms of the GNU General Public License as published by the Free Software 
// Foundation, either version 3 of the License, or (at your option) any later 
// version.
// 
// This program is distributed in the hope that it will be useful, but WITHOUT ANY 
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
// PARTICULAR PURPOSE.  See the GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License along with 
// this program.  If not, see <http://www.gnu.org/licenses/>.
//*********************************************************************************

//*********************************************************************************
// Headers
//*********************************************************************************
#include "pid_controller.h"

//*********************************************************************************
// Macros and Globals
//*********************************************************************************
#define CONSTRAIN(x,lower,upper)    ((x)<(lower)?(lower):((x)>(upper)?(upper):(x)))

//*********************************************************************************
// Public Class Functions
//*********************************************************************************

PIDControl::
PIDControl (float kp, float ki, float kd, float sampleTimeSeconds, float minOutput, 
            float maxOutput, PIDMode mode, PIDDirection controllerDirection)     	
{
    controllerDirection = controllerDirection;
    mode = mode;
    iTerm = 0.0f;
    input = 0.0f;
    lastInput = 0.0f;
    output = 0.0f;
    setpoint = 0.0f;
    
    if(sampleTimeSeconds > 0.0f)
    {
        sampleTime = sampleTimeSeconds;
    }
    else
    {
        // If the passed parameter was incorrect, set to 1 second
        sampleTime = 1.0f;
    }
    
    PIDOutputLimitsSet(minOutput, maxOutput);
    PIDTuningsSet(kp, ki, kd);
}
        
bool PIDControl::
PIDCompute() 
{
    float error, dInput;

    if(mode == MANUAL)
    {
        return false;
    }
    
    // The classic PID error term
    error = setpoint - input;
    
    // Compute the integral term separately ahead of time
    iTerm += alteredKi * error;
    
    // Constrain the integrator to make sure it does not exceed output bounds
    iTerm = CONSTRAIN(iTerm, outMin, outMax);
    
    // Take the "derivative on measurement" instead of "derivative on error"
    dInput = input - lastInput;
    
    // Run all the terms together to get the overall output
    output = alteredKp * error + iTerm - alteredKd * dInput;
    
    // Bound the output
    output = CONSTRAIN(output, outMin, outMax);
    
    // Make the current input the former input
    lastInput = input;
    
    return true;
}
     
void PIDControl::
PIDModeSet(PIDMode mode)                                                                                                                                       
{
    // If the mode changed from MANUAL to AUTOMATIC
    if(mode != mode && mode == AUTOMATIC)
    {
        // Initialize a few PID parameters to new values
        iTerm = output;
        lastInput = input;
        
        // Constrain the integrator to make sure it does not exceed output bounds
        iTerm = CONSTRAIN(iTerm, outMin, outMax);
    }
    
    mode = mode;
}

void PIDControl::
PIDOutputLimitsSet(float min, float max) 							  							  
{
    // Check if the parameters are valid
    if(min >= max)
    {
        return;
    }
    
    // Save the parameters
    outMin = min;
    outMax = max;
    
    // If in automatic, apply the new constraints
    if(mode == AUTOMATIC)
    {
        output = CONSTRAIN(output, min, max);
        iTerm  = CONSTRAIN(iTerm,  min, max);
    }
}

void PIDControl::
PIDTuningsSet(float kp, float ki, float kd)         	                                         
{
    // Check if the parameters are valid
    if(kp < 0.0f || ki < 0.0f || kd < 0.0f)
    {
        return;
    }
    
    // Save the parameters for displaying purposes
    dispKp = kp;
    dispKi = ki;
    dispKd = kd;
    
    // Alter the parameters for PID
    alteredKp = kp;
    alteredKi = ki * sampleTime;
    alteredKd = kd / sampleTime;
    
    // Apply reverse direction to the altered values if necessary
    if(controllerDirection == REVERSE)
    {
        alteredKp = -(alteredKp);
        alteredKi = -(alteredKi);
        alteredKd = -(alteredKd);
    }
}

void PIDControl::
PIDTuningKpSet(float kp)
{
    PIDTuningsSet(kp, dispKi, dispKd);
}

void PIDControl::
PIDTuningKiSet(float ki)
{
    PIDTuningsSet(dispKp, ki, dispKd);
}

void PIDControl::
PIDTuningKdSet(float kd)
{
    PIDTuningsSet(dispKp, dispKi, kd);
}

void PIDControl::
PIDControllerDirectionSet(PIDDirection controllerDirection)	  									  									  									  
{
    // If in automatic mode and the controller's sense of direction is reversed
    if(mode == AUTOMATIC && controllerDirection == REVERSE)
    {
        // Reverse sense of direction of PID gain constants
        alteredKp = -(alteredKp);
        alteredKi = -(alteredKi);
        alteredKd = -(alteredKd);
    }
    
    controllerDirection = controllerDirection;
}

void PIDControl::
PIDSampleTimeSet(float sampleTimeSeconds)                                                       									  									  									   
{
    float ratio;

    if(sampleTimeSeconds > 0.0f)
    {
        // Find the ratio of change and apply to the altered values
        ratio = sampleTimeSeconds / sampleTime;
        alteredKi *= ratio;
        alteredKd /= ratio;
        
        // Save the new sampling time
        sampleTime = sampleTimeSeconds;
    }
}


