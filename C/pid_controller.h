//*********************************************************************************
// Arduino PID Library Version 1.0.1 Modified Version for C -
// Platform Independent
// 
// Revision: 1.0
// 
// Description: The PID Controller module originally meant for Arduino made
// platform independent. Some small bugs present in the original Arduino source
// have been rectified as well.
// 
// For a detailed explanation of the theory behind this library, go to:
// http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
// 
// Revisions can be found here:
// https://www.dropbox.com/sh/5aqe3wxf9fetv4s/ne4C3lgxDR
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

// 
// Header Guard
// 
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

//*********************************************************************************
// Headers
//*********************************************************************************
#include <stdint.h>
#include <stdbool.h>

// 
// C Binding for C++ Compilers
// 
#ifdef __cplusplus
extern "C"
{
#endif

//*********************************************************************************
// Macros and Globals
//*********************************************************************************

typedef enum
{
    MANUAL,
    AUTOMATIC
}
ePIDMode;

typedef enum
{
    DIRECT,
    REVERSE
}
ePIDDirection;

typedef struct
{
    // 
    // Input to the PID Controller
    // 
    float input;
    
    // 
    // Previous input to the PID Controller
    // 
    float lastInput;
    
    // 
    // Output of the PID Controller
    // 
    float output;
    
    // 
    // Gain constant values that were passed by the user
    // These are for display purposes
    // 
    float dispKp;
    float dispKi;
    float dispKd;
    
    // 
    // Gain constant values that the controller alters for
    // its own use
    // 
    float alteredKp;
    float alteredKi;
    float alteredKd;
    
    // 
    // The Integral Term
    // 
    float iTerm;
    
    // 
    // The interval (in seconds) on which the PID controller
    // will be called
    // 
    float sampleTime;
    
    // 
    // The values that the output will be constrained to
    // 
    float outMin;
    float outMax;
    
    // 
    // The user chosen operating point
    // 
    float setpoint;
    
    // 
    // The sense of direction of the controller
    // DIRECT:  A positive setpoint gives a positive output
    // REVERSE: A positive setpoint gives a negative output
    // 
    ePIDDirection controllerDirection;
    
    // 
    // Tells how the controller should respond if the user has
    // taken over manual control or not
    // MANUAL:    PID controller is off.
    // AUTOMATIC: PID controller is on.
    // 
    ePIDMode mode;
}
tPIDParam;

//*********************************************************************************
// Prototypes
//*********************************************************************************

// 
// PID Initialize
// Description:
//      Initializes a tPIDParam instantiation. This should be called at least once
//      before any other PID functions are called.
// Parameters:
//      pid - The address of a tPIDParam instantiation.
//      kp - Positive P gain constant value.
//      ki - Positive I gain constant value.
//      kd - Positive D gain constant value.
//      sampleTimeSeconds - Interval in seconds on which PIDCompute will be called.
//      minOutput - Constrain PID output to this minimum value.
//      maxOutput - Constrain PID output to this maximum value.
//      mode - Tells how the controller should respond if the user has taken over
//             manual control or not.
//             MANUAL:    PID controller is off. User can manually control the output.
//             AUTOMATIC: PID controller is on. PID controller controls the output.
//      controllerDirection - The sense of direction of the controller
//                            DIRECT:  A positive setpoint gives a positive output.
//                            REVERSE: A positive setpoint gives a negative output.
// Returns:
//      Nothing.
// 
extern void PIDInit(tPIDParam *pid, float kp, float ki, float kd, 
                    float sampleTimeSeconds, float minOutput, float maxOutput, 
                    ePIDMode mode, ePIDDirection controllerDirection);     	

// 
// PID Compute
// Description:
//      Should be called on a regular interval defined by sampleTimeSeconds.
//      Typically, PIDSetpointSet and PIDInputSet should be called immediately
//      before PIDCompute.
// Parameters:
//      pid - The address of a tPIDParam instantiation.
// Returns:
//      True if in AUTOMATIC. False if in MANUAL.
//                     
extern bool PIDCompute(tPIDParam *pid); 

// 
// PID Mode Set
// Description:
//      Sets the PID controller to a new mode.
// Parameters:
//      pid - The address of a tPIDParam instantiation.
//      mode - Tells how the controller should respond if the user has taken over
//             manual control or not.
//             MANUAL:    PID controller is off. User can manually control the output.
//             AUTOMATIC: PID controller is on. PID controller controls the output.
// Returns:
//      Nothing.
//              
extern void PIDModeSet(tPIDParam *pid, ePIDMode mode);                                                                                                                                       

// 
// PID Output Limits Set
// Description:
//      Sets the new output limits. The new limits are applied to the PID
//      immediately.
// Parameters:
//      pid - The address of a tPIDParam instantiation.
//      min - Constrain PID output to this minimum value.
//      max - Constrain PID output to this maximum value.
// Returns:
//      Nothing.
// 
extern void PIDOutputLimitsSet(tPIDParam *pid, float min, float max); 							  							  

// 
// PID Tunings Set
// Description:
//      Sets the new gain constant values.
// Parameters:
//      pid - The address of a tPIDParam instantiation.
//      kp - Positive P gain constant value.
//      ki - Positive I gain constant value.
//      kd - Positive D gain constant value.
// Returns:
//      Nothing.
// 
extern void PIDTuningsSet(tPIDParam *pid, float kp, float ki, float kd);         	                                         

// 
// PID Tuning Gain Constant P Set
// Description:
//      Sets the proportional gain constant value.
// Parameters:
//      pid - The address of a tPIDParam instantiation.
//      kp - Positive P gain constant value.
// Returns:
//      Nothing.
// 
extern void PIDTuningKpSet(tPIDParam *pid, float kp);

// 
// PID Tuning Gain Constant I Set
// Description:
//      Sets the proportional gain constant value.
// Parameters:
//      pid - The address of a tPIDParam instantiation.
//      ki - Positive I gain constant value.
// Returns:
//      Nothing.
// 
extern void PIDTuningKiSet(tPIDParam *pid, float ki);

// 
// PID Tuning Gain Constant D Set
// Description:
//      Sets the proportional gain constant value.
// Parameters:
//      pid - The address of a tPIDParam instantiation.
//      kd - Positive D gain constant value.
// Returns:
//      Nothing.
// 
extern void PIDTuningKdSet(tPIDParam *pid, float kd);

// 
// PID Controller Direction Set
// Description:
//      Sets the new controller direction.
// Parameters:
//      pid - The address of a tPIDParam instantiation.
//      controllerDirection - The sense of direction of the controller
//                            DIRECT:  A positive setpoint gives a positive output
//                            REVERSE: A positive setpoint gives a negative output
// Returns:
//      Nothing.
// 
extern void PIDControllerDirectionSet(tPIDParam *pid, ePIDDirection controllerDirection);	  									  									  									  

// 
// PID Sample Time Set
// Description:
//      Sets the new sampling time (in seconds).
// Parameters:
//      pid - The address of a tPIDParam instantiation.
//      sampleTimeSeconds - Interval in seconds on which PIDCompute will be called.
// Returns:
//      Nothing.
// 
extern void PIDSampleTimeSet(tPIDParam *pid, float sampleTimeSeconds);                                                       									  									  									   

// 
// Basic Set and Get Functions for PID Parameters
// 
inline void 
PIDSetpointSet(tPIDParam *pid, float setpoint) { pid->setpoint = setpoint; }

inline void 
PIDInputSet(tPIDParam *pid, float input) { pid->input = input; }

inline float 
PIDOutputGet(tPIDParam *pid) { return pid->output; }

inline float 
PIDKpGet(tPIDParam *pid) { return pid->dispKp; }						  

inline float 
PIDKiGet(tPIDParam *pid) { return pid->dispKi; }						  

inline float 
PIDKdGet(tPIDParam *pid) { return pid->dispKd; }						  

inline ePIDMode 
PIDModeGet(tPIDParam *pid) { return pid->mode; }						  

inline ePIDDirection 
PIDDirectionGet(tPIDParam *pid) { return pid->controllerDirection; }		


// 
// End of C Binding
// 
#ifdef __cplusplus
}
#endif

#endif  // PID_CONTROLLER_H
