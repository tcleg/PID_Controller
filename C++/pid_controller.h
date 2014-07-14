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

//*********************************************************************************
// Macros and Globals
//*********************************************************************************

typedef enum
{
    MANUAL,
    AUTOMATIC
}
PIDMode;

typedef enum
{
    DIRECT,
    REVERSE
}
PIDDirection;

//*********************************************************************************
// Class
//*********************************************************************************

class
PIDControl
{
    public:
        // 
        // Constructor
        // Description:
        //      Initializes the PIDControl instantiation. This should be called at 
        //      least once before any other PID functions are called on the 
        //      instantiation.
        // Parameters:
        //      kp - Positive P gain constant value.
        //      ki - Positive I gain constant value.
        //      kd - Positive D gain constant value.
        //      sampleTimeSeconds - Interval in seconds on which PIDCompute will be 
        //          called.
        //      minOutput - Constrain PID output to this minimum value.
        //      maxOutput - Constrain PID output to this maximum value.
        //      mode - Tells how the controller should respond if the user has 
        //          taken over manual control or not.
        //          MANUAL:    PID controller is off. User can manually control the 
        //                     output.
        //          AUTOMATIC: PID controller is on. PID controller controls the 
        //                     output.
        //      controllerDirection - The sense of direction of the controller
        //          DIRECT:  A positive setpoint gives a positive output.
        //          REVERSE: A positive setpoint gives a negative output.
        // Returns:
        //      Nothing.
        // 
        PIDControl(float kp, float ki, float kd, float sampleTimeSeconds, 
                   float minOutput, float maxOutput, PIDMode mode, 
                   PIDDirection controllerDirection);     	
        
        // 
        // PID Compute
        // Description:
        //      Should be called on a regular interval defined by sampleTimeSeconds.
        //      Typically, PIDSetpointSet and PIDInputSet should be called 
        //      immediately before PIDCompute.
        // Parameters:
        //      None.
        // Returns:
        //      True if in AUTOMATIC. False if in MANUAL.
        //                     
        bool PIDCompute(); 
        
        // 
        // PID Mode Set
        // Description:
        //      Sets the PID controller to a new mode. Tells how the controller 
        //      should respond if the user has taken over manual control or not.
        // Parameters:
        //      mode - 
        //          MANUAL:    PID controller is off. User can manually control the 
        //                     output.
        //          AUTOMATIC: PID controller is on. PID controller controls the 
        //                     output.
        // Returns:
        //      Nothing.
        //              
        void PIDModeSet(PIDMode mode);                                                                                                                                       
        
        // 
        // PID Output Limits Set
        // Description:
        //      Sets the new output limits. The new limits are applied to the PID
        //      immediately.
        // Parameters:
        //      min - Constrain PID output to this minimum value.
        //      max - Constrain PID output to this maximum value.
        // Returns:
        //      Nothing.
        // 
        void PIDOutputLimitsSet(float min, float max); 							  							  
        
        // 
        // PID Tunings Set
        // Description:
        //      Sets the new gain constant values.
        // Parameters:
        //      kp - Positive P gain constant value.
        //      ki - Positive I gain constant value.
        //      kd - Positive D gain constant value.
        // Returns:
        //      Nothing.
        // 
        void PIDTuningsSet(float kp, float ki, float kd);         	                                         
        
        // 
        // PID Tuning Gain Constant P Set
        // Description:
        //      Sets the proportional gain constant value.
        // Parameters:
        //      kp - Positive P gain constant value.
        // Returns:
        //      Nothing.
        // 
        void PIDTuningKpSet(float kp);
        
        // 
        // PID Tuning Gain Constant I Set
        // Description:
        //      Sets the proportional gain constant value.
        // Parameters:
        //      ki - Positive I gain constant value.
        // Returns:
        //      Nothing.
        // 
        void PIDTuningKiSet(float ki);
        
        // 
        // PID Tuning Gain Constant D Set
        // Description:
        //      Sets the proportional gain constant value.
        // Parameters:
        //      kd - Positive D gain constant value.
        // Returns:
        //      Nothing.
        // 
        void PIDTuningKdSet(float kd);
        
        // 
        // PID Controller Direction Set
        // Description:
        //      Sets the new controller direction.
        // Parameters:
        //      controllerDirection - The sense of direction of the controller
        //          DIRECT:  A positive setpoint gives a positive output
        //          REVERSE: A positive setpoint gives a negative output
        // Returns:
        //      Nothing.
        // 
        void PIDControllerDirectionSet(PIDDirection controllerDirection);	  									  									  									  
        
        // 
        // PID Sample Time Set
        // Description:
        //      Sets the new sampling time (in seconds).
        // Parameters:
        //      sampleTimeSeconds - Interval in seconds on which PIDCompute will be 
        //          called.
        // Returns:
        //      Nothing.
        // 
        void PIDSampleTimeSet(float sampleTimeSeconds);                                                       									  									  									   
        
        // 
        // PID Setpoint Set
        // Description:
        //      Alters the setpoint the PID controller will try to achieve.
        // Parameters:
        //      setpoint - The desired setpoint the PID controller will try to 
        //          obtain.
        // Returns:
        //      Nothing.
        // 
        inline void PIDSetpointSet(float setpoint) { pid->setpoint = setpoint; }
        
        // 
        // PID Input Set
        // Description:
        //      Should be called before calling PIDCompute so the PID controller 
        //      will have an updated input value to work with.
        // Parameters:
        //      input - The value the controller will work with.
        // Returns:
        //      Nothing.
        // 
        inline void PIDInputSet(float input) { pid->input = input; }
        
        // 
        // PID Output Get
        // Description:
        //      Typically, this function is called after PIDCompute in order to
        //      retrieve the output of the controller.
        // Parameters:
        //      None.
        // Returns:
        //      The output of the specific PID controller.
        // 
        inline float PIDOutputGet() { return pid->output; }
        
        // 
        // PID Proportional Gain Constant Get
        // Description:
        //      Returns the proportional gain constant value the particular
        //      controller is set to.
        // Parameters:
        //      None.
        // Returns:
        //      The proportional gain constant.
        // 
        inline float PIDKpGet() { return pid->dispKp; }						  
        
        // 
        // PID Integral Gain Constant Get
        // Description:
        //      Returns the integral gain constant value the particular
        //      controller is set to.
        // Parameters:
        //      None.
        // Returns:
        //      The integral gain constant.
        // 
        inline float PIDKiGet() { return pid->dispKi; }						  
        
        // 
        // PID Derivative Gain Constant Get
        // Description:
        //      Returns the derivative gain constant value the particular
        //      controller is set to.
        // Parameters:
        //      None.
        // Returns:
        //      The derivative gain constant.
        // 
        inline float PIDKdGet() { return pid->dispKd; }						  
        
        // 
        // PID Mode Get
        // Description:
        //      Returns the mode the particular controller is set to.
        // Parameters:
        //      None.
        // Returns:
        //      MANUAL or AUTOMATIC depending on what the user set the 
        //      controller to.
        // 
        inline PIDMode PIDModeGet() { return pid->mode; }						  
        
        // 
        // PID Direction Get
        // Description:
        //      Returns the direction the particular controller is set to.
        // Parameters:
        //      None.
        // Returns:
        //      DIRECT or REVERSE depending on what the user set the
        //      controller to.
        // 
        inline PIDDirection PIDDirectionGet() { return pid->controllerDirection; }
        
    private:
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
        PIDDirection controllerDirection;
        
        // 
        // Tells how the controller should respond if the user has
        // taken over manual control or not
        // MANUAL:    PID controller is off.
        // AUTOMATIC: PID controller is on.
        // 
        PIDMode mode;
}

#endif  // PID_CONTROLLER_H
