#include "Control.h"
#include "MotorControl.h"
#include "MotorEncoders.h"
#include "retarget.h"
#include "ComplementaryFilter.h"

#define kP 180
#define kD 2.5




#define velGain 0.10 // 0.08
#define velIntegralGain 0.002
#define maxSetPoint 9



int16_t computePID(float angle, float velocity, float position, float dt)
{
	static float lastError = 0;
	float derivative = 0;
	
	// compute target velocity
	float targetVelocity = - position * 0.6;
	
	// will be calculated based on velocity
	float setPoint = 0;
	float velError = 0;
	static float 	velIntegral = 0;
	
	// compute velocity error
	velError = targetVelocity - velocity;
	
	// update the integral term  
	velIntegral = velIntegral + velError * dt;
	
	if ((velIntegral + velError * dt)*velIntegralGain < maxSetPoint && (velIntegral + velError * dt)*velIntegralGain > -maxSetPoint)
		velIntegral += velError * dt;
	
	printf("velIntegral:%f, velError:%f\r\n", velIntegral, velError);
	// compute final setpoint
	setPoint = velError * velGain + velIntegral * velIntegralGain;

	// constraint set point 
	if(setPoint > maxSetPoint)
		setPoint = maxSetPoint;
	else if(setPoint <- maxSetPoint)
		setPoint = -maxSetPoint;
	
	// stabilizing inner loop 
	
	float error = setPoint - angle;
	// PD output 
	float motorTorque;
	float u;

	// update derivative term
	derivative = (error - lastError)/dt;
	lastError = error;
	
	// compute PID command (motor torque)
	motorTorque =  error * kP + derivative * kD;
	
	// u <- motor voltage, motor neads to spin in oposite direction of desired torque
	u = -motorTorque;
	
	// clamp command value to avoid overflow
	if (u > maxPulse)
		u = maxPulse;
	else if (u < - maxPulse)
		u = -maxPulse;
	
	return u;
}


void stabilizeMotorOutput(float pwm, float dt)
{
	// velocity difference 
	float error =  (velocityLeft - velocityRight);
	static float integral = 0;
	
	integral += error *dt;
	
	float u = integral*0.005 + error*2;
	
	setRightMotorDutyCycle(pwm+u);
	setLeftMotorDutyCycle(pwm-u);

}
