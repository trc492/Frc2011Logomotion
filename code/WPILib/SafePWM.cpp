/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "MotorSafetyHelper.h"
#include "SafePWM.h"

/**
 * Initialize a SafePWM object by setting defaults
 */
void SafePWM::InitSafePWM()
{
	m_safetyHelper = new MotorSafetyHelper(this);
	m_safetyHelper->SetSafetyEnabled(false);
}

/**
 * Constructor for a SafePWM object taking a channel number
 * @param channel The channel number to be used for the underlying PWM object
 */
SafePWM::SafePWM(UINT32 channel): PWM(channel)
{
	InitSafePWM();
}

/**
 * Constructor for a SafePWM object taking channel and slot numbers.
 * @param slot The slot number of the digital module for this PWM object
 * @param channel The channel number in the module for this PWM object
 */
SafePWM::SafePWM(UINT32 slot, UINT32 channel): PWM(slot, channel)
{
	InitSafePWM();
}

/*
 * Set the expiration time for the PWM object
 * @param timeout The timeout (in seconds) for this motor object
 */
void SafePWM::SetExpiration(float timeout)
{
	m_safetyHelper->SetExpiration(timeout);
}

/**
 * Return the expiration time for the PWM object.
 * @returns The expiration time value.
 */
float SafePWM::GetExpiration()
{
	return m_safetyHelper->GetExpiration();
}

/**
 * Check if the PWM object is currently alive or stopped due to a timeout.
 * @returns a bool value that is true if the motor has NOT timed out and should still
 * be running.
 */
bool SafePWM::IsAlive()
{
	return m_safetyHelper->IsAlive();
}

/**
 * Stop the motor associated with this PWM object.
 * This is called by the MotorSafetyHelper object when it has a timeout for this PWM and needs to
 * stop it from running.
 */
void SafePWM::StopMotor()
{
	SetRaw(kPwmDisabled);
}

/**
 * Enable/disable motor safety for this device
 * Turn on and off the motor safety option for this PWM object.
 * @param enabled True if motor safety is enforced for this object
 */
void SafePWM::SetSafetyEnabled(bool enabled)
{
	m_safetyHelper->SetSafetyEnabled(enabled);
}

/**
 * Check if motor safety is enabled for this object
 * @returns True if motor safety is enforced for this object
 */
bool SafePWM::IsSafetyEnabled()
{
	return m_safetyHelper->IsSafetyEnabled();
}

/**
 * Feed the MotorSafety timer when setting the speed.
 * This method is called by the subclass motor whenever it updates its speed, thereby reseting
 * the timeout value.
 * @param speed Value to pass to the PWM class
 */
void SafePWM::SetSpeed(float speed)
{
	PWM::SetSpeed(speed);
	m_safetyHelper->Feed();
}

