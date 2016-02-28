package org.usfirst.frc.team1165.robot;

import org.usfirst.frc.team1165.robot.commands.SwitchCamera;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI
{
	public final Joystick leftStick = new Joystick(0);
	public final JoystickButton cameraButton = new JoystickButton(leftStick,2);
	public OI()
	{
		cameraButton.whenPressed(new SwitchCamera());
	}
	
	public boolean useSecondaryCamera()
	{
		return cameraButton.get();
	}
}
