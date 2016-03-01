package org.usfirst.frc.team1165.robot;

import org.usfirst.frc.team1165.robot.commands.SwitchToNextCamera;
import org.usfirst.frc.team1165.robot.commands.SwitchToSpecifiedCamera;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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
		cameraButton.whenPressed(new SwitchToNextCamera());
		
		SmartDashboard.putData(new SwitchToNextCamera());
		SmartDashboard.putData("cam6", new SwitchToSpecifiedCamera("cam6"));
		SmartDashboard.putData("cam7", new SwitchToSpecifiedCamera("cam7"));
	}
	
	public boolean useSecondaryCamera()
	{
		return cameraButton.get();
	}
}
