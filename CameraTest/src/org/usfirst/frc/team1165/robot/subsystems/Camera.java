package org.usfirst.frc.team1165.robot.subsystems;

import java.io.File;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;

import org.usfirst.frc.team1165.robot.RobotMap;
import org.usfirst.frc.team1165.robot.commands.ProcessCameraFrames;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Camera extends Subsystem implements Runnable
{
	public enum CameraMode { SUBSYSTEM, THREAD };

	private ArrayList<Integer> cameraSessions = new ArrayList<Integer>();
	private int currentSessionIndex;
	private boolean isSwitchSession;
	private Image frame;
	private CameraMode mode;
	
	// This file on the roboRIO file system is used to store dumps of exceptions related to the camera:
	private final static String exceptionLogFile = "/home/lvuser/data/CameraException.txt";
	
	// This file on the roboRIO file system is used to store a list of the supported video modes:
	private final static String videoModesFile = "/home/lvuser/data/NIVision_VideoModes.txt";
	
	// This file on the roboRIO file system is used to store a list of the various vision attributes:
	private final static String visionAttributesFile = "/home/lvuser/data/NIVision_Attributes.txt";
	
	// The default video mode. To see what modes are supported, load the robot code at
	// least once and look at the file indicated by videoModesFile above.
	private final static String videoMode = "640 x 480 YUY 2 30.00 fps";
	
	/**
	 * Constructor.
	 * @param mode Indicates if should run Camera as a SUBSYSTEM or a RUNNABLE
	 */
	public Camera(CameraMode mode)
	{
		this(mode, RobotMap.primaryCameraName);
	}
	
	/**
	 * Constructor.
	 * @param mode Indicates if should run Camera as a SUBSYSTEM or a RUNNABLE
	 * @param cameraNames An arbitrary number of names of cameras to support
	 */
	public Camera(CameraMode mode, String... cameraNames)
	{
		try
		{
			Files.deleteIfExists(Paths.get(exceptionLogFile));
		}
		catch (Exception ex)
		{
		}
		
		this.mode = mode;
		frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
		
		// Create a session for every provided camera name:
		for (String cameraName : cameraNames)
		{
			try
			{
				int session = NIVision.IMAQdxOpenCamera(cameraName, NIVision.IMAQdxCameraControlMode.CameraControlModeController);
				NIVision.IMAQdxSetAttributeString(session, "AcquisitionAttributes::VideoMode", videoMode);
				cameraSessions.add(session);
			}
			catch (Exception ex)
			{
				try
				{
					PrintWriter pw = new PrintWriter(exceptionLogFile);
					pw.println("Error creating session for camera " + cameraName);
					ex.printStackTrace(pw);
					pw.close();
				}
				catch (Exception ex2)
				{
					// do nothing
				}
			}
		}
		
		// Sanity check:
		if (cameraSessions.isEmpty())
		{
			try
			{
				PrintWriter pw = new PrintWriter(exceptionLogFile);
				pw.println("No camera sessions successfully created!");
				pw.close();
			}
			catch (Exception ex2)
			{
				// do nothing
			}
			
			return;
		}
		
				
		try
		{
			// Log some interesting vision processing information at /home/lvuser/data on the roboRIO file system.
			
			new File("/home/lvuser/data").mkdirs();
			
			PrintWriter pw = new PrintWriter(videoModesFile);
			NIVision.dxEnumerateVideoModesResult result = NIVision.IMAQdxEnumerateVideoModes(cameraSessions.get(0));
			pw.println("Current: \"" + result.videoModeArray[result.currentMode].Name + '"');
			pw.println();
			for (NIVision.IMAQdxEnumItem item : result.videoModeArray)
			{
				pw.println('"' + item.Name + '"');
			}
			pw.close();
			
			NIVision.IMAQdxWriteAttributes(cameraSessions.get(0), visionAttributesFile);
		}
		catch (Exception ex)
		{
			// do nothing
		}
				
		// Default to acquiring images from the primary camera:
		currentSessionIndex = 0;
		NIVision.IMAQdxConfigureGrab(cameraSessions.get(currentSessionIndex));
		NIVision.IMAQdxStartAcquisition(cameraSessions.get(currentSessionIndex));
				
		CameraServer.getInstance().setQuality(100);
		
		if (mode == CameraMode.THREAD)
		{
			new Thread(this).start();
		}
	}

	public void initDefaultCommand()
	{
		if (mode == CameraMode.SUBSYSTEM)
		{
			setDefaultCommand(new ProcessCameraFrames(this));
		}
	}

	/**
	 * Handles processing of each camera frame and switching between cameras to keep
	 * all access to camera sessions to a single thread.
	 */
	public void processFrame()
	{
		if (cameraSessions.isEmpty())
		{
			// There is nothing to do:
			return;
		}
		
		if (isSwitchSession)
		{
			isSwitchSession = false;
			
			try
			{
				NIVision.IMAQdxStopAcquisition(cameraSessions.get(currentSessionIndex++));
				if (currentSessionIndex >= cameraSessions.size())
				{
					currentSessionIndex = 0;
				}
				
				NIVision.IMAQdxConfigureGrab(cameraSessions.get(currentSessionIndex));
				NIVision.IMAQdxStartAcquisition(cameraSessions.get(currentSessionIndex));
			}
			catch (Exception ex)
			{
				try
				{
					PrintWriter pw = new PrintWriter(exceptionLogFile);
					pw.println("Error switching to camera session " + currentSessionIndex);
					ex.printStackTrace(pw);
					pw.close();
				}
				catch (Exception ex2)
				{
					// do nothing
				}

			}
			
		}
		
		try
		{
			NIVision.IMAQdxGrab(cameraSessions.get(currentSessionIndex), frame, 1);
		}
		catch (Exception ex)
		{
			try
			{
				PrintWriter pw = new PrintWriter(exceptionLogFile);
				pw.println("Error calling IMAQdxGrab");
				ex.printStackTrace(pw);
				pw.close();
			}
			catch (Exception ex2)
			{
				// do nothing
			}
		}
		//NIVision.imaqSetImageSize(frame, 320, 240);
		CameraServer.getInstance().setImage(frame);
	}
	
	/**
	 * Call to switch to next camera session when next camera frame is processed.
	 */
	public void switchSession()
	{
		isSwitchSession = true;
	}
	
	@Override
	public void run()
	{
		while (true)
		{
			processFrame();
			Timer.delay(0.020);
		}
	}
}