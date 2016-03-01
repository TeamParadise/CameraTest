package org.usfirst.frc.team1165.robot.subsystems;

import java.io.File;
import java.io.FileOutputStream;
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
	private ArrayList<String> cameraNames = new ArrayList<String>();
	private String desiredCamera = "none.";
	private int currentSessionIndex;
	private Image frame;
	private CameraMode mode;
	
	// Indicates to cycle to next camera in list:
	private final static String chooseNextCamera = "Next";
	
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
				this.cameraNames.add(cameraName);
			}
			catch (Exception ex)
			{
				logException("Error creating session for camera " + cameraName, ex);
			}
		}
		
		// Sanity check:
		if (cameraSessions.isEmpty())
		{
			logException("No camera sessions successfully created!", null);			
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
		desiredCamera = this.cameraNames.get(currentSessionIndex);
		startAcquisition(currentSessionIndex);
				
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
	 * Appends a message and/or exception to the exception log file.
	 * @param msg may be null
	 * @param ex may be null
	 */
	private void logException(String msg, Exception ex)
	{
		try
		{
			PrintWriter pw = new PrintWriter(new FileOutputStream(exceptionLogFile, true));
			if (null != msg)
			{
				pw.println(msg);
			}
			if (null != ex)
			{
				ex.printStackTrace(pw);
			}
			pw.close();
		}
		catch (Exception ex2)
		{
			// do nothing
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
		
		if (cameraNames.get(currentSessionIndex) != desiredCamera)
		{
			int tempSessionIndex;
			if (desiredCamera == chooseNextCamera)
			{
				tempSessionIndex = currentSessionIndex + 1;
				if (tempSessionIndex >= cameraSessions.size())
				{
					tempSessionIndex = 0;
				}
			}
			else
			{
				tempSessionIndex = cameraNames.indexOf(desiredCamera);
			}
			
			if (tempSessionIndex == -1)
			{
				logException("No camera match for " + desiredCamera + "; camera not changed", null);
				desiredCamera = cameraNames.get(currentSessionIndex);
			}
			else
			{
				boolean success = true;
				try
				{
					NIVision.IMAQdxStopAcquisition(cameraSessions.get(currentSessionIndex));
				}
				catch (Exception ex)
				{
					logException("Error stopping acquisition for session index " + currentSessionIndex, ex);
					success = false;
				}
				
				success = success && startAcquisition(tempSessionIndex);

				if (success)
				{
					// We successfully started grabbing - update the current session index:
					currentSessionIndex = tempSessionIndex;
				}
				else
				{
					logException("Unable to switch to new camera session; resuming old session index " + currentSessionIndex, null);
					
					startAcquisition(currentSessionIndex);
				}
				
				// Match the name to the camera we are grabbing from:
				desiredCamera = cameraNames.get(currentSessionIndex);
			}
		}
		
		String msg = null;
		try
		{
			msg = "Error grabbing frame for session index " + currentSessionIndex;
			NIVision.IMAQdxGrab(cameraSessions.get(currentSessionIndex), frame, 1);
			
			msg = "Error passing frame to camera server";
			CameraServer.getInstance().setImage(frame);
		}
		catch (Exception ex)
		{
			logException(msg, ex);
		}
		
	}
	
	/**
	 * Call to switch to a specific camera
	 * @param cameraName the name of the camera to switch to
	 */
	public void setCamera(String cameraName)
	{
		desiredCamera = cameraName;
	}
	
	/**
	 * Starts acquisition from the specified session index.
	 * @param sessionIndex
	 * @return true if acquisition successfully started
	 */
	private boolean startAcquisition(int sessionIndex)
	{
		String msg = null;
		try
		{
			msg = "Error configurating grab for session index " + sessionIndex;
			NIVision.IMAQdxConfigureGrab(cameraSessions.get(sessionIndex));
			
			msg = "Error starting acquisition for session index " + sessionIndex;
			NIVision.IMAQdxStartAcquisition(cameraSessions.get(sessionIndex));
			
			return true;
		}
		catch (Exception ex)
		{
			logException(msg, ex);
			return false;
		}

	}
	
	/**
	 * Call to switch to next camera session when next camera frame is processed.
	 */
	public void switchSession()
	{
		desiredCamera = chooseNextCamera;
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