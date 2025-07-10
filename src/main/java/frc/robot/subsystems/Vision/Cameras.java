// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import io.github.pseudoresonance.pixy2api.links.SPILink;

/** Add your docs here. */
/*all camera methods, again never used pixyCam for 2025 season, but can be used for future reference */
public class Cameras {
    private static UsbCamera drive = null;
	private static PixyCamera pixy1 = null;

    public static void setup() {
		initDrive();
		pixy1 = new PixyCamera(new SPILink());
	}

	public static PixyCamera getPixyCamera1() {
		return pixy1;
	}

	//function used to open usb camera in robot init function in robot.java file
	//can add pixyCam or any other cameras if using
	public static void initDrive() {
		drive = CameraServer.startAutomaticCapture();
		drive.setConnectVerbose(0);
		if (drive != null) {
			drive.setResolution(320, 240);
			drive.setFPS(30);
			drive.setWhiteBalanceManual(4500);
			drive.setExposureAuto();
			drive.setBrightness(50);
		}
	}
}
