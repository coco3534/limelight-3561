// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.links.Link;

/** Add your docs here. */
/*pixyCam methods, never used for 2025 season, but can be used for future reference */
public class PixyCamera {
    private static Pixy2 pixy;

    public PixyCamera(Link link) {
		pixy = Pixy2.createInstance(link);
		pixy.init();
	}

	public PixyCamera(Link link, int arg) {
		pixy = Pixy2.createInstance(link);
		pixy.init(arg);
	}

    public Pixy2 getPixy() {
		return pixy;
	}

}
