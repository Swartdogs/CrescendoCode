// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants
{
    public static final Mode currentMode = Mode.REAL;

    public static class Vision
    {
        public static final String CAMERA_NAME = "frontCam";
        // Measurement from the camera to the center of the robot
        public static final int PARAMETER_X = (int)Units.inchesToMeters(12);
        // Measurement from the camera to the side of the robot
        public static final int PARAMETER_Y = (int)Units.inchesToMeters(0);
        // Measurement from the ground to the camera's center
        public static final double     PARAMETER_Z        = Units.inchesToMeters(5.25);
        public static final Rotation3d PARAMETER_ROTATION = new Rotation3d(0.0, 0.0, 0.0);
    }

    public static enum Mode
    {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }
}
