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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public final class Constants
{
    // Private constructor to prevent instantiation
    private Constants()
    {
    }

    public static class CAN
    {
        public static final int MODULE_FL_DRIVE        = 1;
        public static final int MODULE_FL_ROTATE       = 2;
        public static final int MODULE_FR_DRIVE        = 3;
        public static final int MODULE_FR_ROTATE       = 4;
        public static final int MODULE_BL_DRIVE        = 5;
        public static final int MODULE_BL_ROTATE       = 6;
        public static final int MODULE_BR_DRIVE        = 7;
        public static final int MODULE_BR_ROTATE       = 8;
        public static final int INTAKE                 = 9;
        public static final int NOTEPATH_LEADER        = 11;
        public static final int NOTEPATH_FOLLOWER      = 12;
        public static final int SHOOTER_BED_LEADER     = 13;
        public static final int SHOOTER_BED_FOLLOWER   = 14;
        public static final int SHOOTER_FLYWHEEL_UPPER = 15;
        public static final int SHOOTER_FLYWHEEL_LOWER = 16;
    }

    public static class AIO
    {
        public static final int MODULE_FL_SENSOR   = 0;
        public static final int MODULE_FR_SENSOR   = 1;
        public static final int MODULE_BL_SENSOR   = 2;
        public static final int MODULE_BR_SENSOR   = 3;
        public static final int SHOOTER_BED_SENSOR = 4;
    }

    public static class Controls
    {
        public static final double JOYSTICK_DEADBAND = 0.1;
    }

    public static class Characterization
    {
        public static final double START_DELAY_SECS   = 2.0;
        public static final double RAMP_VOLTS_PER_SEC = 0.1;
    }

    public static class Intake
    {
        public static final double INTAKE_DEFAULT_PERCENT_OUTPUT  = 0.8;
        public static final double OUTTAKE_DEFAULT_PERCENT_OUTPUT = 0.3;
    }

    public static class Drive
    {
        public static final double     MAX_LINEAR_SPEED   = Units.feetToMeters(13.5);
        public static final double     TRACK_WIDTH_X      = Units.inchesToMeters(20.0);
        public static final double     TRACK_WIDTH_Y      = Units.inchesToMeters(28.5);
        public static final double     DRIVE_BASE_RADIUS  = Math.hypot(TRACK_WIDTH_X / 2, TRACK_WIDTH_Y / 2);
        public static final double     MAX_ANGULAR_SPEED  = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
        public static final double     WHEEL_RADIUS       = Units.inchesToMeters(2.0);
        public static final double     DRIVE_GEAR_RATIO   = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
        public static final double     TURN_GEAR_RATIO    = (150.0 / 7.0);
        public static final double     HALL_EFFECT_SCALE  = 360 / 0.92;
        public static final double     HALL_EFFECT_OFFSET = (360 - HALL_EFFECT_SCALE) / 2.0;
        public static final Rotation2d MODULE_FL_OFFSET   = Rotation2d.fromRadians(-2.49);
        public static final Rotation2d MODULE_FR_OFFSET   = Rotation2d.fromRadians(2.09);
        public static final Rotation2d MODULE_BL_OFFSET   = Rotation2d.fromRadians(-2.33);
        public static final Rotation2d MODULE_BR_OFFSET   = Rotation2d.fromRadians(-1.63);
    }

    public static class ShooterBed // FIXME: Update all these values
    {
        public static final Rotation2d BED_ANGLE_OFFSET = Rotation2d.fromRadians(0);
        public static final Rotation2d MAX_BED_ANGLE    = Rotation2d.fromRadians(Math.PI / 2);
        public static final Rotation2d MIN_BED_ANGLE    = Rotation2d.fromRadians(0);
    }

    public static class ShooterFlywheel
    {
        public static final double MAX_FLYWHEEL_SPEED = 1000000;
    }

    public static class Notepath
    {
        public static final double NOTEPATH_INTAKE_PICKUP_PERCENT_OUTPUT  = 0.6;
        public static final double NOTEPATH_FEED_PERCENT_OUTPUT           = 0.4;
        public static final double NOTEPATH_SHOOTER_PICKUP_PERCENT_OUTPUT = 0.5;
    }

    public static class General
    {
        public static final double LOOP_PERIOD_SECS = 0.02;
        public static final double MOTOR_VOLTAGE    = 12.0;
    }

    public static class AdvantageKit
    {
        public static final Mode CURRENT_MODE = Mode.REAL;

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
}
