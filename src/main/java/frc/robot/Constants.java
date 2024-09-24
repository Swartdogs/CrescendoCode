package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public final class Constants
{
    // Private constructor to prevent instantiation
    private Constants()
    {
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

    public static class RobotConfig
    {
        public static final Robots CURRENT_ROBOT = Robots.GRASSHOPPER;

        public static enum Robots
        {
            GRASSHOPPER(15, 60), MELMAN(30, 20);

            public double TRACK_WIDTH;
            public double WHEEL_BASE;

            private Robots(double tw, double wb)
            {
                TRACK_WIDTH = tw;
                WHEEL_BASE  = wb;
            }
        }
    }

    public static class AIO
    {
        public static final int MODULE_FL_SENSOR = 0;
        public static final int MODULE_FR_SENSOR = 1;
        public static final int MODULE_BL_SENSOR = 2;
        public static final int MODULE_BR_SENSOR = 3;
    }

    public static class CAN
    {
        public static final int MODULE_FL_DRIVE  = 1;
        public static final int MODULE_FL_ROTATE = 2;
        public static final int MODULE_FR_DRIVE  = 3;
        public static final int MODULE_FR_ROTATE = 4;
        public static final int MODULE_BL_DRIVE  = 5;
        public static final int MODULE_BL_ROTATE = 6;
        public static final int MODULE_BR_DRIVE  = 7;
        public static final int MODULE_BR_ROTATE = 8;
    }

    public static class Characterization
    {
        public static final double START_DELAY_SECS   = 2.0;
        public static final double RAMP_VOLTS_PER_SEC = 0.1;
    }

    public static class Controls
    {
        public static final double JOYSTICK_DEADBAND        = 0.1;
        public static final double ROTATE_JOYSTICK_DEADBAND = 0.15;
    }

    public static class Drive
    {
        public static final double     TRACK_WIDTH_X     = Units.inchesToMeters(20.0);
        public static final double     TRACK_WIDTH_Y     = Units.inchesToMeters(28.5);
        public static final double     WHEEL_RADIUS      = Units.inchesToMeters(2.0);
        public static final double     DRIVE_GEAR_RATIO  = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
        public static final double     TURN_GEAR_RATIO   = (150.0 / 7.0);
        public static final double     MAX_LINEAR_SPEED  = Units.feetToMeters(13.86);
        public static final double     DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2, TRACK_WIDTH_Y / 2);
        public static final double     MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
        public static final Rotation2d MODULE_FL_OFFSET  = Rotation2d.fromRadians(-2.49).plus(Rotation2d.fromDegrees(180));
        public static final Rotation2d MODULE_FR_OFFSET  = Rotation2d.fromRadians(2.09).plus(Rotation2d.fromDegrees(180));
        public static final Rotation2d MODULE_BL_OFFSET  = Rotation2d.fromRadians(-2.33).plus(Rotation2d.fromDegrees(180));
        public static final Rotation2d MODULE_BR_OFFSET  = Rotation2d.fromRadians(1.81);
    }

    public static class Field
    {
        public static final Pose2d BLUE_SPEAKER     = new Pose2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42), new Rotation2d());
        public static final Pose2d BLUE_AMP         = new Pose2d(Units.inchesToMeters(72.5), Units.inchesToMeters(323.0), Rotation2d.fromDegrees(270));
        public static final Pose2d RED_SPEAKER      = new Pose2d(Units.inchesToMeters(625.7), Units.inchesToMeters(218.42), new Rotation2d());
        public static final Pose2d RED_AMP          = new Pose2d(Units.inchesToMeters(578.77), Units.inchesToMeters(323.0), Rotation2d.fromDegrees(270));
        public static final Pose2d BLUE_SOURCE      = new Pose2d(Units.inchesToMeters(615.445), Units.inchesToMeters(22.235), Rotation2d.fromDegrees(120));
        public static final Pose2d BLUE_STAGE_ONE   = new Pose2d(Units.inchesToMeters(173.73), Units.inchesToMeters(192.69), Rotation2d.fromDegrees(120));
        public static final Pose2d BLUE_STAGE_TWO   = new Pose2d(Units.inchesToMeters(173.73), Units.inchesToMeters(130.6), Rotation2d.fromDegrees(240));
        public static final Pose2d BLUE_STAGE_THREE = new Pose2d(Units.inchesToMeters(227.48), Units.inchesToMeters(161.62), Rotation2d.fromDegrees(0));
        public static final Pose2d BLUE_PODIUM      = new Pose2d(Units.inchesToMeters(128.48), Units.inchesToMeters(161.62), Rotation2d.fromDegrees(0));
        public static final Pose2d RED_SOURCE       = new Pose2d(Units.inchesToMeters(35.78), Units.inchesToMeters(22.235), Rotation2d.fromDegrees(60));
        public static final Pose2d RED_STAGE_ONE    = new Pose2d(Units.inchesToMeters(477.69), Units.inchesToMeters(130.6), Rotation2d.fromDegrees(300));
        public static final Pose2d RED_STAGE_TWO    = new Pose2d(Units.inchesToMeters(477.69), Units.inchesToMeters(192.69), Rotation2d.fromDegrees(60));
        public static final Pose2d RED_STAGE_THREE  = new Pose2d(Units.inchesToMeters(423.74), Units.inchesToMeters(161.62), Rotation2d.fromDegrees(180));
        public static final Pose2d RED_PODIUM       = new Pose2d(Units.inchesToMeters(522.74), Units.inchesToMeters(161.62), Rotation2d.fromDegrees(180));
    }

    public static class General
    {
        public static final double LOOP_PERIOD_SECS = 0.02;
        public static final double MOTOR_VOLTAGE    = 12.0;
        public static final double MAX_NEO_SPEED    = 5874;
        public static final double MAX_KRAKEN_SPEED = 6000;
    }
}
