package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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

    public static class AIO
    {
        public static final int MODULE_FL_SENSOR   = 0;
        public static final int MODULE_FR_SENSOR   = 1;
        public static final int MODULE_BL_SENSOR   = 2;
        public static final int MODULE_BR_SENSOR   = 3;
        public static final int CLIMB_LEFT_SENSOR  = 4;
        public static final int CLIMB_RIGHT_SENSOR = 5;
    }

    public static enum AUTOPOSES
    {
        // @formatter: off
        AMP(new Pose2d(15.24, 6.77, Rotation2d.fromDegrees(169.81)), new Pose2d(0.76, 6.77, Rotation2d.fromDegrees(10.19))), MIDDLE(new Pose2d(14.61, 5.56, Rotation2d.fromDegrees(180)), new Pose2d(1.39, 5.56, new Rotation2d())),
        SOURCE(new Pose2d(15.21, 4.23, Rotation2d.fromDegrees(-155.56)), new Pose2d(0.79, 4.23, Rotation2d.fromDegrees(-24.44)));
        // @formatter: on

        Pose2d _redPose;
        Pose2d _bluePose;

        private AUTOPOSES(Pose2d redPose, Pose2d bluePose)
        {
            _redPose  = redPose;
            _bluePose = bluePose;
        }

        public Pose2d getPose()
        {
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) return _redPose;
            return _bluePose;
        }
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
        public static final int SHOOTER_FLYWHEEL_LOWER = 15;
        public static final int SHOOTER_FLYWHEEL_UPPER = 16;
        public static final int CLIMB_LEFT             = 17;
        public static final int CLIMB_RIGHT            = 18;
    }

    public static class Characterization
    {
        public static final double START_DELAY_SECS   = 2.0;
        public static final double RAMP_VOLTS_PER_SEC = 0.1;
    }

    public static class Climb
    {
        public static final double MIN_EXTENSION       = 0.0;
        public static final double LEFT_MAX_EXTENSION  = 17.375;
        public static final double RIGHT_MAX_EXTENSION = 17.5;
        public static final double LEFT_ZERO_OFFSET    = -17.403;
        public static final double RIGHT_ZERO_OFFSET   = 0.3;
        public static final double LEFT_SENSOR_SCALE   = -27.234;
        public static final double RIGHT_SENSOR_SCALE  = 27.301;
    }

    public static class Controls
    {
        public static final double JOYSTICK_DEADBAND        = 0.1;
        public static final double ROTATE_JOYSTICK_DEADBAND = 0.15;
    }

    public static class DIO
    {
        public static final int SHOOTER_BED_SENSOR = 0;
        public static final int NOTE_SENSOR        = 1;
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

    public static class Intake
    {
        public static final double INTAKE_DEFAULT_PERCENT_OUTPUT  = 0.8;
        public static final double OUTTAKE_DEFAULT_PERCENT_OUTPUT = 0.6;
    }

    public static class LED
    {
        public static final int   NUM_LEDS    = 6; // TODO: Change value
        public static final Color RED         = new Color(255, 0, 0);
        public static final Color BLUE        = new Color(0, 0, 255);
        public static final Color ORANGE      = new Color(255, 50, 0);
        public static final Color PURPLE      = new Color(127, 0, 255);
        public static final Color GREEN       = new Color(0, 115, 0);
        public static final Color PINK        = new Color(255, 46, 204);
        public static final Color TEAL        = new Color(0, 70, 139);
        public static final Color WHITE       = new Color(255, 255, 255);
        public static final Color OFF         = new Color(0, 0, 0);
        public static final Color SIGIS_COLOR = new Color(123, 65, 239);
        public static final Color GREY        = new Color(100, 100, 100);
        public static final Color ZOESCOLOR   = new Color(25, 120, 100);
        public static final Color RONISCOLOR  = new Color(27, 44, 129);
    }

    public static class Notepath
    {
        public static final double NOTEPATH_INTAKE_PICKUP_PERCENT_OUTPUT  = 0.15;
        public static final double NOTEPATH_FEED_PERCENT_OUTPUT           = 0.6;
        public static final double NOTEPATH_SHOOTER_PICKUP_PERCENT_OUTPUT = 0.3;
    }

    public static class PathPlanner
    {
        public static final double                      INTAKE_DELAY       = 4.5; // TODO: Change value
        public static final Translation2d               flModuleOffset     = new Translation2d(0.4, -0.4);
        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(5.0, 0, 0), // Translation constants
                new PIDConstants(5.0, 0, 0), // Rotation constants
                Constants.General.MAX_KRAKEN_SPEED, flModuleOffset.getNorm(), // Drive base radius (distance from center to furthest module)
                new ReplanningConfig()
        );
    }

    public static class ShooterBed // FIXME: Update all these values
    {
        public static final double     MAX_BED_VOLTS            = 6.0;
        public static final double     BED_SCALE                = 183.21; // 170.401;
        public static final Rotation2d BED_ANGLE_OFFSET         = Rotation2d.fromDegrees(6.05 /* 59.947 */);
        public static final Rotation2d MAX_BED_ANGLE            = Rotation2d.fromDegrees(96);
        public static final Rotation2d MIN_BED_ANGLE            = Rotation2d.fromDegrees(24);
        public static final Rotation2d BED_INTAKE_PICKUP_ANGLE  = Rotation2d.fromDegrees(65.1);
        public static final Rotation2d BED_SHOOTER_PICKUP_ANGLE = Rotation2d.fromDegrees(52.8);
        public static final Rotation2d BED_SUBWOOFER_SHOT_ANGLE = Rotation2d.fromDegrees(54.5);
        public static final Rotation2d BED_PODIUM_SHOT_ANGLE    = Rotation2d.fromDegrees(31);
        public static final Rotation2d BED_AMP_SHOT_ANGLE       = Rotation2d.fromDegrees(50);
        public static final Rotation2d Bed_TRAP_SHOT_ANGLE      = Rotation2d.fromDegrees(53.5);
        public static final Rotation2d BED_STOW_ANGLE           = Rotation2d.fromDegrees(37);
        public static final Rotation2d BED_CLIMB_VERTICAL_ANGLE = MAX_BED_ANGLE.minus(Rotation2d.fromDegrees(2));
        public static final double     BED_DOWN_MIN_VOLTS       = -3.5;// -0.5;
        public static final double     BED_UP_MIN_VOLTS         = -2.3;// 0.11;
    }

    public static class ShooterFlywheel
    {
        public static final double MAX_FLYWHEEL_SPEED    = 1.0;
        public static final double FLYWHEEL_INTAKE_SPEED = 0.15;
        public static final double VELOCITY_RANGE        = 0.15;
    }

    public static class Vision
    {
        public static final String      PHOTON_CAMERA_NAME     = "photonCam";
        public static final String      SHOOTER_CAMERA_NAME    = "shooterCam";
        public static final String      DRIVER_CAMERA_NAME     = "driveCam";
        public static final Rotation3d  CAMERA_ROTATION        = new Rotation3d(0.0, Units.degreesToRadians(-33), Units.degreesToRadians(180));
        public static final Transform3d CAMERA_TRANSFORM       = new Transform3d(Units.inchesToMeters(-7), Units.inchesToMeters(7), Units.inchesToMeters(9.5), CAMERA_ROTATION);
        public static final String      PHOTON_CAMERA_URL      = "http://10.5.25.12:1181/?action=stream";
        public static final double      MAX_DETECTION_RANGE    = Units.inchesToMeters(300);
        public static final boolean     ENABLE_POSE_CORRECTION = false;
        public static final double      ALIGNMENT_OFFSET       = -3;
    }
}
