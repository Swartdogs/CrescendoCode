package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.gyro.Gyro;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public final class DriveCommands
{
    private DriveCommands()
    {
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and
     * angular velocities).
     */
    public static Command joystickDrive(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, BooleanSupplier robotCentric)
    {
        return Commands.run(() ->
        {
            // Apply deadband
            double     linearMagnitude = MathUtil.applyDeadband(Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), Constants.Controls.JOYSTICK_DEADBAND);
            Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
            double     omega           = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), Constants.Controls.JOYSTICK_DEADBAND);

            // Square values
            linearMagnitude = linearMagnitude * linearMagnitude;
            omega           = Math.copySign(omega * omega, omega);

            Rotation2d allianceAdjustment = Rotation2d.fromDegrees(0);

            Optional<Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
            {
                allianceAdjustment = Rotation2d.fromDegrees(180);
            }

            // Calcaulate new linear velocity
            Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection).transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();

            // Convert to field relative speeds & send command
            if (robotCentric.getAsBoolean())
            {
                System.out.println("Robot centric");
                drive.runVelocity(
                        ChassisSpeeds.fromRobotRelativeSpeeds(
                                linearVelocity.getX() * Constants.Drive.MAX_LINEAR_SPEED, linearVelocity.getY() * Constants.Drive.MAX_LINEAR_SPEED, omega * Constants.Drive.MAX_ANGULAR_SPEED, drive.getRotation().div(2)
                        )
                );
            }
            else
            {
                System.out.println("Field centric");
                drive.runVelocity(
                        ChassisSpeeds
                                .fromFieldRelativeSpeeds(linearVelocity.getX() * Constants.Drive.MAX_LINEAR_SPEED, linearVelocity.getY() * Constants.Drive.MAX_LINEAR_SPEED, omega * Constants.Drive.MAX_ANGULAR_SPEED, drive.getRotation())
                );
            }

        }, drive);
    }

    public static Command driveAtOrientation(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, double setpoint, double maxSpeed)
    {
        return Commands.runOnce(() -> drive.rotateInit(setpoint, maxSpeed)).andThen(joystickDrive(drive, xSupplier, ySupplier, () -> drive.rotateExecute(), () -> false));
    }

    public static Command aimAtSpeaker(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, double maxSpeed)
    {
        return Commands.runOnce(() -> drive.rotateInit(getHeadingToPose(drive, Constants.Field.BLUE_SPEAKER), maxSpeed))
                .andThen(joystickDrive(drive, xSupplier, ySupplier, () -> drive.rotateExecute(getHeadingToPose(drive, Constants.Field.BLUE_SPEAKER)), () -> false));
    }

    public static Command aimAtAmp(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, double maxSpeed)
    {
        return Commands.runOnce(() -> drive.rotateInit(getHeadingToPose(drive, Constants.Field.BLUE_AMP), maxSpeed))
                .andThen(joystickDrive(drive, xSupplier, ySupplier, () -> drive.rotateExecute(getHeadingToPose(drive, Constants.Field.BLUE_AMP)), () -> false));
    }

    private static double getHeadingToPose(Drive drive, Pose2d pose)
    {
        double deltaY = pose.getY() - drive.getPose().getY();
        double deltaX = pose.getX() - drive.getPose().getX();

        return Math.atan2(deltaY, deltaX) / Math.PI * 180;
    }

    public static Command resetGyro(Drive drive, Gyro gyro)
    {
        return Commands.runOnce(() -> drive.setPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))).ignoringDisable(true);
    }

    public static Command driveVolts(Drive drive, double volts)
    {
        return Commands.runOnce(() -> drive.runVolts(volts));
    }
}
