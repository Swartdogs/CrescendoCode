package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.util.Utilities;

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

    public static Command joystickDrive(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, BooleanSupplier robotCentric, Dashboard dashboard)
    {
        return joystickDrive(drive, xSupplier, ySupplier, omegaSupplier, robotCentric, 2, 3, dashboard);
    }

    public static Command joystickDrive(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, BooleanSupplier robotCentric, int translateExponent, double rotateExponent, Dashboard dashboard)
    {
        return Commands.run(() ->
        {
            // Apply deadband
            double     linearMagnitude = MathUtil.applyDeadband(Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), Constants.Controls.JOYSTICK_DEADBAND);
            Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
            double     omega           = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), Constants.Controls.JOYSTICK_DEADBAND);

            // Square values
            linearMagnitude = Math.pow(linearMagnitude, translateExponent);
            omega           = Math.copySign(Math.pow(Math.abs(omega), rotateExponent), omega);

            // Calcaulate new linear velocity
            Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection).transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();

            // Convert to field relative speeds & send command
            if (robotCentric.getAsBoolean())
            {
                var chassisSpeeds = new ChassisSpeeds(linearVelocity.getX() * Constants.Drive.MAX_LINEAR_SPEED, linearVelocity.getY() * Constants.Drive.MAX_LINEAR_SPEED, omega * Constants.Drive.MAX_ANGULAR_SPEED);

                if (!dashboard.isDriverCamera())
                {
                    chassisSpeeds = new ChassisSpeeds(-chassisSpeeds.vxMetersPerSecond, -chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
                }

                drive.runVelocity(chassisSpeeds);
            }
            else
            {
                if (!Utilities.isBlueAlliance())
                {
                    linearVelocity = linearVelocity.unaryMinus();
                }

                drive.runVelocity(
                        ChassisSpeeds
                                .fromFieldRelativeSpeeds(linearVelocity.getX() * Constants.Drive.MAX_LINEAR_SPEED, linearVelocity.getY() * Constants.Drive.MAX_LINEAR_SPEED, omega * Constants.Drive.MAX_ANGULAR_SPEED, drive.getRotation())
                );
            }

        }, drive);
    }

    public static Command driveAtOrientation(Drive drive, Dashboard dashboard, DoubleSupplier xSupplier, DoubleSupplier ySupplier, BooleanSupplier robotCentric, double setpoint, double maxSpeed)
    {
        return Commands.runOnce(() -> drive.rotateInit(setpoint, maxSpeed)).andThen(joystickDrive(drive, xSupplier, ySupplier, () -> drive.rotateExecute(), robotCentric, 2, 1, dashboard));
    }

    public static Command aimAtSpeaker(Drive drive, Dashboard dashboard, DoubleSupplier xSupplier, DoubleSupplier ySupplier, BooleanSupplier robotCentric, double maxSpeed)
    {
        return Commands.either(
                Commands.runOnce(() -> drive.rotateInit(getHeadingToPose(drive, Constants.Field.BLUE_SPEAKER), maxSpeed))
                        .andThen(joystickDrive(drive, xSupplier, ySupplier, () -> drive.rotateExecute(getHeadingToPose(drive, Constants.Field.BLUE_SPEAKER)), robotCentric, 2, 1, dashboard)), // blue
                                                                                                                                                                                               // alliance
                Commands.runOnce(() -> drive.rotateInit(getHeadingToPose(drive, Constants.Field.RED_SPEAKER), maxSpeed))
                        .andThen(joystickDrive(drive, xSupplier, ySupplier, () -> drive.rotateExecute(getHeadingToPose(drive, Constants.Field.RED_SPEAKER)), robotCentric, 2, 1, dashboard)), // red alliance
                () -> Utilities.isBlueAlliance()
        );

    }

    public static Command aimAtAmp(Drive drive, Dashboard dashboard, DoubleSupplier xSupplier, DoubleSupplier ySupplier, double maxSpeed)
    {
        return Commands.either(
                Commands.runOnce(() -> drive.rotateInit(getHeadingToPose(drive, Utilities.getAutoPose(Constants.Field.BLUE_AMP)), maxSpeed))
                        .andThen(joystickDrive(drive, xSupplier, ySupplier, () -> drive.rotateExecute(getHeadingToPose(drive, Constants.Field.BLUE_AMP)), () -> false, 2, 1, dashboard)),
                Commands.runOnce(() -> drive.rotateInit(getHeadingToPose(drive, Utilities.getAutoPose(Constants.Field.BLUE_AMP)), maxSpeed))
                        .andThen(joystickDrive(drive, xSupplier, ySupplier, () -> drive.rotateExecute(getHeadingToPose(drive, Constants.Field.RED_AMP)), () -> false, 2, 1, dashboard)),
                () -> Utilities.isBlueAlliance()
        );
    }

    private static double getHeadingToPose(Drive drive, Pose2d pose)
    {
        double deltaY = pose.getY() - drive.getPose().getY();
        double deltaX = pose.getX() - drive.getPose().getX();

        return Units.radiansToDegrees(Math.atan2(deltaY, deltaX)) + 180;
    }

    public static Command resetGyro(Drive drive, Gyro gyro)
    {
        return Commands.runOnce(() ->
        {
            var pose = drive.getPose();
            drive.setPose(new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(Utilities.isBlueAlliance() ? 0 : 180)));
        }).ignoringDisable(true);
    }

    public static Command driveVolts(Drive drive, double volts)
    {
        return Commands.runOnce(() -> drive.runVolts(volts));
    }

    public static Command reduceSpeed(Drive drive)
    {
        return Commands.startEnd(() -> drive.setSpeedMultipler(0.2), () -> drive.setSpeedMultipler(1));
    }
}
