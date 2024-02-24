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

import java.util.Optional;
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
    public static Command joystickDrive(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier)
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
            drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            linearVelocity.getX() * Constants.Drive.MAX_LINEAR_SPEED, linearVelocity.getY() * Constants.Drive.MAX_LINEAR_SPEED, omega * Constants.Drive.MAX_ANGULAR_SPEED, drive.getRotation().minus(allianceAdjustment)
                    )
            );
        }, drive);
    }

    public static Command driveAtOrientation(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, double setpoint, double maxSpeed)
    {
        return Commands.runOnce(() -> drive.rotateInit(setpoint, maxSpeed)).andThen(joystickDrive(drive, xSupplier, ySupplier, () -> drive.rotateExecute()));
    }

    public static Command aimAtSpeaker(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, double maxSpeed)
    {
        return Commands.runOnce(() -> drive.rotateInit(getHeadingToPose(drive, Constants.Field.BLUE_SPEAKER), maxSpeed))
                .andThen(joystickDrive(drive, xSupplier, ySupplier, () -> drive.rotateExecute(getHeadingToPose(drive, Constants.Field.BLUE_SPEAKER))));
    }

    public static Command aimAtAmp(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, double maxSpeed)
    {
        return Commands.runOnce(() -> drive.rotateInit(getHeadingToPose(drive, Constants.Field.BLUE_AMP), maxSpeed))
                .andThen(joystickDrive(drive, xSupplier, ySupplier, () -> drive.rotateExecute(getHeadingToPose(drive, Constants.Field.BLUE_AMP))));
    }

    private static double getHeadingToPose(Drive drive, Pose2d pose)
    {
        double deltaY = pose.getY() - drive.getPose().getY();
        double deltaX = pose.getX() - drive.getPose().getX();

        return Math.atan2(deltaY, deltaX) / Math.PI * 180;
    }

    // public static Command pathFinding(Drive drive, Pose2d targetPose,
    // PathConstraints constraints)
    // {
    // return new PathfindHolonomic(
    // targetPose, constraints, 3.0, // Goal end velocity in m/s. Optional
    // drive::getPose, drive::getChassisSpeeds, drive::runVelocity,
    // Constants.PathPlanner.pathFollowerConfig, // HolonomicPathFollwerConfig, see
    // the API or "Follow a single path" example for
    // // more info
    // 0.0, // Rotation delay distance in meters. This is how far the robot should
    // travel
    // // before attempting to rotate. Optional
    // drive // Reference to drive subsystem to set requirements
    // );
    // }
}
