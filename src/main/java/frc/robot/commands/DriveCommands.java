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
package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

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

            // Calcaulate new linear velocity
            Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection).transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();

            // Convert to field relative speeds & send command
            drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(linearVelocity.getX() * Constants.Drive.MAX_LINEAR_SPEED, linearVelocity.getY() * Constants.Drive.MAX_LINEAR_SPEED, omega * Constants.Drive.MAX_ANGULAR_SPEED, drive.getRotation())
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

    public static Command pathFinding(Pose2d pose, PathConstraints constraints)
    {
        return AutoBuilder.pathfindToPose(
                pose, constraints, 0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel
                    // before attempting to rotate.
        );
    }
}
