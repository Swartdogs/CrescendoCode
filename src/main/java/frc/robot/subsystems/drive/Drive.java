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
package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.util.LocalADStarAK;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase
{
    private final Gyro                     _gyro;
    private final Module[]                 _modules    = new Module[4]; // FL, FR, BL, BR
    private final SwerveDrivePoseEstimator _poseEstimator;
    private final SwerveDriveKinematics    _kinematics = new SwerveDriveKinematics(getModuleTranslations());

    public Drive(Gyro gyro, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO)
    {
        _gyro = gyro;

        _modules[0] = new Module(flModuleIO, 0);
        _modules[1] = new Module(frModuleIO, 1);
        _modules[2] = new Module(blModuleIO, 2);
        _modules[3] = new Module(brModuleIO, 3);

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configureHolonomic(
                this::getPose, this::setPose, this::getChassisSpeeds, this::runVelocity, new HolonomicPathFollowerConfig(Constants.Drive.MAX_LINEAR_SPEED, Constants.Drive.DRIVE_BASE_RADIUS, new ReplanningConfig()),
                () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red, this
        );

        Pathfinding.setPathfinder(new LocalADStarAK());

        PathPlannerLogging.setLogActivePathCallback((activePath) ->
        {
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });

        PathPlannerLogging.setLogTargetPoseCallback((targetPose) ->
        {
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

        _poseEstimator = new SwerveDrivePoseEstimator(_kinematics, new Rotation2d(), getModulePositions(), new Pose2d());
    }

    public void periodic()
    {
        for (var module : _modules)
        {
            module.periodic();
        }

        // Stop moving when disabled
        if (DriverStation.isDisabled())
        {
            for (var module : _modules)
            {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled())
        {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry
        _poseEstimator.update(_gyro.getYawPosition(), getModulePositions());
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds)
    {
        // Calculate module setpoints
        ChassisSpeeds       discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = _kinematics.toSwerveModuleStates(discreteSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, Constants.Drive.MAX_LINEAR_SPEED);

        // Send setpoints to modules
        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];

        for (int i = 0; i < 4; i++)
        {
            // The module returns the optimized state, useful for logging
            optimizedSetpointStates[i] = _modules[i].runSetpoint(setpointStates[i]);
        }

        // Log setpoint states
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
    }

    /** Stops the drive. */
    public void stop()
    {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement.
     * The modules will return to their normal orientations the next time a nonzero
     * velocity is requested.
     */
    public void stopWithX()
    {
        Rotation2d[] headings = new Rotation2d[4];

        for (int i = 0; i < 4; i++)
        {
            headings[i] = getModuleTranslations()[i].getAngle();
        }

        _kinematics.resetHeadings(headings);
        stop();
    }

    /** Runs forwards at the commanded voltage. */
    public void runCharacterizationVolts(double volts)
    {
        for (int i = 0; i < 4; i++)
        {
            _modules[i].runCharacterization(volts);
        }
    }

    /** Returns the average drive velocity in radians/sec. */
    public double getCharacterizationVelocity()
    {
        double driveVelocityAverage = 0.0;

        for (var module : _modules)
        {
            driveVelocityAverage += module.getCharacterizationVelocity();
        }

        return driveVelocityAverage / 4.0;
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp)
    {
        _poseEstimator.addVisionMeasurement(pose, timestamp);
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the
     * modules.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates()
    {
        SwerveModuleState[] states = new SwerveModuleState[4];

        for (int i = 0; i < 4; i++)
        {
            states[i] = _modules[i].getState();
        }

        return states;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose()
    {
        return _poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation()
    {
        return _poseEstimator.getEstimatedPosition().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose)
    {
        _poseEstimator.resetPosition(_gyro.getYawPosition(), getModulePositions(), pose);
    }

    public SwerveModulePosition[] getModulePositions()
    {
        SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];

        for (int i = 0; i < 4; i++)
        {
            wheelPositions[i] = _modules[i].getPosition();
        }

        return wheelPositions;
    }

    public ChassisSpeeds getChassisSpeeds()
    {
        return _kinematics.toChassisSpeeds(getModuleStates());
    }

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations()
    {
        return new Translation2d[] { new Translation2d(Constants.Drive.TRACK_WIDTH_X / 2.0, Constants.Drive.TRACK_WIDTH_Y / 2.0), new Translation2d(Constants.Drive.TRACK_WIDTH_X / 2.0, -Constants.Drive.TRACK_WIDTH_Y / 2.0),
                new Translation2d(-Constants.Drive.TRACK_WIDTH_X / 2.0, Constants.Drive.TRACK_WIDTH_Y / 2.0), new Translation2d(-Constants.Drive.TRACK_WIDTH_X / 2.0, -Constants.Drive.TRACK_WIDTH_Y / 2.0) };
    }
}
