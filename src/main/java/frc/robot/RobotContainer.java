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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CompositeCommands;
import frc.robot.commands.ClimbCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOVictorSPX;
import frc.robot.commands.ShooterBedCommands;
import frc.robot.commands.ShooterFlywheelCommands;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.gyro.GyroIO;
import frc.robot.subsystems.gyro.GyroIONavX2;
import frc.robot.subsystems.gyro.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonlib;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.notepath.Notepath;
import frc.robot.subsystems.notepath.NotepathIO;
import frc.robot.subsystems.notepath.NotepathIOSim;
import frc.robot.subsystems.notepath.NotepathIOSparkMax;
import frc.robot.subsystems.shooter.ShooterBed;
import frc.robot.subsystems.shooter.ShooterBedIO;
import frc.robot.subsystems.shooter.ShooterBedIOSim;
import frc.robot.subsystems.shooter.ShooterBedIOVictorSPX;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterFlywheelIO;
import frc.robot.subsystems.shooter.ShooterFlywheelIOSim;
import frc.robot.subsystems.shooter.ShooterFlywheelIOSparkMax;

public class RobotContainer
{
    // Subsystems
    private final Drive           _drive;
    private final Intake          _intake;
    private final Notepath        _notepath;
    private final ShooterBed      _shooterBed;
    private final ShooterFlywheel _shooterFlywheel;
    private final Climb           _climb;
    private final Gyro            _gyro;
    @SuppressWarnings("unused")
    private final Vision          _vision;
    @SuppressWarnings("unused")
    private final Dashboard       _dashboard;

    // Controls
    private final Joystick              _joystick   = new Joystick(1);
    private final CommandXboxController _controller = new CommandXboxController(0);

    public RobotContainer()
    {

        switch (Constants.AdvantageKit.CURRENT_MODE)
        {
            // Real robot, instantiate hardware IO implementations
            case REAL:
                _gyro = new Gyro(new GyroIONavX2());
                _drive = new Drive(
                        _gyro, new ModuleIOSparkMax(Constants.CAN.MODULE_FL_DRIVE, Constants.CAN.MODULE_FL_ROTATE, Constants.AIO.MODULE_FL_SENSOR, Constants.Drive.MODULE_FL_OFFSET),
                        new ModuleIOSparkMax(Constants.CAN.MODULE_FR_DRIVE, Constants.CAN.MODULE_FR_ROTATE, Constants.AIO.MODULE_FR_SENSOR, Constants.Drive.MODULE_FR_OFFSET),
                        new ModuleIOSparkMax(Constants.CAN.MODULE_BL_DRIVE, Constants.CAN.MODULE_BL_ROTATE, Constants.AIO.MODULE_BL_SENSOR, Constants.Drive.MODULE_BL_OFFSET),
                        new ModuleIOSparkMax(Constants.CAN.MODULE_BR_DRIVE, Constants.CAN.MODULE_BR_ROTATE, Constants.AIO.MODULE_BR_SENSOR, Constants.Drive.MODULE_BR_OFFSET)
                );
                _vision = new Vision(_drive, new VisionIOPhotonlib());
                _intake = new Intake(new IntakeIOSparkMax());
                _notepath = new Notepath(new NotepathIOSparkMax());
                _shooterBed = new ShooterBed(new ShooterBedIOVictorSPX());
                _shooterFlywheel = new ShooterFlywheel(new ShooterFlywheelIOSparkMax());
                _climb = new Climb(_gyro, new ClimbIOVictorSPX());
                break;

            // Sim robot, instantiate physics sim IO implementations
            case SIM:
                _gyro = new Gyro(new GyroIOSim(this::getChassisSpeeds));
                _drive = new Drive(_gyro, new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
                _vision = new Vision(_drive, new VisionIOPhotonlib());
                _intake = new Intake(new IntakeIOSim());
                _notepath = new Notepath(new NotepathIOSim());
                _shooterBed = new ShooterBed(new ShooterBedIOSim());
                _shooterFlywheel = new ShooterFlywheel(new ShooterFlywheelIOSim());
                _climb = new Climb(_gyro, new ClimbIOSim());
                break;

            // Replayed robot, disable IO implementations
            default:
                _gyro = new Gyro(new GyroIO() {});
                _drive = new Drive(_gyro, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                _vision = new Vision(_drive, new VisionIO() {});
                _intake = new Intake(new IntakeIO() {});
                _notepath = new Notepath(new NotepathIO() {});
                _shooterBed = new ShooterBed(new ShooterBedIO() {});
                _shooterFlywheel = new ShooterFlywheel(new ShooterFlywheelIO() {});
                _climb = new Climb(_gyro, new ClimbIO() {});
                break;
        }

        _dashboard = new Dashboard(_shooterBed, _notepath, _shooterFlywheel, _drive, _intake, _climb);

        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings()
    {
        Pose2d targetPose = new Pose2d(300, 200, Rotation2d.fromDegrees(180));
        PathConstraints constraints = new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
        Command pathfindingCommand = AutoBuilder.pathfindToPose(targetPose, constraints, 0.0, 0.0);

        _drive.setDefaultCommand(DriveCommands.joystickDrive(_drive, () -> -_joystick.getY(), () -> -_joystick.getX(), () -> -_joystick.getZ()));
        new JoystickButton(_joystick, 4).whileTrue(DriveCommands.driveAtOrientation(_drive, () -> -_joystick.getY(), () -> -_joystick.getX(), 90.00, 1)); // TODO: test values
        new JoystickButton(_joystick, 3).whileTrue(DriveCommands.aimAtSpeaker(_drive, () -> -_joystick.getY(), () -> -_joystick.getX(), 1)); // TODO: test values
        new JoystickButton(_joystick, 2).whileTrue(DriveCommands.aimAtAmp(_drive, () -> -_joystick.getY(), () -> -_joystick.getX(), 1)); // TODO: test values

        _controller.y().onTrue(CompositeCommands.startIntake(_intake, _notepath, _shooterBed));
        _controller.x().onTrue(CompositeCommands.stopIntaking(_intake, _notepath));
        _controller.a().onTrue(CompositeCommands.shooterPickup(_shooterBed, _shooterFlywheel, _notepath));
        _controller.b().onTrue(CompositeCommands.stopShooter(_shooterFlywheel, _notepath));

        _controller.leftBumper().onTrue(pathfindingCommand);
        _controller.rightBumper().onTrue(ShooterBedCommands.setBedAngle(_shooterBed, 45));

        _controller.back().whileTrue(ShooterFlywheelCommands.shooterFlywheelShoot(_shooterFlywheel, 6, 10).andThen(Commands.idle(_shooterFlywheel)).finallyDo(() -> _shooterFlywheel.stop()));
        _controller.start().whileTrue(ShooterFlywheelCommands.shooterFlywheelShoot(_shooterFlywheel, 8, 8).andThen(Commands.idle(_shooterFlywheel)).finallyDo(() -> _shooterFlywheel.stop()));

        _controller.leftBumper().whileTrue(ClimbCommands.setVoltage(_climb, () -> -_controller.getLeftY(), () -> -_controller.getRightY()).finallyDo(() -> _climb.stop()));
        _controller.rightBumper().onTrue(ClimbCommands.setHeight(_climb, 0)); // TODO: set setpoint
        _controller.rightTrigger().onTrue(CompositeCommands.startShooter(_shooterFlywheel, 3000, 3000));
        _controller.leftTrigger().onTrue(CompositeCommands.startNotepath(_notepath, _shooterFlywheel));
    }

    private ChassisSpeeds getChassisSpeeds()
    {
        return _drive.getChassisSpeeds();
    }

    public Command getAutonomousCommand()
    {
        return _dashboard.getAuto();
    }
}
