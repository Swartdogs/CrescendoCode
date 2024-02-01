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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.CmdNotepathStartFeed;
import frc.robot.commands.CmdNotepathReverseFeed;
import frc.robot.commands.CmdIntakeReverseIntake;
import frc.robot.commands.CmdIntakeStartIntake;
import frc.robot.commands.CmdShooterBedSetBedAngle;
import frc.robot.commands.CmdShooterFlywheelShoot;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
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
import frc.robot.subsystems.shooter.ShooterBedIOSparkMax;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterFlywheelIO;
import frc.robot.subsystems.shooter.ShooterFlywheelIOSim;
import frc.robot.subsystems.shooter.ShooterFlywheelIOSparkMax;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer
{
    // Subsystems
    private final Drive           _drive;
    private final Intake          _intake;
    private final Notepath        _notepath;
    private final ShooterBed      _shooterBed;
    private final ShooterFlywheel _shooterFlywheel;

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> _autoChooser;

    // Controls
    private final Joystick              _joystick   = new Joystick(1);
    private final CommandXboxController _controller = new CommandXboxController(0);

    public RobotContainer()
    {
        switch (Constants.AdvantageKit.CURRENT_MODE)
        {
            // Real robot, instantiate hardware IO implementations
            case REAL:
                _drive = new Drive(
                        new GyroIONavX2(), new ModuleIOSparkMax(Constants.CAN.MODULE_FL_DRIVE, Constants.CAN.MODULE_FL_ROTATE, Constants.AIO.MODULE_FL_SENSOR, Constants.Drive.MODULE_FL_OFFSET),
                        new ModuleIOSparkMax(Constants.CAN.MODULE_FR_DRIVE, Constants.CAN.MODULE_FR_ROTATE, Constants.AIO.MODULE_FR_SENSOR, Constants.Drive.MODULE_FR_OFFSET),
                        new ModuleIOSparkMax(Constants.CAN.MODULE_BL_DRIVE, Constants.CAN.MODULE_BL_ROTATE, Constants.AIO.MODULE_BL_SENSOR, Constants.Drive.MODULE_BL_OFFSET),
                        new ModuleIOSparkMax(Constants.CAN.MODULE_BR_DRIVE, Constants.CAN.MODULE_BR_ROTATE, Constants.AIO.MODULE_BR_SENSOR, Constants.Drive.MODULE_BR_OFFSET)
                );
                _intake = new Intake(new IntakeIOSparkMax());
                _notepath = new Notepath(new NotepathIOSparkMax());
                _shooterBed = new ShooterBed(new ShooterBedIOSparkMax(Constants.ShooterBed.BED_ANGLE_OFFSET));
                _shooterFlywheel = new ShooterFlywheel(new ShooterFlywheelIOSparkMax());
                break;

            // Sim robot, instantiate physics sim IO implementations
            case SIM:
                _drive = new Drive(new GyroIO() {}, new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
                _intake = new Intake(new IntakeIOSim());
                _notepath = new Notepath(new NotepathIOSim());
                _shooterBed = new ShooterBed(new ShooterBedIOSim());
                _shooterFlywheel = new ShooterFlywheel(new ShooterFlywheelIOSim());
                break;

            // Replayed robot, disable IO implementations
            default:
                _drive = new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                _intake = new Intake(new IntakeIO() {});
                _notepath = new Notepath(new NotepathIO() {});
                _shooterBed = new ShooterBed(new ShooterBedIO() {});
                _shooterFlywheel = new ShooterFlywheel(new ShooterFlywheelIO() {});
                break;
        }

        _autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up feedforward characterization
        _autoChooser.addOption("Drive FF Characterization", new FeedForwardCharacterization(_drive, _drive::runCharacterizationVolts, _drive::getCharacterizationVelocity));

        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings()
    {
        CmdIntakeStartIntake     startIntake         = new CmdIntakeStartIntake(_intake);
        CmdIntakeReverseIntake   reverseIntake       = new CmdIntakeReverseIntake(_intake);
        CmdNotepathStartFeed     startNotepathFeed   = new CmdNotepathStartFeed(_notepath);
        CmdNotepathReverseFeed   reverseNotepathFeed = new CmdNotepathReverseFeed(_notepath);
        CmdShooterFlywheelShoot  flipShoot           = new CmdShooterFlywheelShoot(_shooterFlywheel, 6, 10);
        CmdShooterFlywheelShoot  straightShoot       = new CmdShooterFlywheelShoot(_shooterFlywheel, 8, 8);
        CmdShooterBedSetBedAngle setBedLow           = new CmdShooterBedSetBedAngle(_shooterBed, new Rotation2d(Math.PI / 6));
        CmdShooterBedSetBedAngle setBedHigh          = new CmdShooterBedSetBedAngle(_shooterBed, new Rotation2d(Math.PI / 4));

        _drive.setDefaultCommand(DriveCommands.joystickDrive(_drive, () -> -_joystick.getY(), () -> -_joystick.getX(), () -> -_joystick.getZ()));

        _controller.y().whileTrue(startNotepathFeed);
        _controller.x().whileTrue(reverseNotepathFeed);
        _controller.a().whileTrue(startIntake);
        _controller.b().whileTrue(reverseIntake);
        _controller.leftBumper().onTrue(setBedLow);
        _controller.rightBumper().onTrue(setBedHigh);
        _controller.back().whileTrue(flipShoot);
        _controller.start().whileTrue(straightShoot);
    }

    public Command getAutonomousCommand()
    {
        return _autoChooser.get();
    }
}
