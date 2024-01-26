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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CmdShooterBedSetBedAngle;
import frc.robot.commands.CmdShooterFlywheelShoot;
import frc.robot.commands.CmdShooterFlywheelStop;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.shooter.ShooterBed;
import frc.robot.subsystems.shooter.ShooterBedIOSparkMax;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterFlywheelIOSparkMax;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer
{
    // Subsystems
    private final Drive _drive;
    private ShooterBed _shooterBed;
    private ShooterFlywheel _shooterFlywheel;

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> _autoChooser;

    // Controls
    private final CommandXboxController _controller = new CommandXboxController(1);

    public RobotContainer()
    {
        switch (Constants.AdvantageKit.CURRENT_MODE)
        {
        // Real robot, instantiate hardware IO implementations
        case REAL:
            _drive = new Drive(new GyroIONavX2(),
                            new ModuleIOSparkMax(Constants.CAN.MODULE_FL_DRIVE, Constants.CAN.MODULE_FL_ROTATE,
                                            Constants.AIO.MODULE_FL_SENSOR, Constants.Drive.MODULE_FL_OFFSET),
                            new ModuleIOSparkMax(Constants.CAN.MODULE_FR_DRIVE, Constants.CAN.MODULE_FR_ROTATE,
                                            Constants.AIO.MODULE_FR_SENSOR, Constants.Drive.MODULE_FR_OFFSET),
                            new ModuleIOSparkMax(Constants.CAN.MODULE_BL_DRIVE, Constants.CAN.MODULE_BL_ROTATE,
                                            Constants.AIO.MODULE_BL_SENSOR, Constants.Drive.MODULE_BL_OFFSET),
                            new ModuleIOSparkMax(Constants.CAN.MODULE_BR_DRIVE, Constants.CAN.MODULE_BR_ROTATE,
                                            Constants.AIO.MODULE_BR_SENSOR, Constants.Drive.MODULE_BR_OFFSET));

            _shooterBed         = new ShooterBed(new ShooterBedIOSparkMax(7, 8, 3, Constants.Shooter.BED_ANGLE_OFFSET));
            _shooterFlywheel    = new ShooterFlywheel(new ShooterFlywheelIOSparkMax(9, 10)); // FIXME: Set correct IDs
            break;

        // Sim robot, instantiate physics sim IO implementations
        case SIM:
            _drive = new Drive(new GyroIO()
            {}, new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
            break;

        // Replayed robot, disable IO implementations
        default:
            _drive = new Drive(new GyroIO()
            {}, new ModuleIO()
            {}, new ModuleIO()
            {}, new ModuleIO()
            {}, new ModuleIO()
            {});
            break;
        }

        _autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up feedforward characterization
        _autoChooser.addOption("Drive FF Characterization", new FeedForwardCharacterization(_drive,
                        _drive::runCharacterizationVolts, _drive::getCharacterizationVelocity));

        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings()
    {
        CmdShooterFlywheelShoot _flipShoot = new CmdShooterFlywheelShoot(_shooterFlywheel, 6, 10);
        CmdShooterFlywheelShoot _straightShoot = new CmdShooterFlywheelShoot(_shooterFlywheel, 8, 8);

        CmdShooterFlywheelStop _stopShooter = new CmdShooterFlywheelStop(_shooterFlywheel);

        CmdShooterBedSetBedAngle _setBedLow = new CmdShooterBedSetBedAngle(_shooterBed, new Rotation2d(30));
        CmdShooterBedSetBedAngle _setBedHigh = new CmdShooterBedSetBedAngle(_shooterBed, new Rotation2d(45));

        _drive.setDefaultCommand(DriveCommands.joystickDrive(_drive, () -> -_controller.getLeftY(), () -> -_controller.getRightX(),
                        () -> -_controller.getRightX()));
        _controller.leftTrigger().whileTrue(_straightShoot);
        _controller.rightTrigger().whileTrue(_flipShoot);
        _controller.leftBumper().whileTrue(_stopShooter);
        _controller.povDown().whileTrue(_setBedLow);
        _controller.povUp().whileTrue(_setBedHigh);

    }

    public Command getAutonomousCommand()
    {
        return _autoChooser.get();
    }
}
