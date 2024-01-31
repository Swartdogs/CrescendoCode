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

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonlib;

public class RobotContainer
{
    // Subsystems
    private final Drive _drive;
    private Vision      _vision;

    // Controls
    private final Joystick _joystick = new Joystick(1);

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
                _vision = new Vision(_drive, new VisionIOPhotonlib());
                break;

            // Sim robot, instantiate physics sim IO implementations
            case SIM:
                _drive = new Drive(new GyroIO() {}, new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
                break;

            // Replayed robot, disable IO implementations
            default:
                _drive = new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                _vision = new Vision(_drive, new VisionIO() {});
                break;
        }

        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings()
    {
        _drive.setDefaultCommand(DriveCommands.joystickDrive(_drive, () -> -_joystick.getY(), () -> -_joystick.getX(), () -> -_joystick.getZ()));
    }

    public Command getAutonomousCommand()
    {
        return null;
    }
}
