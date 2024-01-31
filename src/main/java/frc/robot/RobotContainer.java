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

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonlib;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
    // Subsystems
    private final Drive drive;
    private Vision      _vision;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Dashboard inputs
    // private final LoggedDashboardNumber flywheelSpeedInput =
    // new LoggedDashboardNumber("Flywheel Speed", 1500.0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        switch (Constants.currentMode)
        {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(new GyroIO() {}, new ModuleIOSparkMax(0), new ModuleIOSparkMax(1), new ModuleIOSparkMax(2), new ModuleIOSparkMax(3));
                _vision = new Vision(new VisionIOPhotonlib());
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(new GyroIO() {}, new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                _vision = new Vision(new VisionIO() {});
                break;
        }

        // Set up auto routines
        // NamedCommands.registerCommand(
        // "Run Flywheel",
        // Commands.startEnd(
        // () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop,
        // flywheel)
        // .withTimeout(5.0));
        // autoChooser = new LoggedDashboardChooser<>("Auto Choices",
        // AutoBuilder.buildAutoChooser());

        // Set up feedforward characterization
        // autoChooser.addOption(
        // "Flywheel FF Characterization",
        // new FeedForwardCharacterization(
        // flywheel, flywheel::runVolts, flywheel::getCharacterizationVelocity));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings()
    {
        drive.setDefaultCommand(DriveCommands.joystickDrive(drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX()));
        // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
        // controller
        // .b()
        // .onTrue(
        // Commands.runOnce(
        // () ->
        // drive.setPose(
        // new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
        // drive)
        // .ignoringDisable(true));
        // controller
        // .a()
        // .whileTrue(
        // Commands.startEnd(
        // () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop,
        // flywheel));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return null;
    }
}
