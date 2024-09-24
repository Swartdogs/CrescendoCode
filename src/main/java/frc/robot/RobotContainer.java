package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.gyro.GyroIO;
import frc.robot.subsystems.gyro.GyroIONavX2;
import frc.robot.subsystems.gyro.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOHardware;
import frc.robot.subsystems.gyro.Gyro;

public class RobotContainer
{
    // Subsystems
    private final Drive _drive;
    private final Gyro  _gyro;

    // Controls
    private final CommandJoystick _joystick = new CommandJoystick(1);

    public RobotContainer()
    {

        switch (Constants.AdvantageKit.CURRENT_MODE)
        {
            // Real robot, instantiate hardware IO implementations
            case REAL:
                _gyro = new Gyro(new GyroIONavX2());
                _drive = new Drive(
                        _gyro, new ModuleIOHardware(Constants.CAN.MODULE_FL_DRIVE, Constants.CAN.MODULE_FL_ROTATE, Constants.AIO.MODULE_FL_SENSOR, Constants.Drive.MODULE_FL_OFFSET),
                        new ModuleIOHardware(Constants.CAN.MODULE_FR_DRIVE, Constants.CAN.MODULE_FR_ROTATE, Constants.AIO.MODULE_FR_SENSOR, Constants.Drive.MODULE_FR_OFFSET),
                        new ModuleIOHardware(Constants.CAN.MODULE_BL_DRIVE, Constants.CAN.MODULE_BL_ROTATE, Constants.AIO.MODULE_BL_SENSOR, Constants.Drive.MODULE_BL_OFFSET),
                        new ModuleIOHardware(Constants.CAN.MODULE_BR_DRIVE, Constants.CAN.MODULE_BR_ROTATE, Constants.AIO.MODULE_BR_SENSOR, Constants.Drive.MODULE_BR_OFFSET)
                );
                break;

            // Sim robot, instantiate physics sim IO implementations
            case SIM:
                _gyro = new Gyro(new GyroIOSim(this::getChassisSpeeds));
                _drive = new Drive(_gyro, new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
                break;

            // Replayed robot, disable IO implementations
            default:
                _gyro = new Gyro(new GyroIO() {});
                _drive = new Drive(_gyro, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                break;
        }

        // Configure the button bindings
        configureDefaultCommands();
        configureDriverCommands();
    }

    private void configureDefaultCommands()
    {
        _drive.setDefaultCommand(DriveCommands.joystickDrive(_drive, () -> -_joystick.getY(), () -> -_joystick.getX(), () -> -_joystick.getZ(), this::getRobotCentric));
    }

    private void configureDriverCommands()
    {
        _joystick.button(2).whileTrue(DriveCommands.reduceSpeed(_drive));
        _joystick.button(12).onTrue(DriveCommands.resetGyro(_drive, _gyro));
    }

    private ChassisSpeeds getChassisSpeeds()
    {
        return _drive.getChassisSpeeds();
    }

    private boolean getRobotCentric()
    {
        return _joystick.button(3).getAsBoolean();
    }

    public Command getAutonomousCommand()
    {
        return null;
    }
}
