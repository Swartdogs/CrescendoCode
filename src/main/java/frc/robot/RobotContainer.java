package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CompositeCommands;
import frc.robot.commands.ClimbCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShooterBedCommands;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOVictorSPX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.gyro.GyroIO;
import frc.robot.subsystems.gyro.GyroIONavX2;
import frc.robot.subsystems.gyro.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOHardware;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonlib;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.leds.LED;
import frc.robot.subsystems.leds.LEDIO;
import frc.robot.subsystems.leds.LEDIOHardware;
import frc.robot.subsystems.leds.LEDIOSim;
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
import static frc.robot.Constants.LED.*;

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
    private final LED             _led;

    // Dashboard inputs
    // private final LoggedDashboardChooser<Command> _autoChooser;
    @SuppressWarnings("unused")
    private final Dashboard _dashboard;

    // Controls
    private final CommandJoystick       _joystick   = new CommandJoystick(1);
    @SuppressWarnings("unused")
    private final Joystick              _testJoy    = new Joystick(2);
    private final CommandXboxController _controller = new CommandXboxController(0);

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
                _intake = new Intake(new IntakeIOSparkMax());
                _notepath = new Notepath(new NotepathIOSparkMax());
                _shooterBed = new ShooterBed(new ShooterBedIOVictorSPX());
                _shooterFlywheel = new ShooterFlywheel(new ShooterFlywheelIOSparkMax());
                _climb = new Climb(_gyro, new ClimbIOVictorSPX());
                _led = new LED(new LEDIOHardware());
                _vision = new Vision(_drive, new VisionIOPhotonlib(_drive));
                break;

            // Sim robot, instantiate physics sim IO implementations
            case SIM:
                _gyro = new Gyro(new GyroIOSim(this::getChassisSpeeds));
                _drive = new Drive(_gyro, new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
                _intake = new Intake(new IntakeIOSim());
                _notepath = new Notepath(new NotepathIOSim());
                _shooterBed = new ShooterBed(new ShooterBedIOSim());
                _shooterFlywheel = new ShooterFlywheel(new ShooterFlywheelIOSim());
                _climb = new Climb(_gyro, new ClimbIOSim());
                _led = new LED(new LEDIOSim());
                _vision = new Vision(_drive, new VisionIO() {});
                break;

            // Replayed robot, disable IO implementations
            default:
                _gyro = new Gyro(new GyroIO() {});
                _drive = new Drive(_gyro, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                _intake = new Intake(new IntakeIO() {});
                _notepath = new Notepath(new NotepathIO() {});
                _shooterBed = new ShooterBed(new ShooterBedIO() {});
                _shooterFlywheel = new ShooterFlywheel(new ShooterFlywheelIO() {});
                _climb = new Climb(_gyro, new ClimbIO() {});
                _led = new LED(new LEDIO() {});
                _vision = new Vision(_drive, new VisionIO() {});
                break;
        }

        _dashboard = new Dashboard(_shooterBed, _notepath, _shooterFlywheel, _drive, _intake, _climb, _led);

        // Configure the button bindings
        configureDefaultCommands();
        configureDriverCommands();
        configureOperatorCommands();
    }

    private void configureDefaultCommands()
    {
        Trigger _hasNote    = new Trigger(() -> _notepath.hasNote());
        Trigger _isShooting = new Trigger(() -> _shooterFlywheel.isShooting());
        Trigger _isIntaking = new Trigger(() -> _intake.isIntaking() || _shooterFlywheel.isIntaking());
        // _controller.a().onTrue(CompositeCommands.shooterPickup(_shooterBed,
        // _shooterFlywheel, _notepath));

        _drive.setDefaultCommand(DriveCommands.joystickDrive(_drive, () -> -_joystick.getY(), () -> -_joystick.getX(), () -> -_joystick.getZ(), this::getRobotCentric, _dashboard));
        // _shooterBed.setDefaultCommand(ShooterBedCommands.setVolts(_shooterBed, () ->
        // Constants.ShooterBed.MAX_BED_VOLTS *
        // -MathUtil.applyDeadband(_controller.getLeftY(),
        // Constants.Controls.JOYSTICK_DEADBAND)));

        _led.setDefaultCommand(CompositeCommands.Teleop.LEDSetSolidColor(_led, ORANGE));

        _hasNote.onTrue(CompositeCommands.Teleop.LEDSetDefaultColor(_led, BLUE));
        _hasNote.onFalse(CompositeCommands.Teleop.LEDSetDefaultColor(_led, PURPLE));
        _isShooting.whileTrue(CompositeCommands.Teleop.LEDPulseColor(_led, BLUE));
        _isIntaking.whileTrue(CompositeCommands.Teleop.LEDPulseColor(_led, PURPLE));
    }

    private void configureDriverCommands()
    {
        _joystick.button(1).onTrue(CompositeCommands.General.startNotepath(_shooterBed, _notepath, _shooterFlywheel, _drive));
        _joystick.button(2).whileTrue(DriveCommands.reduceSpeed(_drive));
        _joystick.button(4).onTrue(Commands.runOnce(() -> _dashboard.toggleCamera()).ignoringDisable(true));
        _joystick.button(7).whileTrue(CompositeCommands.Teleop.redSourceOrPass(_drive, _dashboard, () -> -_joystick.getY(), () -> -_joystick.getX(), this::getRobotCentric, 0.6)); // 60 degrees left
        _joystick.button(8).whileTrue(CompositeCommands.Teleop.blueSourceOrPass(_drive, _dashboard, () -> -_joystick.getY(), () -> -_joystick.getX(), this::getRobotCentric, 0.6)); // 60 degrees right
        _joystick.button(9).whileTrue(CompositeCommands.Teleop.blueAmpOrSubwoofer(_drive, _dashboard, () -> -_joystick.getY(), () -> -_joystick.getX(), this::getRobotCentric, 0.6));
        _joystick.button(10).whileTrue(CompositeCommands.Teleop.redAmpOrSubwoofer(_drive, _dashboard, () -> -_joystick.getY(), () -> -_joystick.getX(), this::getRobotCentric, 0.6));
        _joystick.button(11).whileTrue(CompositeCommands.Teleop.visionAimAtSpeaker(_drive, _vision, () -> -_joystick.getY(), () -> -_joystick.getX(), () -> -_joystick.getZ(), this::getRobotCentric, _dashboard));
        _joystick.button(12).onTrue(DriveCommands.resetGyro(_drive, _gyro));
    }

    private void configureOperatorCommands()
    {
        _controller.a().onTrue(CompositeCommands.Teleop.intakePickup(_intake, _notepath, _shooterBed, _controller.getHID()));
        _controller.b().onTrue(CompositeCommands.General.stopIntaking(_intake, _notepath));
        _controller.x().whileTrue(CompositeCommands.Teleop.suckIn(_notepath, _shooterFlywheel));
        _controller.y().onTrue(CompositeCommands.Teleop.shooterPickup(_shooterBed, _shooterFlywheel, _notepath, _controller.getHID()));

        _controller.leftTrigger().whileTrue(CompositeCommands.Teleop.climbJoystick(_climb, () -> -_controller.getRightY(), () -> -_controller.getLeftY()));
        _controller.rightTrigger().whileTrue(CompositeCommands.Teleop.setBedVolts(_shooterBed, () -> -_controller.getRightY()));

        _controller.leftStick().onTrue(CompositeCommands.General.stopShooter(_shooterFlywheel, _notepath));
        // _controller.rightStick().whileTrue(CompositeCommands.Teleop.LEDPartyMode(_led));
        // _controller.rightStick().onTrue(CompositeCommands.General.startNotepath(_shooterBed,
        // _notepath, _shooterFlywheel, _drive));

        _controller.leftBumper().whileTrue(ClimbCommands.setHeight(_climb, 1));
        _controller.rightBumper().whileTrue(ClimbCommands.setHeight(_climb, 10.5));

        _controller.start().onTrue(CompositeCommands.General.setHasNote(_notepath, true));
        _controller.back().onTrue(CompositeCommands.General.setHasNote(_notepath, false));

        // _controller.povUp().onTrue(ShooterBedCommands.setAngle(_shooterBed, 65));
        // _controller.povLeft().onTrue(ShooterBedCommands.setAngle(_shooterBed, 60));
        // _controller.povRight().onTrue(ShooterBedCommands.setAngle(_shooterBed, 60));
        // _controller.povDown().onTrue(ShooterBedCommands.setAngle(_shooterBed, 30));

        _controller.povUp().onTrue(CompositeCommands.Teleop.startShooter(_shooterFlywheel, _notepath, _shooterBed, 2800, 2800, ShooterBed.BedAngle.TrapShot));
        _controller.povDown().onTrue(CompositeCommands.Teleop.startShooter(_shooterFlywheel, _notepath, _shooterBed, 4000, 4000, ShooterBed.BedAngle.SubwooferShot));
        _controller.povLeft().onTrue(CompositeCommands.Teleop.blueAmpOrPodium(_shooterFlywheel, _notepath, _shooterBed));
        _controller.povRight().onTrue(CompositeCommands.Teleop.redAmpOrPodium(_shooterFlywheel, _notepath, _shooterBed));
    }

    private ChassisSpeeds getChassisSpeeds()
    {
        return _drive.getChassisSpeeds();
    }

    private boolean getRobotCentric()
    {
        return _joystick.button(3).getAsBoolean();
    }

    public LED getLEDSubsystem()
    {
        return _led;
    }

    public Command getAutonomousCommand()
    {
        return _dashboard.getAuto();
    }
}
