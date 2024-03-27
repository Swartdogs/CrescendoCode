package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CompositeCommands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ClimbCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOVictorSPX;
import frc.robot.commands.ShooterBedCommands;
import frc.robot.commands.ShooterFlywheelCommands;
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
import frc.robot.subsystems.notepath.Notepath.NotepathState;
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
    private final LED             _led;

    // Dashboard inputs
    //private final LoggedDashboardChooser<Command> _autoChooser;
    @SuppressWarnings("unused")
    private final Dashboard       _dashboard;

    // Controls
    private final Joystick              _joystick   = new Joystick(1);
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
                _vision = new Vision(_drive, new VisionIOPhotonlib());
                _intake = new Intake(new IntakeIOSparkMax());
                _notepath = new Notepath(new NotepathIOSparkMax());
                _shooterBed = new ShooterBed(new ShooterBedIOVictorSPX());
                _shooterFlywheel = new ShooterFlywheel(new ShooterFlywheelIOSparkMax());
                _climb = new Climb(_gyro, new ClimbIOVictorSPX());
                _led = new LED(new LEDIOHardware());
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
                _led = new LED(new LEDIOSim());
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
                _led = new LED(new LEDIO() {});
                break;
        }

        // Configure the button bindings
        configureButtonBindings();
        _dashboard = new Dashboard(_shooterBed, _notepath, _shooterFlywheel, _drive, _intake, _climb, _led);
    }

    private void configureButtonBindings()
    {
        Trigger _hasNote = new Trigger(() -> _notepath.hasNote());
        // _controller.a().onTrue(CompositeCommands.shooterPickup(_shooterBed,
        // _shooterFlywheel, _notepath));

        _drive.setDefaultCommand(DriveCommands.joystickDrive(_drive, () -> -_joystick.getY(), () -> -_joystick.getX(), () -> -_joystick.getZ()));
        // _shooterBed.setDefaultCommand(ShooterBedCommands.setVolts(_shooterBed, () ->
        // Constants.ShooterBed.MAX_BED_VOLTS *
        // -MathUtil.applyDeadband(_controller.getLeftY(),
        // Constants.Controls.JOYSTICK_DEADBAND)));

        _hasNote.onTrue(CompositeCommands.LEDBlinkingTeleOp(_led));
        _hasNote.onFalse(CompositeCommands.LEDBlink(_led, Constants.LED.RED).andThen(CompositeCommands.LEDTeleop(_led)));

        // _controller.y().whileTrue(IntakeCommands.start(_intake).andThen(Commands.idle(_intake)).finallyDo(()
        // -> _intake.set(IntakeState.Off)));
        // _controller.a().whileTrue(ShooterFlywheelCommands.intake(_shooterFlywheel).andThen(Commands.idle(_shooterFlywheel)).finallyDo(()
        // -> _shooterFlywheel.stop()));

        _controller.a().onTrue(CompositeCommands.intakePickup(_intake, _notepath, _shooterBed));
        _controller.b().onTrue(CompositeCommands.stopIntaking(_intake, _notepath));
        _controller.y().onTrue(CompositeCommands.shooterPickup(_shooterBed, _shooterFlywheel, _notepath));
        _controller.x().whileTrue(CompositeCommands.suckIn(_notepath, _shooterFlywheel).andThen(Commands.idle(_notepath, _shooterFlywheel)).finallyDo(() ->
        {
            _notepath.set(NotepathState.Off);
            _shooterFlywheel.stop();
        }));

        // _controller.y().onTrue(CompositeCommands.intakePickup(_intake, _notepath,
        // _shooterBed));
        // _controller.x().onTrue(CompositeCommands.stopIntaking(_intake, _notepath));
        // _controller.a().onTrue(CompositeCommands.shooterPickup(_shooterBed,
        // _shooterFlywheel, _notepath));
        // _controller.b().onTrue(CompositeCommands.stopShooter(_shooterFlywheel,
        // _notepath));

        // Test commands for shooterbed - on gamepad
        // _controller.back().onTrue(ShooterBedCommands.setBedIntakePickupAngle(_shooterBed));
        // _controller.start().onTrue(ShooterBedCommands.setBedShooterPickupAngle(_shooterBed));
        // _controller.leftBumper().whileTrue(ShooterBedCommands.runBed(_shooterBed, ()
        // -> -_controller.getLeftY() * Constants.General.MOTOR_VOLTAGE));
        // _controller.rightBumper().onTrue(ShooterBedCommands.setBedAngle(_shooterBed,
        // 45));
        _controller.leftBumper().whileTrue(ShooterBedCommands.setVolts(_shooterBed, Constants.ShooterBed.MAX_BED_VOLTS).andThen(Commands.idle(_shooterBed)).finallyDo(() -> _shooterBed.setVolts(0)));
        _controller.rightBumper().whileTrue(ShooterBedCommands.setVolts(_shooterBed, -Constants.ShooterBed.MAX_BED_VOLTS).andThen(Commands.idle(_shooterBed)).finallyDo(() -> _shooterBed.setVolts(0)));

        _controller.start().onTrue(CompositeCommands.startShooter(_shooterFlywheel, _notepath, _shooterBed, 4000, 4000, ShooterBed.BedAngle.SubwooferShot));
        _controller.back().onTrue(CompositeCommands.stopShooter(_shooterFlywheel, _notepath));
        // _controller.start().onTrue(ShooterBedCommands.setAngle(_shooterBed,
        // ShooterBed.BedAngle.ShooterLoad));
        // _controller.back().onTrue(ShooterBedCommands.setAngle(_shooterBed,
        // ShooterBed.BedAngle.SubwooferShot));

        _controller.leftTrigger().whileTrue(CompositeCommands.climbJoystick(_climb, _shooterBed, ShooterBed.BedAngle.ClimbVertical, () -> -_controller.getLeftY(), () -> -_controller.getRightY()));
        _controller.rightTrigger().onTrue(CompositeCommands.startNotepath(_notepath, _shooterFlywheel, _led));

        _controller.povUp().onTrue(Commands.runOnce(() -> _notepath.setHasNote(true)).ignoringDisable(true));
        _controller.povDown().onTrue(Commands.runOnce(() -> _notepath.setHasNote(false)).ignoringDisable(true));
        _controller.povLeft().onTrue(CompositeCommands.startShooter(_shooterFlywheel, 4000, 4000));
        _controller.povRight().onTrue(CompositeCommands.startShooter(_shooterFlywheel, 2000, 5000));

        new JoystickButton(_joystick, 12).onTrue(DriveCommands.resetGyro(_drive, _gyro));
        new JoystickButton(_joystick, 11).whileTrue(DriveCommands.driveVolts(_drive, 12).andThen(Commands.idle(_drive)).finallyDo(() -> _drive.stop()));

        // Test commands for climb - on gamepad
        _controller.leftTrigger().whileTrue(ClimbCommands.setVolts(_climb, () -> -_controller.getLeftY(), () -> -_controller.getRightY()).finallyDo(() -> _climb.stop()));
        // _controller.rightTrigger().onTrue(ClimbCommands.setHeight(_climb, 0)); //
        // TODO: set setpoint

        new JoystickButton(_testJoy, 12).onTrue(CompositeCommands.loadInMotion(_intake, _notepath));
        new JoystickButton(_testJoy, 11).onTrue(CompositeCommands.loadWhileStopped(_intake, _notepath));
        new JoystickButton(_testJoy, 10).onTrue(CompositeCommands.intakePickup(_intake, _notepath, _shooterBed));
        new JoystickButton(_testJoy, 9).onTrue(CompositeCommands.loadWhileStopped(_intake, _notepath));
        new JoystickButton(_testJoy, 8).onTrue(CompositeCommands.startNotepath(_notepath, _shooterFlywheel, _led));
        new JoystickButton(_testJoy, 7).onTrue(ShooterFlywheelCommands.start(_shooterFlywheel, 3000, 3000));
        new JoystickButton(_testJoy, 1).onTrue(Commands.runOnce(() ->
        {
        }, _intake, _notepath, _shooterBed, _shooterFlywheel));

        // // Test commands for notepath - on joystick
        // new JoystickButton(_joystick,
        // 3).whileTrue(NotepathCommands.intakeLoad(_notepath).andThen(Commands.idle(_notepath)).finallyDo(()
        // -> _notepath.set(NotepathState.Off)));
        // new JoystickButton(_joystick,
        // 4).whileTrue(NotepathCommands.shooterLoad(_notepath).andThen(Commands.idle(_notepath)).finallyDo(()
        // -> _notepath.set(NotepathState.Off)));
        // new JoystickButton(_joystick,
        // 5).whileTrue(NotepathCommands.feed(_notepath).andThen(Commands.idle(_notepath)).finallyDo(()
        // -> _notepath.set(NotepathState.Off)));
        // new JoystickButton(_joystick, 6).onTrue(NotepathCommands.stop(_notepath));

        // // Test commands for shooterfly - on joystick
        // new JoystickButton(_joystick,
        // 7).whileTrue(ShooterFlywheelCommands.start(_shooterFlywheel, 2000,
        // 2000).andThen(Commands.idle(_shooterFlywheel)).finallyDo(() ->
        // _shooterFlywheel.stop()));
        // new JoystickButton(_joystick,
        // 8).whileTrue(ShooterFlywheelCommands.start(_shooterFlywheel, 1000,
        // 2000).andThen(Commands.idle(_shooterFlywheel)).finallyDo(() ->
        // _shooterFlywheel.stop()));
        // new JoystickButton(_joystick,
        // 9).onTrue(ShooterFlywheelCommands.intake(_shooterFlywheel));
        // new JoystickButton(_joystick,
        // 10).onTrue(ShooterFlywheelCommands.stop(_shooterFlywheel));
    }

    private ChassisSpeeds getChassisSpeeds()
    {
        return _drive.getChassisSpeeds();
    }

    public Command getAutonomousCommand()
    {
        return AutoBuilder.buildAuto("New Auto");
        //return _dashboard.getAuto();
    }

    public LED getLEDSubsystem()
    {
        return _led;
    }
}
