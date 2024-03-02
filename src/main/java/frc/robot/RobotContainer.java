package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CompositeCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOVictorSPX;
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
    @SuppressWarnings("unused")
    private final Joystick              _testJoy    = new Joystick(2);
    private final CommandXboxController _controller = new CommandXboxController(0);

    // public enum Controller
    // {
    // Joystick(0), Joystick(1);

    // private Joystick _joystick;
    // private Joystick _testJoy;
    // private ArrayList<JoystickButton> _buttons;

    // private Controller(int port, )
    // {
    // _joystick = new Joystick(port);
    // _buttons = new ArrayList<JoystickButton>();

    // for (int i = 0; i < 12; i++)
    // {
    // _buttons.add(new JoystickButton(_joystick, i + 1));
    // }
    // }

    // public Joystick joystick()
    // {
    // return _joystick;
    // }

    // public Joystick testJoy()
    // {
    // return _testJoy;
    // }

    // public JoystickButton button(int button)
    // {
    // return _buttons.get(button - 1);
    // }
    // }

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
        configureDefaultCommands();
        configureDriverCommands();
        configureOperatorCommands();
        // configureTestCommands();
    }

    private void configureDefaultCommands()
    {
        _drive.setDefaultCommand(DriveCommands.joystickDrive(_drive, () -> -_joystick.getY(), () -> -_joystick.getX(), () -> -_joystick.getZ()));
    }

    private void configureDriverCommands()
    {
        new JoystickButton(_joystick, 12).onTrue(DriveCommands.resetGyro(_drive, _gyro));
        new JoystickButton(_joystick, 2).onTrue(Commands.runOnce(()-> _dashboard.toggle()));
    }

    private void configureOperatorCommands()
    {
        _controller.a().onTrue(CompositeCommands.Teleop.intakePickup(_intake, _notepath, _shooterBed, _controller.getHID()));
        _controller.b().onTrue(CompositeCommands.General.stopIntaking(_intake, _notepath));
        _controller.x().whileTrue(CompositeCommands.Teleop.suckIn(_notepath, _shooterFlywheel));
        _controller.y().onTrue(CompositeCommands.Teleop.shooterPickup(_shooterBed, _shooterFlywheel, _notepath));

        _controller.leftBumper().whileTrue(CompositeCommands.Teleop.setBedVolts(_shooterBed, Constants.ShooterBed.MAX_BED_VOLTS));
        _controller.rightBumper().whileTrue(CompositeCommands.Teleop.setBedVolts(_shooterBed, -Constants.ShooterBed.MAX_BED_VOLTS));

        _controller.start().onTrue(CompositeCommands.Teleop.startShooter(_shooterFlywheel, _notepath, _shooterBed, 4000, 4000, ShooterBed.BedAngle.SubwooferShot));
        _controller.back().onTrue(CompositeCommands.General.stopShooter(_shooterFlywheel, _notepath));

        // Intentionally control left arm with right stick and right arm with left
        // stick. It's more intuitive for the drivers
        _controller.leftTrigger().whileTrue(CompositeCommands.Teleop.climbJoystick(_climb, () -> -_controller.getRightY(), () -> -_controller.getLeftY()));
        _controller.rightTrigger().onTrue(CompositeCommands.General.startNotepath(_shooterBed, _notepath, _shooterFlywheel));

        _controller.povUp().onTrue(CompositeCommands.General.setHasNote(_notepath, true));
        _controller.povDown().onTrue(CompositeCommands.General.setHasNote(_notepath, false));
        _controller.povLeft().onTrue(CompositeCommands.Teleop.startShooter(_shooterFlywheel, _notepath, 4000, 4000));
        _controller.povRight().onTrue(CompositeCommands.Teleop.startShooter(_shooterFlywheel, _notepath, 2000, 5000));
    }

    private void configureTestCommands()
    {
        DriverStation.silenceJoystickConnectionWarning(true);

        new JoystickButton(_testJoy, 1).onTrue(Commands.runOnce(() ->
        {
        }, _intake, _notepath, _shooterBed, _shooterFlywheel));
        new JoystickButton(_testJoy, 7).onTrue(ShooterFlywheelCommands.start(_shooterFlywheel, 3000, 3000));
        new JoystickButton(_testJoy, 8).onTrue(CompositeCommands.General.startNotepath(_shooterBed, _notepath, _shooterFlywheel));
        new JoystickButton(_testJoy, 9).onTrue(CompositeCommands.Autonomous.loadWhileStopped(_intake, _notepath));
        new JoystickButton(_testJoy, 10).onTrue(CompositeCommands.Autonomous.intakePickup(_intake, _notepath, _shooterBed));
        new JoystickButton(_testJoy, 11).onTrue(CompositeCommands.Autonomous.loadWhileStopped(_intake, _notepath));
        new JoystickButton(_testJoy, 12).onTrue(CompositeCommands.Autonomous.loadInMotion(_intake, _notepath));
    }

    private ChassisSpeeds getChassisSpeeds()
    {
        return _drive.getChassisSpeeds();
    }

    public Command getAutonomousCommand()
    {
        return null;
        // return AutoBuilder.buildAuto("New Auto");
        // return _dashboard.getAuto();
    }
}
