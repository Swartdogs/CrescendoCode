package frc.robot.subsystems;

import java.util.EnumSet;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleConsumer;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.CompositeCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.NotepathCommands;
import frc.robot.commands.ShooterFlywheelCommands;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.notepath.Notepath;
import frc.robot.subsystems.shooter.ShooterBed;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.util.DeferredInstantCommand;
import frc.robot.util.Utilities;

public class Dashboard extends SubsystemBase
{
    /*
     * Dashboard
     */

    // Field
    private final Field2d _field;

    // Color Box
    private final NetworkTableEntry _allianceBox;
    private final NetworkTableEntry _hasNote;

    // Swerve angles
    private final NetworkTableEntry _frAngle;
    private final NetworkTableEntry _flAngle;
    private final NetworkTableEntry _brAngle;
    private final NetworkTableEntry _blAngle;

    // Climb
    private final NetworkTableEntry _leftHeight;
    private final NetworkTableEntry _rightHeight;

    // Intake
    private final NetworkTableEntry _intakeSpeed;

    // Notepath
    private final NetworkTableEntry _notepathOutput;

    // Shooter bed
    private final NetworkTableEntry _bedAngle;

    // Shooter flywheel
    private final NetworkTableEntry _upperVelocity;
    private final NetworkTableEntry _lowerVelocity;

    /*
     * Settings
     */

    // Drive
    private final NetworkTableEntry _frOffset;
    private final NetworkTableEntry _flOffset;
    private final NetworkTableEntry _brOffset;
    private final NetworkTableEntry _blOffset;

    // Intake
    private final NetworkTableEntry _intakeInSpeed;
    private final NetworkTableEntry _intakeOutSpeed;

    // Climb
    @SuppressWarnings("unused")
    private final NetworkTableEntry _climbLeftOffset;
    @SuppressWarnings("unused")
    private final NetworkTableEntry _climbRightOffset;
    @SuppressWarnings("unused")
    private final NetworkTableEntry _climbMinHeight;
    @SuppressWarnings("unused")
    private final NetworkTableEntry _climbMaxHeight;

    // Notepath
    private final NetworkTableEntry _notepathShootSpeed;
    private final NetworkTableEntry _pickupIntakeSpeed;
    private final NetworkTableEntry _shooterIntakeSpeed;

    // Shooter Bed
    @SuppressWarnings("unused")
    private final NetworkTableEntry _shooterOffset;
    @SuppressWarnings("unused")
    private final NetworkTableEntry _bedMinimumAngle;
    @SuppressWarnings("unused")
    private final NetworkTableEntry _bedMaximumAngle;
    @SuppressWarnings("unused")
    private final NetworkTableEntry _bedPickupIntakeAngle;
    @SuppressWarnings("unused")
    private final NetworkTableEntry _bedPickupShooterAngle;

    // Shooter Flywheel
    private final NetworkTableEntry _flywheelVelocityRange;
    private final NetworkTableEntry _maxFlywheelSpeed;
    private final NetworkTableEntry _flywheelIntakeSpeed;

    // Vision
    private final UsbCamera   _shooterCamera;
    private final UsbCamera   _driverCamera;
    private final MjpegServer _switchedCamera;
    private boolean           _showDriverCamera = true;

    /*
     * Subsystems
     */
    private final ShooterBed      _shooterBed;
    private final Notepath        _notepath;
    private final Intake          _intake;
    private final ShooterFlywheel _shooterFlywheel;
    private final Drive           _drive;
    private final Climb           _climb;

    /*
     * SendableChoosers for Autonomous options
     */
    private final LoggedDashboardChooser<Integer> _autoDelayChooser;
    private final LoggedDashboardChooser<Command> _autoChooser;

    public Dashboard(ShooterBed shooterBed, Notepath notepath, ShooterFlywheel shooterFlywheel, Drive drive, Intake intake, Climb climb)
    {
        _drive           = drive;
        _shooterBed      = shooterBed;
        _notepath        = notepath;
        _shooterFlywheel = shooterFlywheel;
        _intake          = intake;
        _climb           = climb;

        _driverCamera = CameraServer.startAutomaticCapture(Constants.Vision.DRIVER_CAMERA_NAME, 0);
        _driverCamera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

        _shooterCamera = CameraServer.startAutomaticCapture(Constants.Vision.SHOOTER_CAMERA_NAME, 1);
        _shooterCamera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

        _switchedCamera = CameraServer.addSwitchedCamera("Switched camera");
        _switchedCamera.setSource(_shooterCamera);

        /*
         * DASHBOARD
         */
        var dashboard = NetworkTableInstance.getDefault().getTable("Shuffleboard/Dashboard");
        // Field
        _field = new Field2d();
        SmartDashboard.putData("Shuffleboard/Dashboard/Field", _field);

        // Color box
        _allianceBox = dashboard.getEntry("Color Box/Alliance");
        _hasNote     = dashboard.getEntry("Color Box/Has Note");

        // Swerve Module Angles
        _flAngle = dashboard.getEntry("Swerve Angles/FL Angle");
        _frAngle = dashboard.getEntry("Swerve Angles/FR Angle");
        _blAngle = dashboard.getEntry("Swerve Angles/BL Angle");
        _brAngle = dashboard.getEntry("Swerve Angles/BR Angle");

        // Height, Intake and Speed
        _leftHeight  = dashboard.getEntry("Climb and Intake/Left Height");
        _intakeSpeed = dashboard.getEntry("Climb and Intake/Intake Speed");
        _rightHeight = dashboard.getEntry("Climb and Intake/Right Height");

        // Notepath
        _notepathOutput = dashboard.getEntry("Notepath");

        // Shooter bed
        _bedAngle = dashboard.getEntry("Bed angle");

        // Shooter flywheel
        _upperVelocity = dashboard.getEntry("Shooter flywheel/Upper Velocity");
        _lowerVelocity = dashboard.getEntry("Shooter flywheel/Lower Velocity");

        // Autonomous delay chooser setup
        var delayChooser = new SendableChooser<Integer>();
        delayChooser.setDefaultOption("0", 0);
        delayChooser.addOption("1", 1);
        delayChooser.addOption("2", 2);
        delayChooser.addOption("3", 3);
        delayChooser.addOption("4", 4);
        delayChooser.addOption("5", 5);
        _autoDelayChooser = new LoggedDashboardChooser<>("Auto Delay", delayChooser);

        // START UP
        NamedCommands.registerCommand("Set Pose to Middle Side", Commands.runOnce(() -> _drive.setPose(Utilities.getAutoPose(new Pose2d(1.39, 5.56, new Rotation2d())))));
        NamedCommands.registerCommand("Set Pose to Source Side", Commands.runOnce(() -> _drive.setPose(Utilities.getAutoPose(new Pose2d(0.79, 4.23, Rotation2d.fromDegrees(-24.44))))));
        NamedCommands.registerCommand("Set Pose to Amp Side", Commands.runOnce(() -> _drive.setPose(Utilities.getAutoPose(new Pose2d(0.76, 6.77, Rotation2d.fromDegrees(10.19))))));
        NamedCommands.registerCommand("Auto Delay", Commands.defer(() -> Commands.waitSeconds(autoDelayTime()), Set.of()));

        NamedCommands.registerCommand("Set Shooter Angle", CompositeCommands.Autonomous.setBedAngle(_shooterBed, 38.8));
        NamedCommands.registerCommand("Set Shooter Angle Note 2", CompositeCommands.Autonomous.setBedAngle(_shooterBed, 34));
        NamedCommands.registerCommand("Set Shooter Angle Note 1", CompositeCommands.Autonomous.setBedAngle(_shooterBed, 28.3));
        NamedCommands.registerCommand("Podium Shot Angle", CompositeCommands.Autonomous.setBedAngle(_shooterBed, 31.5));

        NamedCommands.registerCommand("Set Inner Note Speed", CompositeCommands.Autonomous.startShooter(shooterFlywheel, 4000, 4000));
        NamedCommands.registerCommand("Set Initial Speed", CompositeCommands.Autonomous.startShooter(shooterFlywheel, 4500, 4500));
        NamedCommands.registerCommand("Set Speed 5000", CompositeCommands.Autonomous.startShooter(shooterFlywheel, 5000, 5000));
        NamedCommands.registerCommand("Set Shooter Max Speed", CompositeCommands.Autonomous.startShooter(shooterFlywheel, 5800, 5800));
        NamedCommands.registerCommand("Set Bed Angle", CompositeCommands.Autonomous.setBedAngle(_shooterBed, 52.5));
        NamedCommands.registerCommand("Set Bed Angle Intake", CompositeCommands.Autonomous.setBedAngle(shooterBed, 65.1));

        NamedCommands.registerCommand("Load", CompositeCommands.Autonomous.load(_notepath)); // TODO: Does this need to be registered as a deferred instant if the contents
                                                                                             // // // are?
        NamedCommands.registerCommand("Intake Pickup", CompositeCommands.Autonomous.intakePickup(_intake, _notepath, _shooterBed));
        NamedCommands.registerCommand("Start Notepath", CompositeCommands.Autonomous.startNotepath(_notepath)); // Notepath sequence, shuts off after sensor not tripped

        // Named Commands for starting shooter, intake, and notepath, does not shut off
        NamedCommands.registerCommand("Start Shooter", CompositeCommands.Autonomous.startShooter(_shooterFlywheel, 4000, 4000));
        NamedCommands.registerCommand("Start Intake", IntakeCommands.start(_intake));
        NamedCommands.registerCommand("Notepath On", new DeferredInstantCommand(() -> NotepathCommands.intakeLoad(_notepath))); // TODO: Rename other to notepath sequence overall check
        NamedCommands.registerCommand("Notepath Off", NotepathCommands.stop(_notepath));
        NamedCommands.registerCommand("Stop Intake", IntakeCommands.stop(intake));
        NamedCommands.registerCommand("Stop Shooter", ShooterFlywheelCommands.stop(shooterFlywheel));

        _autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        SmartDashboard.putData("Shuffleboard/Dashboard/Autonomous/Delay Options", _autoDelayChooser.getSendableChooser());
        SmartDashboard.putData("Shuffleboard/Dashboard/Autonomous/Autonomous Path", _autoChooser.getSendableChooser());

        /*
         * SETTINGS
         */

        var settings = NetworkTableInstance.getDefault().getTable("Shuffleboard/Settings/Settings");

        // Drive
        _flOffset = settings.getEntry("Drive/FL Offset");
        _frOffset = settings.getEntry("Drive/FR Offset");
        _blOffset = settings.getEntry("Drive/BL Offset");
        _brOffset = settings.getEntry("Drive/BR Offset");

        // Intake
        _intakeInSpeed  = settings.getEntry("Intake/Intake In Speed");
        _intakeOutSpeed = settings.getEntry("Intake/Intake Out Speed");

        // Climb
        _climbLeftOffset  = settings.getEntry("Climb/Left Offset");
        _climbRightOffset = settings.getEntry("Climb/Right Offset");
        _climbMinHeight   = settings.getEntry("Climb/Arm Minimum Height");
        _climbMaxHeight   = settings.getEntry("Climb/Arm Maximum Height");

        // Notepath
        _notepathShootSpeed = settings.getEntry("Notepath/Note Shoot Speed");
        _pickupIntakeSpeed  = settings.getEntry("Notepath/Pickup Intake Speed");
        _shooterIntakeSpeed = settings.getEntry("Notepath/Shooter Intake Speed");

        // Shooter Bed
        _bedMinimumAngle       = settings.getEntry("Shooter Bed/Bed Minimum Angle");
        _bedMaximumAngle       = settings.getEntry("Shooter Bed/Bed Maximum Angle");
        _shooterOffset         = settings.getEntry("Shooter Bed/Shooter Offset");
        _bedPickupIntakeAngle  = settings.getEntry("Shooter Bed/Bed Pickup Intake Angle");
        _bedPickupShooterAngle = settings.getEntry("Shooter Bed/Bed Pickup Shooter Angle");

        // Shooter Flywheel
        _flywheelVelocityRange = settings.getEntry("Shooter Flywheel/Flywheel Velocity Range");
        _maxFlywheelSpeed      = settings.getEntry("Shooter Flywheel/Max Flywheel Speed");
        _flywheelIntakeSpeed   = settings.getEntry("Shooter Flywheel/Flywheel Intake Speed");

        // Drive
        initializeSetting("FL Offset", Constants.Drive.MODULE_FL_OFFSET.getDegrees(), _flOffset, value ->
        {
            _drive.setModuleAbsoluteEncoderOffset(0, Rotation2d.fromDegrees(value));
        });

        initializeSetting("FR Offset", Constants.Drive.MODULE_FR_OFFSET.getDegrees(), _frOffset, value ->
        {
            _drive.setModuleAbsoluteEncoderOffset(1, Rotation2d.fromDegrees(value));
        });

        initializeSetting("BL Offset", Constants.Drive.MODULE_BL_OFFSET.getDegrees(), _blOffset, value ->
        {
            _drive.setModuleAbsoluteEncoderOffset(2, Rotation2d.fromDegrees(value));
        });

        initializeSetting("BR Offset", Constants.Drive.MODULE_BR_OFFSET.getDegrees(), _brOffset, value ->
        {
            _drive.setModuleAbsoluteEncoderOffset(3, Rotation2d.fromDegrees(value));
        });

        // Intake
        initializeSetting("Intake In Speed", Constants.Intake.INTAKE_DEFAULT_PERCENT_OUTPUT, _intakeInSpeed, value ->
        {
            _intake.setInSpeed(value);
        });

        initializeSetting("Intake Out Speed", Constants.Intake.OUTTAKE_DEFAULT_PERCENT_OUTPUT, _intakeOutSpeed, value ->
        {
            _intake.setOutSpeed(value);
        });

        // Climb
        // initializeSetting("Climb Left Offset", Constants.Climb.LEFT_ZERO_OFFSET,
        // _climbLeftOffset, value ->
        // {
        // _climb.setLeftOffset(value);
        // });

        // initializeSetting("Climb Right Offset", Constants.Climb.RIGHT_ZERO_OFFSET,
        // _climbRightOffset, value ->
        // {
        // _climb.setRightOffset(value);
        // });

        // initializeSetting("Climb Minimum Height", Constants.Climb.MIN_EXTENSION,
        // _climbMinHeight, value ->
        // {
        // _climb.setMinExtension(value);
        // });

        // initializeSetting("Climb Maximum Height", Constants.Climb.LEFT_MAX_EXTENSION,
        // _climbMaxHeight, value ->
        // {
        // _climb.setMaxExtension(value);
        // });

        // Notepath
        initializeSetting("Notepath Shoot Speed", Constants.Notepath.NOTEPATH_FEED_PERCENT_OUTPUT, _notepathShootSpeed, value ->
        {
            _notepath.setFeedSpeed(value);
        });

        initializeSetting("Pickup Intake Speed", Constants.Notepath.NOTEPATH_INTAKE_PICKUP_PERCENT_OUTPUT, _pickupIntakeSpeed, value ->
        {
            _notepath.setIntakeLoadSpeed(value);
        });

        initializeSetting("Shooter Intake Speed", Constants.Notepath.NOTEPATH_SHOOTER_PICKUP_PERCENT_OUTPUT, _shooterIntakeSpeed, value ->
        {
            _notepath.setShooterLoadSpeed(value);
        });

        // Shooter bed
        // initializeSetting("Shooter Bed Offset",
        // Constants.ShooterBed.BED_ANGLE_OFFSET.getDegrees(), _shooterOffset, value ->
        // {
        // _shooterBed.setAngleOffset(Rotation2d.fromDegrees(value));
        // });

        // initializeSetting("Bed Minimum Angle",
        // Constants.ShooterBed.MIN_BED_ANGLE.getDegrees(), _bedMinimumAngle, value ->
        // {
        // _shooterBed.setMinAngle(Rotation2d.fromDegrees(value));
        // });

        // initializeSetting("Bed Maximum Angle",
        // Constants.ShooterBed.MAX_BED_ANGLE.getDegrees(), _bedMaximumAngle, value ->
        // {
        // _shooterBed.setMaxAngle(Rotation2d.fromDegrees(value));
        // });

        // initializeSetting("Bed Intake Pickup Angle",
        // Constants.ShooterBed.BED_INTAKE_PICKUP_ANGLE.getDegrees(),
        // _bedPickupIntakeAngle, value ->
        // {
        // _shooterBed.setIntakeLoadAngle(Rotation2d.fromDegrees(value));
        // });

        // initializeSetting("Bed Shooter Pickup Angle",
        // Constants.ShooterBed.BED_SHOOTER_PICKUP_ANGLE.getDegrees(),
        // _bedPickupShooterAngle, value ->
        // {
        // _shooterBed.setShooterLoadAngle(Rotation2d.fromDegrees(value));
        // });

        // Shooter flywheel
        initializeSetting("Shooter Velocity Range", Constants.ShooterFlywheel.VELOCITY_RANGE, _flywheelVelocityRange, value ->
        {
            _shooterFlywheel.setVelocityRange(value);
        });

        initializeSetting("Max Flywheel Speed", Constants.ShooterFlywheel.MAX_FLYWHEEL_SPEED, _maxFlywheelSpeed, value ->
        {
            _shooterFlywheel.setMaxSpeed(value);
        });

        initializeSetting("Flywheel Intake Speed", Constants.ShooterFlywheel.FLYWHEEL_INTAKE_SPEED, _flywheelIntakeSpeed, value ->
        {
            _shooterFlywheel.setIntakeSpeed(value);
        });
    }

    public void toggleCamera()
    {
        if (_showDriverCamera)
        {
            _switchedCamera.setSource(_shooterCamera);
            _showDriverCamera = false;
        }
        else
        {
            _switchedCamera.setSource(_driverCamera);
            _showDriverCamera = true;
        }
    }

    public boolean isDriverCamera()
    {
        return true;
    }

    public void initializeSetting(String key, double defaultValue, NetworkTableEntry entry, DoubleConsumer consumer)
    {
        NetworkTableInstance.getDefault().addListener(entry, EnumSet.of(NetworkTableEvent.Kind.kValueRemote), event ->
        {
            consumer.accept(entry.getDouble(defaultValue));
            Preferences.setDouble(key, entry.getDouble(defaultValue));
        });

        double value = defaultValue;

        if (Preferences.containsKey(key))
        {
            value = Preferences.getDouble(key, defaultValue);
        }
        else
        {
            Preferences.initDouble(key, defaultValue);
        }

        consumer.accept(value);
        entry.setDouble(value);
    }

    public Command getAuto()
    {
        return _autoChooser.get();
    }

    public Integer autoDelayTime()
    {
        return _autoDelayChooser.get();
    }

    @Override
    public void periodic()
    {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        SwerveModuleState[] moduleStates = _drive.getModuleStates();

        // Color box
        _allianceBox.setBoolean(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue);
        _hasNote.setBoolean(_notepath.hasNote());

        // Swerve angles
        _flAngle.setDouble(Double.parseDouble(String.format("%6.2f", moduleStates[0].angle.getDegrees())));
        _frAngle.setDouble(Double.parseDouble(String.format("%6.2f", moduleStates[1].angle.getDegrees())));
        _brAngle.setDouble(Double.parseDouble(String.format("%6.2f", moduleStates[2].angle.getDegrees())));
        _blAngle.setDouble(Double.parseDouble(String.format("%6.2f", moduleStates[3].angle.getDegrees())));

        // Field
        _field.setRobotPose(_drive.getPose());

        // Climb
        _leftHeight.setDouble(Double.parseDouble(String.format("%6.2f", _climb.getLeftExtension())));
        _rightHeight.setDouble(Double.parseDouble(String.format("%6.2f", _climb.getRightExtension())));

        // Intake
        _intakeSpeed.setDouble(Double.parseDouble(String.format("%6.2f", _intake.getSpeed())));

        // Notepath
        _notepathOutput.setDouble(Double.parseDouble(String.format("%6.2f", _notepath.getSpeed())));

        // Shooter bed
        _bedAngle.setDouble(Double.parseDouble(String.format("%6.2f", _shooterBed.getBedAngle().getDegrees())));

        // Shooter flywheel
        _upperVelocity.setDouble(Double.parseDouble(String.format("%6.2f", _shooterFlywheel.getUpperVelocity())));
        _lowerVelocity.setDouble(Double.parseDouble(String.format("%6.2f", _shooterFlywheel.getLowerVelocity())));
    }
}
