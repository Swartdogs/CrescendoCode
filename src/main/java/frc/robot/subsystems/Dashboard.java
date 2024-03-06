package frc.robot.subsystems;

import java.util.EnumSet;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleConsumer;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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

public class Dashboard extends SubsystemBase
{
    /*
     * Dashboard
     */

    // Field
    private final Field2d _field;

    // Color Box
    private final GenericEntry _allianceBox;
    private final GenericEntry _hasNote;

    // Swerve angles
    private final GenericEntry _frAngle;
    private final GenericEntry _flAngle;
    private final GenericEntry _brAngle;
    private final GenericEntry _blAngle;

    // Climb
    private final GenericEntry _leftHeight;
    private final GenericEntry _rightHeight;

    // Intake
    private final GenericEntry _intakeSpeed;

    // Notepath
    private final GenericEntry _notepathOutput;

    // Shooter bed
    private final GenericEntry _bedAngle;

    // Shooter flywheel
    private final GenericEntry _upperVelocity;
    private final GenericEntry _lowerVelocity;

    /*
     * Settings
     */

    // Drive
    private final GenericEntry _frOffset;
    private final GenericEntry _flOffset;
    private final GenericEntry _brOffset;
    private final GenericEntry _blOffset;

    // Intake
    private final GenericEntry _intakeInSpeed;
    private final GenericEntry _intakeOutSpeed;

    // Climb
    private final GenericEntry _climbLeftOffset;
    private final GenericEntry _climbRightOffset;
    private final GenericEntry _climbMinHeight;
    private final GenericEntry _climbMaxHeight;

    // Notepath
    private final GenericEntry _notepathShootSpeed;
    private final GenericEntry _pickupIntakeSpeed;
    private final GenericEntry _shooterIntakeSpeed;

    // Shooter Bed
    private final GenericEntry _shooterOffset;
    private final GenericEntry _bedMinimumAngle;
    private final GenericEntry _bedMaximumAngle;
    private final GenericEntry _bedPickupIntakeAngle;
    private final GenericEntry _bedPickupShooterAngle;

    // Shooter Flywheel
    private final GenericEntry _flywheelVelocityRange;
    private final GenericEntry _maxFlywheelSpeed;
    private final GenericEntry _flywheelIntakeSpeed;

    //Vision
    HttpCamera photonCamera;
    UsbCamera driverCamera;
    VideoSink videoSink;
    boolean showFrontCamera = true;


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
    private final SendableChooser<Integer>        _autoDelayChooser;
    private final LoggedDashboardChooser<Command> _autoChooser;

    public Dashboard(ShooterBed shooterBed, Notepath notepath, ShooterFlywheel shooterFlywheel, Drive drive, Intake intake, Climb climb)
    {
        _drive           = drive;
        _shooterBed      = shooterBed;
        _notepath        = notepath;
        _shooterFlywheel = shooterFlywheel;
        _intake          = intake;
        _climb           = climb;

        /*
         * DASHBOARD
         */
        var dashboard = Shuffleboard.getTab("Dashboard");

        photonCamera = new HttpCamera(Constants.Vision.PHOTON_CAMERA_NAME, Constants.Vision.PHOTON_CAMERA_URL, HttpCameraKind.kMJPGStreamer);
        photonCamera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

        driverCamera = CameraServer.startAutomaticCapture(Constants.Vision.DRIVER_CAMERA_NAME, 1);
        driverCamera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

        videoSink = CameraServer.addSwitchedCamera("Switched camera");

        // Camera Stream
        dashboard.add("Camera Stream", videoSink).withWidget(BuiltInWidgets.kCameraStream).withPosition(0, 0).withSize(15, 10)
                .withProperties(Map.of("Show crosshair", false, "Show controls", false));

        // CameraServer.addSwitchedCamera(getName());

        // Field
        _field = new Field2d();
        dashboard.add("Field", _field).withPosition(15, 0).withSize(15, 8);

        // Color box
        var colorBoxLayout = dashboard.getLayout("Color Box", BuiltInLayouts.kGrid).withPosition(15, 12).withSize(5, 3).withProperties(Map.of("Number of columns", 1, "Number of rows", 2, "Label position", "LEFT"));
        _allianceBox = colorBoxLayout.add("Alliance", false).withPosition(0, 0).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color when true", "blue", "Color when false", "red")).getEntry();
        _hasNote     = colorBoxLayout.add("Has Note", false).withPosition(0, 1).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color when true", "green", "Color when false", "red")).getEntry();

        // Swerve Module Angles
        var driveSettingsLayout = dashboard.getLayout("Swerve Angles", BuiltInLayouts.kGrid).withPosition(15, 8).withSize(5, 4).withProperties(Map.of("Number of columns", 2, "Number of rows", 2));
        _flAngle = driveSettingsLayout.add("FL Angle", 0.0).withPosition(0, 0).withWidget(BuiltInWidgets.kTextView).getEntry();
        _frAngle = driveSettingsLayout.add("FR Angle", 0.0).withPosition(1, 0).withWidget(BuiltInWidgets.kTextView).getEntry();
        _blAngle = driveSettingsLayout.add("BL Angle", 0.0).withPosition(0, 1).withWidget(BuiltInWidgets.kTextView).getEntry();
        _brAngle = driveSettingsLayout.add("BR Angle", 0.0).withPosition(1, 1).withWidget(BuiltInWidgets.kTextView).getEntry();

        // Height, Intake and Speed
        var heightAndIntakeLayout = dashboard.getLayout("Climb and Intake", BuiltInLayouts.kGrid).withPosition(0, 10).withSize(15, 5).withProperties(Map.of("Number of columns", 3, "Number of rows", 1));
        _leftHeight  = heightAndIntakeLayout.add("Left Height", 0).withPosition(0, 0).withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("Orientation", "VERTICAL", "Min", 0, "Max", 25)).getEntry();
        _intakeSpeed = heightAndIntakeLayout.add("Intake Speed", 0).withPosition(1, 0).withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("Min", -1, "Max", 1)).getEntry();
        _rightHeight = heightAndIntakeLayout.add("Right Height", 0).withPosition(2, 0).withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("Orientation", "VERTICAL", "Min", 0, "Max", 25)).getEntry();

        // Notepath
        _notepathOutput = dashboard.add("Notepath", 0).withPosition(26, 8).withSize(6, 7).withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("Min", -1, "Max", 1)).getEntry();

        // Shooter bed
        _bedAngle = dashboard.add("Bed angle", 0).withPosition(32, 8).withSize(3, 7).withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("Orientation", "VERTICAL", "Min", 0, "Max", 90)).getEntry();

        // Shooter flywheel
        var shooterFlywheelLayout = dashboard.getLayout("Shooter flywheel", BuiltInLayouts.kGrid).withPosition(20, 8).withSize(6, 7).withProperties(Map.of("Number of columns", 1, "Number of rows", 2, "Label position", "HIDDEN"));
        _upperVelocity = shooterFlywheelLayout.add("Upper Velocity", 0).withPosition(0, 0).withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("Min", -6000, "Max", 6000)).getEntry();
        _lowerVelocity = shooterFlywheelLayout.add("Lower Velocity", 0).withPosition(0, 1).withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("Min", -6000, "Max", 6000)).getEntry();

        // Autonomous Options
        var autonomousLayout = dashboard.getLayout("Autonomous", BuiltInLayouts.kGrid).withPosition(30, 0).withSize(9, 8).withProperties(Map.of("Number of columns", 1, "Number of rows", 4, "Label position", "LEFT"));

        // Autonomous delay chooser setup
        _autoDelayChooser = new SendableChooser<>();
        _autoDelayChooser.setDefaultOption("0", 0);
        _autoDelayChooser.addOption("1", 1);
        _autoDelayChooser.addOption("2", 2);
        _autoDelayChooser.addOption("3", 3);
        _autoDelayChooser.addOption("4", 4);
        _autoDelayChooser.addOption("5", 5);

        NamedCommands.registerCommand("Set Pose to Middle Side", Commands.runOnce(() -> _drive.setPose(new Pose2d(1.39, 5.56, new Rotation2d()))));
        NamedCommands.registerCommand("Set Pose to Source Side", Commands.runOnce(() -> _drive.setPose(new Pose2d(0.79, 4.23, new Rotation2d(-24.44)))));
        NamedCommands.registerCommand("Set Pose to Amp Side ", Commands.runOnce(() -> _drive.setPose(new Pose2d(0.76, 6.77, new Rotation2d(10.19)))));

        // NamedCommands.registerCommand("Load in Motion", CompositeCommands.Autonomous.loadInMotion(intake, notepath));
        // NamedCommands.registerCommand("Load While Stopped", CompositeCommands.Autonomous.loadWhileStopped(intake, notepath));
        //NamedCommands.registerCommand("Stop Intake", CompositeCommands.General.stopIntaking(_intake, _notepath));
        NamedCommands.registerCommand("Auto Delay", Commands.defer(() -> Commands.waitSeconds(autoDelayTime()), Set.of()));
        NamedCommands.registerCommand("Load", CompositeCommands.Autonomous.load(_notepath)); //TODO: Does this need to be registered as a deferred instant if the contents are? 
        NamedCommands.registerCommand("Intake Pickup", CompositeCommands.Autonomous.intakePickup(_intake, _notepath, _shooterBed));
        NamedCommands.registerCommand("Start Notepath", CompositeCommands.General.startNotepath(_shooterBed, _notepath, _shooterFlywheel)); //Notepath sequence, shuts off after sensor not tripped
        
        //Named Commands for starting shooter, intake, and notepath, does not shut off
        NamedCommands.registerCommand("Start Shooter", ShooterFlywheelCommands.start(_shooterFlywheel, 3000, 3000));
        NamedCommands.registerCommand("Start Intake", IntakeCommands.start(_intake));
        NamedCommands.registerCommand("Notepath On ", NotepathCommands.intakeLoad(_notepath)); //TODO: Rename other to notepath sequence overall check
        NamedCommands.registerCommand("Notepath Off", NotepathCommands.stop(_notepath));

        _autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        autonomousLayout.add("Delay Options", _autoDelayChooser).withPosition(0, 0).withWidget(BuiltInWidgets.kSplitButtonChooser);
        autonomousLayout.add("Autonomous Path", _autoChooser.getSendableChooser()).withPosition(0, 1).withWidget(BuiltInWidgets.kComboBoxChooser);

        /*
         * SETTINGS
         */

        var settingsTab = Shuffleboard.getTab("Settings");

        var outerLayout = settingsTab.getLayout("Settings", BuiltInLayouts.kGrid).withPosition(0, 0).withSize(40, 10).withProperties(Map.of("Number of columns", 6, "Number of rows", 1, "Label position", "TOP"));

        var driveSettings = outerLayout.getLayout("Drive", BuiltInLayouts.kList).withPosition(0, 0).withProperties(Map.of("label position", "LEFT"));
        _flOffset = driveSettings.add("FL Offset", Constants.Drive.MODULE_FL_OFFSET.getDegrees()).withWidget(BuiltInWidgets.kTextView).getEntry();
        _frOffset = driveSettings.add("FR Offset", Constants.Drive.MODULE_FR_OFFSET.getDegrees()).withWidget(BuiltInWidgets.kTextView).getEntry();
        _blOffset = driveSettings.add("BL Offset", Constants.Drive.MODULE_BL_OFFSET.getDegrees()).withWidget(BuiltInWidgets.kTextView).getEntry();
        _brOffset = driveSettings.add("BR Offset", Constants.Drive.MODULE_BR_OFFSET.getDegrees()).withWidget(BuiltInWidgets.kTextView).getEntry();

        var intakeSettings = outerLayout.getLayout("Intake", BuiltInLayouts.kList).withPosition(1, 0).withProperties(Map.of("label position", "LEFT"));
        _intakeInSpeed  = intakeSettings.add("Intake In Speed", Constants.Intake.INTAKE_DEFAULT_PERCENT_OUTPUT).withWidget(BuiltInWidgets.kTextView).getEntry();
        _intakeOutSpeed = intakeSettings.add("Intake Out Speed", Constants.Intake.OUTTAKE_DEFAULT_PERCENT_OUTPUT).withWidget(BuiltInWidgets.kTextView).getEntry();

        var climbSettings = outerLayout.getLayout("Climb", BuiltInLayouts.kList).withPosition(2, 0).withProperties(Map.of("label position", "LEFT"));
        _climbLeftOffset  = climbSettings.add("Left Offset", Constants.Climb.LEFT_ZERO_OFFSET).withWidget(BuiltInWidgets.kTextView).getEntry();
        _climbRightOffset = climbSettings.add("Right Offset", Constants.Climb.RIGHT_ZERO_OFFSET).withWidget(BuiltInWidgets.kTextView).getEntry();
        _climbMinHeight   = climbSettings.add("Arm Minimum Height", Constants.Climb.MIN_EXTENSION).withWidget(BuiltInWidgets.kTextView).getEntry();
        _climbMaxHeight   = climbSettings.add("Arm Maximum Height", Constants.Climb.MAX_EXTENSION).withWidget(BuiltInWidgets.kTextView).getEntry();

        var notepathSettings = outerLayout.getLayout("Notepath", BuiltInLayouts.kList).withPosition(3, 0).withProperties(Map.of("label position", "LEFT"));
        _notepathShootSpeed = notepathSettings.add("Note Shoot Speed", Constants.Notepath.NOTEPATH_FEED_PERCENT_OUTPUT).withWidget(BuiltInWidgets.kTextView).getEntry();
        _pickupIntakeSpeed  = notepathSettings.add("Pickup Intake Speed", Constants.Notepath.NOTEPATH_INTAKE_PICKUP_PERCENT_OUTPUT).withWidget(BuiltInWidgets.kTextView).getEntry();
        _shooterIntakeSpeed = notepathSettings.add("Shooter Intake Speed", Constants.Notepath.NOTEPATH_SHOOTER_PICKUP_PERCENT_OUTPUT).withWidget(BuiltInWidgets.kTextView).getEntry();

        var shooterBedSettings = outerLayout.getLayout("Shooter Bed", BuiltInLayouts.kList).withPosition(4, 0).withProperties(Map.of("label position", "LEFT"));
        _bedMinimumAngle       = shooterBedSettings.add("Bed Minimum Angle", Constants.ShooterBed.MIN_BED_ANGLE.getDegrees()).withWidget(BuiltInWidgets.kTextView).getEntry();
        _bedMaximumAngle       = shooterBedSettings.add("Bed Maximum Angle", Constants.ShooterBed.MAX_BED_ANGLE.getDegrees()).withWidget(BuiltInWidgets.kTextView).getEntry();
        _shooterOffset         = shooterBedSettings.add("Shooter Offset", Constants.ShooterBed.BED_ANGLE_OFFSET.getDegrees()).withWidget(BuiltInWidgets.kTextView).getEntry();
        _bedPickupIntakeAngle  = shooterBedSettings.add("Bed Pickup Intake Angle", Constants.ShooterBed.BED_INTAKE_PICKUP_ANGLE.getDegrees()).withWidget(BuiltInWidgets.kTextView).getEntry();
        _bedPickupShooterAngle = shooterBedSettings.add("Bed Pickup Shooter Angle", Constants.ShooterBed.BED_SHOOTER_PICKUP_ANGLE.getDegrees()).withWidget(BuiltInWidgets.kTextView).getEntry();

        var shooterFlywheelSettings = outerLayout.getLayout("Shooter Flywheel", BuiltInLayouts.kList).withPosition(5, 0).withProperties(Map.of("label position", "LEFT"));
        _flywheelVelocityRange = shooterFlywheelSettings.add("Flywheel Velocity Range", Constants.ShooterFlywheel.VELOCITY_RANGE).withWidget(BuiltInWidgets.kTextView).getEntry();
        _maxFlywheelSpeed      = shooterFlywheelSettings.add("Max Flywheel Speed", Constants.ShooterFlywheel.MAX_FLYWHEEL_SPEED).withWidget(BuiltInWidgets.kTextView).getEntry();
        _flywheelIntakeSpeed   = shooterFlywheelSettings.add("Flywheel Intake Speed", Constants.ShooterFlywheel.FLYWHEEL_INTAKE_SPEED).withWidget(BuiltInWidgets.kTextView).getEntry();

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
        initializeSetting("Climb Left Offset", Constants.Climb.LEFT_ZERO_OFFSET, _climbLeftOffset, value ->
        {
            _climb.setLeftOffset(value);
        });

        initializeSetting("Climb Right Offset", Constants.Climb.RIGHT_ZERO_OFFSET, _climbRightOffset, value ->
        {
            _climb.setRightOffset(value);
        });

        initializeSetting("Climb Minimum Height", Constants.Climb.MIN_EXTENSION, _climbMinHeight, value ->
        {
            _climb.setMinExtension(value);
        });

        initializeSetting("Climb Maximum Height", Constants.Climb.MAX_EXTENSION, _climbMaxHeight, value ->
        {
            _climb.setMaxExtension(value);
        });

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
        initializeSetting("Shooter Bed Offset", Constants.ShooterBed.BED_ANGLE_OFFSET.getDegrees(), _shooterOffset, value ->
        {
            _shooterBed.setAngleOffset(Rotation2d.fromDegrees(value));
        });

        initializeSetting("Bed Minimum Angle", Constants.ShooterBed.MIN_BED_ANGLE.getDegrees(), _bedMinimumAngle, value ->
        {
            _shooterBed.setMinAngle(Rotation2d.fromDegrees(value));
        });

        initializeSetting("Bed Maximum Angle", Constants.ShooterBed.MAX_BED_ANGLE.getDegrees(), _bedMaximumAngle, value ->
        {
            _shooterBed.setMaxAngle(Rotation2d.fromDegrees(value));
        });

        initializeSetting("Bed Intake Pickup Angle", Constants.ShooterBed.BED_INTAKE_PICKUP_ANGLE.getDegrees(), _bedPickupIntakeAngle, value ->
        {
            _shooterBed.setIntakeLoadAngle(Rotation2d.fromDegrees(value));
        });

        initializeSetting("Bed Shooter Pickup Angle", Constants.ShooterBed.BED_SHOOTER_PICKUP_ANGLE.getDegrees(), _bedPickupShooterAngle, value ->
        {
            _shooterBed.setShooterLoadAngle(Rotation2d.fromDegrees(value));
        });

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

      public void toggle() {
        showFrontCamera = !showFrontCamera;
        if(showFrontCamera) {
        videoSink.setSource(photonCamera);
        }
        else {
        videoSink.setSource(driverCamera);
        }
    }

    public void initializeSetting(String key, double defaultValue, GenericEntry entry, DoubleConsumer consumer)
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
        return _autoDelayChooser.getSelected();
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
