package frc.robot.subsystems;

import java.util.EnumSet;
import java.util.Map;
import java.util.function.DoubleConsumer;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.notepath.Notepath;
import frc.robot.subsystems.shooter.ShooterBed;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.ShooterFlywheel;

public class Dashboard extends SubsystemBase
{
    private static Dashboard _instance;
    private Field2d          _field;

    // Widgets for the dashboard
    private GenericEntry          _allianceBox;
    private GenericEntry          _HasNote;
    private GenericEntry          _intakeBox;
    private GenericEntry          _frAngle;
    private GenericEntry          _flAngle;
    private GenericEntry          _brAngle;
    private GenericEntry          _blAngle;
    private GenericEntry          _LeftHeight;
    private GenericEntry          _Rightheight;
    private GenericEntry          _speedDial;
    private GenericEntry          _IntakeSpeed;
    private GenericEntry          _autonomousSplitButton;
    private GenericEntry          _autonomousComboBox;
    private GenericEntry          _notepathOutput;
    private GenericEntry          _bedAngle;
    private GenericEntry          _upperVelocity;
    private GenericEntry          _lowerVelocity;
    private GenericEntry          _frOffset;
    private GenericEntry          _flOffset;
    private GenericEntry          _brOffset;
    private GenericEntry          _blOffset;
    private GenericEntry          _IntakeInSpeed;
    private GenericEntry          _climberleftOffset;
    private GenericEntry          _climberRightOffset;
    private GenericEntry          _climberMinHeight;
    private GenericEntry          _NoteShootSpeed;
    private GenericEntry          _PickupIntakeSpeed;
    private GenericEntry          _ShooterIntakeSpeed;
    private GenericEntry          _ShooterOffset;
    private GenericEntry          _IntakeOutSpeed;
    private GenericEntry          _bedMinimumAngle;
    private GenericEntry          _bedMaximumAngle;
    private GenericEntry          _maxFlywheelVelocity;
    private GenericEntry          _climberMaxHeight;
    private final ShooterBed      _ShooterBed;
    private final Notepath        _notepath;
    private final Intake          _Intake;
    private final ShooterFlywheel _ShooterFlywheel;
    private final Drive           _drive;

    // SendableChoosers for Autonomous options
    private SendableChooser<Integer> _autoDelayChooser;

    public Dashboard(ShooterBed shooterBed, Notepath notepath, ShooterFlywheel ShooterFlywheel, Drive drive, Intake intake)
    {
        _drive           = drive;
        _ShooterBed      = shooterBed;
        _notepath        = notepath;
        _ShooterFlywheel = ShooterFlywheel;
        _Intake          = intake;

        var tab         = Shuffleboard.getTab("Dashboard");
        var SettingsTab = Shuffleboard.getTab("Settings");

        // Camera Stream
        tab.addCamera("Camera Stream", Constants.Vision.CAMERA_NAME, "mjpg:http://10.5.25.12:1181/?action=stream").withWidget("Camera Stream").withPosition(0, 0).withSize(15, 10)
                .withProperties(Map.of("Show crosshair", false, "Show controls", false));
        // Field
        _field = new Field2d();
        tab.add("field", _field).withPosition(15, 0).withSize(14, 8);

        // Alliance, Note, and Intake boxes
        var booleanBoxLayout = tab.getLayout("Color Box", BuiltInLayouts.kGrid).withPosition(15, 12).withSize(5, 3).withProperties(Map.of("Number of columns", 1, "Number of rows", 3, "Label position", "LEFT"));
        _allianceBox = booleanBoxLayout.add("Alliance", false).withPosition(0, 0).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color when true", "blue", "Color when false", "red")).getEntry();
        _HasNote     = booleanBoxLayout.add("Has Note", false).withPosition(0, 1).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color when true", "green", "Color when false", "red")).getEntry();

        // Swerve Module Angles
        var driveSettingsLayout = tab.getLayout("Drive Subsystem", BuiltInLayouts.kGrid).withPosition(15, 8).withSize(5, 4);
        _flAngle = driveSettingsLayout.add("FL Angle", 0.0).withPosition(0, 0).withSize(2, 2).withWidget(BuiltInWidgets.kTextView).getEntry();
        _frAngle = driveSettingsLayout.add("FR Angle", 0.0).withPosition(2, 0).withSize(2, 2).withWidget(BuiltInWidgets.kTextView).getEntry();
        _blAngle = driveSettingsLayout.add("BL Angle", 0.0).withPosition(0, 2).withSize(2, 2).withWidget(BuiltInWidgets.kTextView).getEntry();
        _brAngle = driveSettingsLayout.add("BR Angle", 0.0).withPosition(2, 2).withSize(2, 2).withWidget(BuiltInWidgets.kTextView).getEntry();

        // Height, Intake and Speed
        var heightAndIntakeLayout = tab.getLayout("Height and Intake", BuiltInLayouts.kGrid).withPosition(0, 10).withSize(15, 5).withProperties(Map.of("Number of columns", 3, "Number of rows", 1));
        _LeftHeight  = heightAndIntakeLayout.add("Left Height", 0).withSize(5, 4).withPosition(0, 0).withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("Orientation", "VERTICAL")).getEntry();
        _Rightheight = heightAndIntakeLayout.add("Right Height", 0).withWidget(BuiltInWidgets.kNumberBar).withPosition(2, 0).withProperties(Map.of("Orientation", "VERTICAL")).getEntry();
        _IntakeSpeed = heightAndIntakeLayout.add("Intake Speed", 0).withPosition(1, 0).withWidget(BuiltInWidgets.kNumberBar).getEntry();

        _notepathOutput = tab.add("Notepath output", 0).withSize(6, 7).withPosition(26, 8).withWidget(BuiltInWidgets.kNumberBar).getEntry();
        _bedAngle       = tab.add("Bed angle", 0).withSize(3, 7).withPosition(32, 8).withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("Orientation", "VERTICAL")).getEntry();

        var shooterFlywheel = tab.getLayout("Shooter flywheel", BuiltInLayouts.kGrid).withPosition(20, 8).withSize(6, 7).withProperties(Map.of("Number of columns", 1, "Number of rows", 2, "Label position", "HIDDEN"));;
        _upperVelocity = shooterFlywheel.add("Upper Velocity", 0).withPosition(0, 0).withSize(1, 1).withWidget(BuiltInWidgets.kNumberBar).getEntry();
        _lowerVelocity = shooterFlywheel.add("Lower Velocity", 0).withPosition(0, 1).withSize(1, 1).withWidget(BuiltInWidgets.kNumberBar).getEntry();
        // Autonomous Options
        var autonomousLayout = tab.getLayout("Autonomous", BuiltInLayouts.kGrid).withPosition(29, 0).withSize(10, 7).withProperties(Map.of("Number of columns", 1, "Number of rows", 4, "Label position", "LEFT"));

        // Autonomous delay chooser setup
        _autoDelayChooser = new SendableChooser<>();
        _autoDelayChooser.setDefaultOption("0", 0);
        _autoDelayChooser.addOption("1", 1);
        _autoDelayChooser.addOption("2", 2);
        _autoDelayChooser.addOption("3", 3);
        _autoDelayChooser.addOption("4", 4);
        _autoDelayChooser.addOption("5", 5);
        autonomousLayout.add("Delay Options", _autoDelayChooser).withPosition(0, 0).withSize(1, 1).withWidget(BuiltInWidgets.kSplitButtonChooser);

        // Autonomous start position chooser setup
        // _autoStartPositionChooser = new SendableChooser<>();
        // _autoStartPositionChooser.setDefaultOption("Middle",
        // DrivePosition.MiddleStart);
        // _autoStartPositionChooser.addOption("Substation Side",
        // DrivePosition.SubstationStart);
        // _autoStartPositionChooser.addOption("Wall Side", DrivePosition.WallStart);
        // autonomousLayout.add("Start Position", _autoStartPositionChooser)
        // .withPosition(0, 1)
        // .withSize(1, 1)
        // .withWidget(BuiltInWidgets.kComboBoxChooser);

        var outerLayout = SettingsTab.getLayout("Settings", BuiltInLayouts.kGrid).withPosition(0, 0).withSize(40, 10).withProperties(Map.of("Number of columns", 6, "Number of rows", 1, "Label position", "TOP"));

        var driveSettings = outerLayout.getLayout("Drive Subsystem", BuiltInLayouts.kList).withPosition(0, 0).withSize(8, 10).withProperties(Map.of("label position", "LEFT"));

        _flOffset = driveSettings.add("FL Offset", 0.0).withPosition(0, 0).withWidget(BuiltInWidgets.kTextView).getEntry();

        _frOffset = driveSettings.add("FR Offset", 0.0).withPosition(2, 0).withWidget(BuiltInWidgets.kTextView).getEntry();

        _blOffset = driveSettings.add("BL Offset", 0.0).withPosition(0, 2).withWidget(BuiltInWidgets.kTextView).getEntry();

        _brOffset = driveSettings.add("BR Offset", 0.0).withPosition(2, 2).withWidget(BuiltInWidgets.kTextView).getEntry();

        var IntakeSetting = outerLayout.getLayout("Intake", BuiltInLayouts.kList).withPosition(1, 0).withSize(7, 10).withProperties(Map.of("label position", "LEFT"));

        _IntakeInSpeed = IntakeSetting.add("Intake In Speed", 0.0).withPosition(0, 0).withWidget(BuiltInWidgets.kTextView).withPosition(0, 0).getEntry();

        _IntakeOutSpeed = IntakeSetting.add("Intake Out Speed", 0.0).withWidget(BuiltInWidgets.kTextView).withPosition(0, 1).getEntry();

        var climbSetting = outerLayout.getLayout("Climber", BuiltInLayouts.kList).withPosition(2, 0).withProperties(Map.of("label position", "LEFT")).withSize(7, 10);

        _climberleftOffset = climbSetting.add("Left Offset", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();

        _climberRightOffset = climbSetting.add("Right Offset", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();

        _climberMinHeight = climbSetting.add("Arm Minimum Height", 0.0).withPosition(0, 0).withWidget(BuiltInWidgets.kTextView).getEntry();

        _climberMaxHeight = climbSetting.add("Arm Maximum Height", 0.0).withPosition(2, 0).withWidget(BuiltInWidgets.kTextView).getEntry();

        var noteSetting = outerLayout.getLayout("Notepath", BuiltInLayouts.kList).withPosition(3, 0).withSize(8, 10).withProperties(Map.of("label position", "LEFT"));

        _NoteShootSpeed = noteSetting.add("Note Shoot Speed", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();

        _PickupIntakeSpeed = noteSetting.add("Pickup Intake Speed", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();

        _ShooterIntakeSpeed = noteSetting.add("Shooter Intake Speed", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();

        var shooterBedSetting = outerLayout.getLayout("Shooter Bed", BuiltInLayouts.kList).withPosition(4, 0).withSize(7, 10).withProperties(Map.of("label position", "LEFT"));

        _bedMinimumAngle = shooterBedSetting.add("Bed Minimum Angle", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
        _bedMaximumAngle = shooterBedSetting.add("Bed Maximum Angle", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();
        _ShooterOffset   = shooterBedSetting.add("Shooter Relative Offset", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();

        var shooterFlywheelSetting = outerLayout.getLayout("Shooter Flywheel", BuiltInLayouts.kList).withPosition(5, 0).withSize(7, 10).withProperties(Map.of("label position", "LEFT"));

        _maxFlywheelVelocity = shooterFlywheelSetting.add("Max Flywheel Velocity ", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();

        initializeSetting("FL Offset", Constants.Drive.MODULE_FL_OFFSET.getDegrees(), _flOffset, value ->
        {
            // empty lambda
        });

        initializeSetting("FR Offset", Constants.Drive.MODULE_FR_OFFSET.getDegrees(), _frOffset, value ->
        {
            // empty lambda
        });

        initializeSetting("BL Offset", Constants.Drive.MODULE_BL_OFFSET.getDegrees(), _blOffset, value ->
        {
            // empty lambda
        });

        initializeSetting("BR Offset", Constants.Drive.MODULE_BR_OFFSET.getDegrees(), _brOffset, value ->
        {
            // empty lambda
        });

        initializeSetting("Intake In Speed", Constants.Intake.INTAKE_DEFAULT_PERCENT_OUTPUT, _IntakeInSpeed, value ->
        {
            intake.setIntakePercentOutput(value);
        });

        initializeSetting("Intake Out Speed", Constants.Intake.OUTTAKE_DEFAULT_PERCENT_OUTPUT, _IntakeOutSpeed, value ->
        {
            intake.setOuttakePercentOutput(value);
        });

        initializeSetting("Climber Speed", 0, _climberMinHeight, value ->
        {
            // empty lambda
        });

        initializeSetting("Climber Left Offset", 0, _climberleftOffset, value ->
        {
            // empty lambda
        });

        initializeSetting("Climber Right Offset", 0, _climberRightOffset, value ->
        {
            // empty lambda
        });

        initializeSetting("Note Shoot Speed", Constants.Notepath.NOTEPATH_FEED_PERCENT_OUTPUT, _NoteShootSpeed, value ->
        {
            notepath.setNotepathFeedPercentOutput(value);
        });

        initializeSetting("Shooter Bed Offset", Constants.ShooterBed.BED_ANGLE_OFFSET.getDegrees(), _ShooterOffset, value ->
        {
            shooterBed.setAngleOffset(Rotation2d.fromDegrees(value));
        });

        initializeSetting("Max Flywheel Velocity", Constants.ShooterFlywheel.MAX_FLYWHEEL_SPEED, _maxFlywheelVelocity, value ->
        {
            ShooterFlywheel.setMaxFlywheelSpeed(value);
        });

        initializeSetting("Pickup Intake Speed", Constants.Intake.INTAKE_DEFAULT_PERCENT_OUTPUT, _PickupIntakeSpeed, value ->
        {
            notepath.setNotepathIntakePickupPercentOutput(value);
        });

        initializeSetting("Shooter Intake Speed", Constants.Intake.INTAKE_DEFAULT_PERCENT_OUTPUT, _ShooterIntakeSpeed, value ->
        {
            notepath.setNotepathShooterPickupPercentOutput(value);
        });

        initializeSetting("Bed Minimum Angle", 0, _bedMinimumAngle, value ->
        {
            shooterBed.setMinAngle(Rotation2d.fromDegrees(value));
        });

        initializeSetting("Bed Maximum Angle", 0, _bedMaximumAngle, value ->
        {
            shooterBed.setMaxAngle(Rotation2d.fromDegrees(value));
        });

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

    @Override
    public void periodic()
    {
        var isBlue = false;

        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue)
        {
            isBlue = true;
        }
        else
        {
            isBlue = false;
        }

        SwerveModuleState[] moduleStates = _drive.getModuleStates();

        _allianceBox.setBoolean(isBlue);
        _HasNote.setBoolean(false);
        _frAngle.setDouble(Double.parseDouble(String.format("%6.2f", moduleStates[0].angle.getDegrees())));
        _flAngle.setDouble(Double.parseDouble(String.format("%6.2f", moduleStates[1].angle.getDegrees())));
        _brAngle.setDouble(Double.parseDouble(String.format("%6.2f", moduleStates[2].angle.getDegrees())));
        _blAngle.setDouble(Double.parseDouble(String.format("%6.2f", moduleStates[3].angle.getDegrees())));
        _field.setRobotPose(_drive.getPose());

        _LeftHeight.setDouble(Double.parseDouble(String.format("%6.2f", 0.0)));
        _IntakeSpeed.setDouble(Double.parseDouble(String.format("%6.2f", _Intake.getPercentOutput())));
        _Rightheight.setDouble(Double.parseDouble(String.format("%6.2f", 0.0)));

        _notepathOutput.setDouble(Double.parseDouble(String.format("%6.2f", _notepath.getPercentOutput())));
        _bedAngle.setDouble(Double.parseDouble(String.format("%6.2f", _ShooterBed.getBedAngle().getDegrees())));

        _upperVelocity.setDouble(Double.parseDouble(String.format("%6.2f", _ShooterFlywheel.getUpperFlywheelVelocity())));
        _lowerVelocity.setDouble(Double.parseDouble(String.format("%6.2f", _ShooterFlywheel.getLowerFlywheelVelocity())));
    }
}
