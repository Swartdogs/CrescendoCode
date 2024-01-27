package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants;
import java.util.EnumSet;
import java.util.Map;
import java.util.function.DoubleConsumer;

public class Dashboard {
  private static Dashboard _instance;

  // Widgets for the dashboard
  private GenericEntry _cameraStream;
  private GenericEntry _field;
  private GenericEntry _allianceBox;
  private GenericEntry _TargetBox;
  private GenericEntry _intakeBox;
  private GenericEntry _frAngle;
  private GenericEntry _flAngle;
  private GenericEntry _brAngle;
  private GenericEntry _blAngle;
  private GenericEntry _Leftheight;
  private GenericEntry _Rightheight;
  private GenericEntry _speedDial;
  private GenericEntry _IntakeSpeed;
  private GenericEntry _autonomousSplitButton;
  private GenericEntry _autonomousComboBox;

  private GenericEntry _frOffset;
  private GenericEntry _flOffset;
  private GenericEntry _brOffset;
  private GenericEntry _blOffset;
  private GenericEntry _IntakeInSpeed;
  private GenericEntry _IntakeOutSpeed;
  private GenericEntry _HangerleftOffset;
  private GenericEntry _HangerrightOffset;
  private GenericEntry _HangerSpeed;
  private GenericEntry _NoteShootSpeed;
  private GenericEntry _NoteloadTimeOut;
  private GenericEntry _ShooterOffset;

  // SendableChoosers for Autonomous options
  private SendableChooser<Integer> _autoDelayChooser;

  public Dashboard() {
    var tab = Shuffleboard.getTab("Dashboard");
    var SettingsTab = Shuffleboard.getTab("Settings");

    // Camera Stream
    _cameraStream =
        tab.add("Camera Stream", false) // get the intanse of the camera stream
            .withPosition(0, 0)
            .withSize(12, 13)
            .withProperties(Map.of())
            // .withWidget(BuiltInWidgets.kCameraStream)
            .getEntry();

    // Field
    _field =
        tab.add("field", false)
            .withPosition(12, 0)
            .withSize(14, 8)
            .withProperties(Map.of())
            .getEntry();

    // Alliance, Target, and Intake boxes
    var booleanBoxLayout =
        tab.getLayout("Color Box", BuiltInLayouts.kGrid)
            .withPosition(26, 0)
            .withSize(4, 4)
            .withProperties(
                Map.of("Number of columns", 1, "Number of rows", 3, "Label position", "LEFT"));
    _allianceBox =
        booleanBoxLayout
            .add("Alliance", false)
            .withPosition(0, 0)
            .withSize(8, 1)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("Color when true", "blue", "Color when false", "red"))
            .getEntry();
    _TargetBox =
        booleanBoxLayout
            .add("Has Targets", false)
            .withPosition(0, 1)
            .withSize(8, 1)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("Color when true", "green", "Color when false", "red"))
            .getEntry();

    // Swerve Module Angles
    var driveSettingsLayout =
        tab.getLayout("Drive Subsystem", BuiltInLayouts.kGrid).withPosition(26, 4).withSize(4, 3);
    _flAngle =
        driveSettingsLayout
            .add("FL Offset", 0.0)
            .withPosition(0, 0)
            .withSize(2, 2)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
    _frAngle =
        driveSettingsLayout
            .add("FR Offset", 0.0)
            .withPosition(2, 0)
            .withSize(2, 2)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
    _blAngle =
        driveSettingsLayout
            .add("BL Offset", 0.0)
            .withPosition(0, 2)
            .withSize(2, 2)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
    _brAngle =
        driveSettingsLayout
            .add("BR Offset", 0.0)
            .withPosition(2, 2)
            .withSize(2, 2)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();

    // Height, Intake and Speed
    var heightAndIntakeAndSpeedLayout =
        tab.getLayout("Height, Speed, Intake", BuiltInLayouts.kGrid)
            .withPosition(12, 8)
            .withSize(18, 5)
            .withProperties(Map.of("Number of columns", 4, "Number of rows", 1));
    _Leftheight =
        heightAndIntakeAndSpeedLayout
            .add("Left Height", 0)
            .withSize(5, 4)
            .withPosition(0, 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("Orientation", "VERTICAL"))
            .getEntry();
    _speedDial =
        heightAndIntakeAndSpeedLayout
            .add("Speed", 0)
            .withWidget(BuiltInWidgets.kDial)
            .withPosition(1, 0)
            .getEntry();

    _Rightheight =
        heightAndIntakeAndSpeedLayout
            .add("Right Height", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withPosition(2, 0)
            .withProperties(Map.of("Orientation", "VERTICAL"))
            .getEntry();
    _IntakeSpeed =
        heightAndIntakeAndSpeedLayout
            .add("Intake Speed", 0)
            .withPosition(3, 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();
    // Autonomous Options
    var autonomousLayout =
        tab.getLayout("Autonomous", BuiltInLayouts.kGrid)
            .withPosition(30, 0)
            .withSize(8, 13)
            .withProperties(
                Map.of("Number of columns", 1, "Number of rows", 4, "Label position", "LEFT"));

    // Autonomous delay chooser setup
    _autoDelayChooser = new SendableChooser<>();
    _autoDelayChooser.setDefaultOption("0", 0);
    _autoDelayChooser.addOption("1", 1);
    _autoDelayChooser.addOption("2", 2);
    _autoDelayChooser.addOption("3", 3);
    _autoDelayChooser.addOption("4", 4);
    _autoDelayChooser.addOption("5", 5);
    autonomousLayout
        .add("Delay Options", _autoDelayChooser)
        .withPosition(0, 0)
        .withSize(1, 1)
        .withWidget(BuiltInWidgets.kSplitButtonChooser);

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

    /* initializeOffsetSetting("FL Offset", Constants.Drive.FL_OFFSET, _flAngle,
            Drive.getInstance().getSwerveModule(Constants.Drive.FL_INDEX)::setRotationZero);
    initializeOffsetSetting("FR Offset", Constants.Drive.FR_OFFSET, _frAngle,
            Drive.getInstance().getSwerveModule(Constants.Drive.FR_INDEX)::setRotationZero);
    initializeOffsetSetting("BL Offset", Constants.Drive.BL_OFFSET, _blAngle,
            Drive.getInstance().getSwerveModule(Constants.Drive.BL_INDEX)::setRotationZero);
    initializeOffsetSetting("BR Offset", Constants.Drive.BR_OFFSET, _brAngle,
            Drive.getInstance().getSwerveModule(Constants.Drive.BR_INDEX)::setRotationZero);
     */

    var outerLayout =
        SettingsTab.getLayout("Settings", BuiltInLayouts.kGrid)
            .withPosition(0, 0)
            .withSize(37, 10)
            .withProperties(
                Map.of("Number of columns", 5, "Number of rows", 1, "Label position", "TOP"));

    var driveSettings =
        outerLayout
            .getLayout("Drive Subsystem", BuiltInLayouts.kList)
            .withPosition(0, 0)
            .withSize(8, 10)
            .withProperties(Map.of("label position", "LEFT"));

    _flOffset =
        driveSettings
            .add("FL Offset", 0.0)
            .withPosition(0, 0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();

    _frOffset =
        driveSettings
            .add("FR Offset", 0.0)
            .withPosition(2, 0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();

    _blOffset =
        driveSettings
            .add("BL Offset", 0.0)
            .withPosition(0, 2)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();

    _brOffset =
        driveSettings
            .add("BR Offset", 0.0)
            .withPosition(2, 2)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();

    var IntakeSetting =
        outerLayout
            .getLayout("Intake", BuiltInLayouts.kList)
            .withPosition(1, 0)
            .withSize(7, 10)
            .withProperties(Map.of("label position", "LEFT"));

    _IntakeInSpeed =
        IntakeSetting.add("Intake In Speed", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 0)
            .getEntry();

    _IntakeOutSpeed =
        IntakeSetting.add("Intake Out Speed", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 1)
            .getEntry();

    var hangerSetting =
        outerLayout
            .getLayout("Hanger", BuiltInLayouts.kList)
            .withPosition(2, 0)
            .withProperties(Map.of("label position", "LEFT"))
            .withSize(7, 10);

    _HangerleftOffset =
        hangerSetting.add("Left Offset", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();

    _HangerrightOffset =
        hangerSetting.add("Right Offset", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();

    _HangerSpeed =
        hangerSetting.add("Hanger Speed", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();

    var noteSetting =
        outerLayout
            .getLayout("Note", BuiltInLayouts.kList)
            .withPosition(3, 0)
            .withSize(8, 10)
            .withProperties(Map.of("label position", "LEFT"));

    _NoteShootSpeed =
        noteSetting.add("Note Shoot Speed", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();

    _NoteloadTimeOut =
        noteSetting.add("Note Load Timeout", 0.0).withWidget(BuiltInWidgets.kTextView).getEntry();

    var shooterSetting =
        outerLayout
            .getLayout("Shooter", BuiltInLayouts.kList)
            .withPosition(4, 0)
            .withSize(7, 10)
            .withProperties(Map.of("label position", "LEFT"));

    _ShooterOffset =
        shooterSetting
            .add("Shooter Relative Offset", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();

    initializeSetting(
        "FL Offset",
        Constants.Drive.FL_OFFSET,
        _flOffset,
        value -> {
          // empty lambda
        });

    initializeSetting(
        "FR Offset",
        Constants.Drive.FR_OFFSET,
        _frOffset,
        value -> {
          // empty lambda
        });

    initializeSetting(
        "BL Offset",
        Constants.Drive.BL_OFFSET,
        _blOffset,
        value -> {
          // empty lambda
        });

    initializeSetting(
        "BR Offset",
        Constants.Drive.BR_OFFSET,
        _brOffset,
        value -> {
          // empty lambda
        });

    initializeSetting(
        "Intake In Speed",
        Constants.Drive.Intake_IN_SPEED,
        _IntakeInSpeed,
        value -> {
          // empty lambda
        });

    initializeSetting(
        "Intake Out Speed",
        Constants.Drive.Intake_OUT_SPEED,
        _IntakeInSpeed,
        value -> {
          // empty lambda
        });

    initializeSetting(
        "Hanger Speed",
        Constants.Drive.HANGER_SPEED,
        _HangerSpeed,
        value -> {
          // empty lambda
        });

    initializeSetting(
        "Hanger Left Offset",
        Constants.Drive.HANGER_LEFT_OFFSET,
        _HangerleftOffset,
        value -> {
          // empty lambda
        });

    initializeSetting(
        "Hanger Right Offset",
        Constants.Drive.HANGER_RIGHT_OFFSET,
        _HangerrightOffset,
        value -> {
          // empty lambda
        });

    initializeSetting(
        "Note Shoot Speed",
        Constants.Drive.NOTE_SHOOT_SPEED,
        _NoteShootSpeed,
        value -> {
          // empty lambda
        });

    initializeSetting(
        "Note Load Timeout",
        Constants.Drive.NOTE_LOAD_TIMEOUT,
        _NoteloadTimeOut,
        value -> {
          // empty lambda
        });

    initializeSetting(
        "Shooter Offset",
        Constants.Drive.SHOOTER_OFFSET,
        _ShooterOffset,
        value -> {
          // empty lambda
        });
  }

  public void initializeSetting(
      String key, double defaultValue, GenericEntry entry, DoubleConsumer consumer) {
    NetworkTableInstance.getDefault()
        .addListener(
            entry,
            EnumSet.of(NetworkTableEvent.Kind.kValueRemote),
            event -> {
              consumer.accept(entry.getDouble(defaultValue));
              Preferences.setDouble(key, entry.getDouble(defaultValue));
            });

    double value = defaultValue;

    if (Preferences.containsKey(key)) {
      value = Preferences.getDouble(key, defaultValue);
    } else {
      Preferences.initDouble(key, defaultValue);
    }

    consumer.accept(value);
    entry.setDouble(value);
  }

  public void periodic() {
    _allianceBox.setBoolean(false);
    _TargetBox.setBoolean(false);
    _intakeBox.setBoolean(false);
    _frAngle.setDouble(Double.parseDouble(String.format("%6.2f", 0.0)));
    _flAngle.setDouble(Double.parseDouble(String.format("%6.2f", 0.0)));
    _brAngle.setDouble(Double.parseDouble(String.format("%6.2f", 0.0)));
    _blAngle.setDouble(Double.parseDouble(String.format("%6.2f", 0.0)));

    _Leftheight.setDouble(Double.parseDouble(String.format("%6.2f", 0.0)));
    _Rightheight.setDouble(Double.parseDouble(String.format("%6.2f", 0.0)));
    _speedDial.setDouble(Double.parseDouble(String.format("%6.2f", 0.0)));
    _IntakeSpeed.setDouble(Double.parseDouble(String.format("%6.2f", 0.0)));
  }
}
