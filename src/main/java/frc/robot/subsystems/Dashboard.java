package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
  private GenericEntry _heightNumberBar;
  private GenericEntry _speedDial;
  private GenericEntry _autonomousSplitButton;
  private GenericEntry _autonomousComboBox;

  // SendableChoosers for Autonomous options
  private SendableChooser<Integer> _autoDelayChooser;

  public Dashboard() {
    var tab = Shuffleboard.getTab("Dashboard");

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

    _intakeBox =
        booleanBoxLayout
            .add("Intake", false)
            .withPosition(0, 2)
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

    // Height and Speed
    var heightAndSpeedLayout =
        tab.getLayout("Height and Speed", BuiltInLayouts.kGrid)
            .withPosition(12, 8)
            .withSize(9, 5)
            .withProperties(Map.of("Number of columns", 2, "Number of rows", 1));
    _heightNumberBar =
        heightAndSpeedLayout.add("Height", 0).withWidget(BuiltInWidgets.kNumberBar).getEntry();
    _speedDial =
        heightAndSpeedLayout
            .add("Speed", 0)
            .withWidget(BuiltInWidgets.kDial)
            .getEntry();
    // Autonomous Options
    var autonomousLayout =
        tab.getLayout("Autonomous", BuiltInLayouts.kGrid)
            .withPosition(21, 8)
            .withSize(9, 5)
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
  }

  // Method to initialize settings with default values and listen for changes
  public void initializeOffsetSetting(
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

  // Method to get the selected auto delay
  public int getAutoDelay() {
    return _autoDelayChooser.getSelected();
  }

  // Method to start the dashboard updates
  public void startDashboard() {
    Thread thread =
        new Thread(
            () -> {
              while (true) periodic();
            });
    thread.start();
  }

  // Periodic method to update widgets
  public void periodic() {
    // // Update the height and speed widgets
    // _heightNumberBar.setDouble(/* Get the current height */);
    // _speedDial.setDouble(/* Get the current speed */);

    // Update Alliance color
    // _allianceBox.setBoolean(DriverStation.getAlliance() == Alliance.Blue);

    // // Update Swerve Module angles
    // _frAngle.setDouble(Drive.getInstance().getModule(Constants.FR_MODULE).getAngle());
    // _flAngle.setDouble(Drive.getInstance().getModule(Constants.FL_MODULE).getAngle());
    // _brAngle.setDouble(Drive.getInstance().getModule(Constants.BR_MODULE).getAngle());
    // _blAngle.setDouble(Drive.getInstance().getModule(Constants.BL_MODULE).getAngle());

    // // Update Intake state
    // _intakeBox.setBoolean(Intake.getInstance().hasCube());
  }
}
