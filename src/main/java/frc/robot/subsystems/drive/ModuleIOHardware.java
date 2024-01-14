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

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOHardware implements ModuleIO {
  // Gear ratios for SDS MK4i L2, adjust as necessary
  private static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  private static final double TURN_GEAR_RATIO = 150.0 / 7.0;

  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  private final TalonFX driveTalon;

  private final CANSparkMax turnSparkMax;

  private final RelativeEncoder turnRelativeEncoder;
  private final AnalogInput turnAbsoluteEncoder;

  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOHardware(int index) {
    switch (index) {
      case 0:
        driveTalon = new TalonFX(1);
        turnSparkMax = new CANSparkMax(2, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(0);
        absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      case 1:
        driveTalon = new TalonFX(3);
        turnSparkMax = new CANSparkMax(4, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(1);
        absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      case 2:
        driveTalon = new TalonFX(5);
        turnSparkMax = new CANSparkMax(6, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(2);
        absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      case 3:
        driveTalon = new TalonFX(7);
        turnSparkMax = new CANSparkMax(8, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(3);
        absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(true);

    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, driveVelocity, driveAppliedVolts, driveCurrent);

    driveTalon.optimizeBusUtilization();

    turnSparkMax.restoreFactoryDefaults();

    turnSparkMax.setCANTimeout(250);

    turnRelativeEncoder = turnSparkMax.getEncoder();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);

    inputs.drivePositionRad =
        Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

    inputs.turnAbsolutePosition =
        new Rotation2d(
                turnAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V() * 2.0 * Math.PI)
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
