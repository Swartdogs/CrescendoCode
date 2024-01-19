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
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX _driveTalon;
  private final TalonFX _turnTalon;
  private final CANcoder _cancoder;

  private final StatusSignal<Double> _drivePosition;
  private final StatusSignal<Double> _driveVelocity;
  private final StatusSignal<Double> _driveAppliedVolts;
  private final StatusSignal<Double> _driveCurrent;

  private final StatusSignal<Double> _turnAbsolutePosition;
  private final StatusSignal<Double> _turnPosition;
  private final StatusSignal<Double> _turnVelocity;
  private final StatusSignal<Double> _turnAppliedVolts;
  private final StatusSignal<Double> _turnCurrent;

  private final boolean _isTurnMotorInverted = true;
  private final Rotation2d _absoluteEncoderOffset;

  public ModuleIOTalonFX(
      int driveCanId, int turnCanId, int absoluteEncoderChannel, double absoluteEncoderOffset) {
    _driveTalon = new TalonFX(driveCanId);
    _turnTalon = new TalonFX(turnCanId);
    _cancoder = new CANcoder(absoluteEncoderChannel);
    _absoluteEncoderOffset = Rotation2d.fromDegrees(absoluteEncoderOffset);

    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    _driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(true);

    var turnConfig = new TalonFXConfiguration();
    turnConfig.CurrentLimits.StatorCurrentLimit = 30.0;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    _turnTalon.getConfigurator().apply(turnConfig);
    setTurnBrakeMode(true);

    _cancoder.getConfigurator().apply(new CANcoderConfiguration());

    _drivePosition = _driveTalon.getPosition();
    _driveVelocity = _driveTalon.getVelocity();
    _driveAppliedVolts = _driveTalon.getMotorVoltage();
    _driveCurrent = _driveTalon.getStatorCurrent();

    _turnAbsolutePosition = _cancoder.getAbsolutePosition();
    _turnPosition = _turnTalon.getPosition();
    _turnVelocity = _turnTalon.getVelocity();
    _turnAppliedVolts = _turnTalon.getMotorVoltage();
    _turnCurrent = _turnTalon.getStatorCurrent();

    // Required for odometry, use faster rate
    BaseStatusSignal.setUpdateFrequencyForAll(100.0, _drivePosition, _turnPosition);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        _driveVelocity,
        _driveAppliedVolts,
        _driveCurrent,
        _turnAbsolutePosition,
        _turnVelocity,
        _turnAppliedVolts,
        _turnCurrent);

    _driveTalon.optimizeBusUtilization();
    _turnTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        _drivePosition,
        _driveVelocity,
        _driveAppliedVolts,
        _driveCurrent,
        _turnAbsolutePosition,
        _turnPosition,
        _turnVelocity,
        _turnAppliedVolts,
        _turnCurrent);

    inputs.drivePositionRad =
        Units.rotationsToRadians(_drivePosition.getValueAsDouble())
            / Constants.Drive.DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(_driveVelocity.getValueAsDouble())
            / Constants.Drive.DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = _driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {_driveCurrent.getValueAsDouble()};

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(_turnAbsolutePosition.getValueAsDouble())
            .minus(_absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(
            _turnPosition.getValueAsDouble() / Constants.Drive.TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsToRadians(_turnVelocity.getValueAsDouble())
            / Constants.Drive.TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = _turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = new double[] {_turnCurrent.getValueAsDouble()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    _driveTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    _turnTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    _driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted =
        _isTurnMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    _turnTalon.getConfigurator().apply(config);
  }
}
