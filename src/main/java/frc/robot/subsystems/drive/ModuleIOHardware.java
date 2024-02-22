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

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn
 * motor controller (NEO or NEO 550), and analog absolute encoder connected to
 * the RIO
 * <p>
 * NOTE: This implementation should be used as a starting point and adapted to
 * different hardware configurations (e.g. If using a CANcoder, copy from
 * "ModuleIOTalonFX")
 * <p>
 * To calibrate the absolute encoder offsets, point the modules straight (such
 * that forward motion on the drive motor will propel the robot forward) and
 * copy the reported values from the absolute encoders using AdvantageScope.
 * These values are logged under "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOHardware implements ModuleIO
{
    private final TalonFX              _driveTalonFX;
    private final CANSparkMax          _turnSparkMax;
    private final RelativeEncoder      _turnRelativeEncoder;
    private final AnalogEncoder        _turnAbsoluteEncoder;
    private final boolean              _isTurnMotorInverted = true;
    private final Rotation2d           _absoluteEncoderOffset;
    private final StatusSignal<Double> _drivePosition;
    private final StatusSignal<Double> _driveVelocity;
    private final StatusSignal<Double> _driveAppliedVolts;
    private final StatusSignal<Double> _driveCurrent;

    public ModuleIOHardware(int driveCanId, int turnCanId, int absoluteEncoderChannel, Rotation2d absoluteEncoderOffset)
    {
        _driveTalonFX          = new TalonFX(driveCanId);
        _turnSparkMax          = new CANSparkMax(turnCanId, MotorType.kBrushless);
        _turnAbsoluteEncoder   = new AnalogEncoder(absoluteEncoderChannel);
        _absoluteEncoderOffset = absoluteEncoderOffset;

        _turnSparkMax.restoreFactoryDefaults();
        _turnSparkMax.setCANTimeout(250);

        _turnRelativeEncoder = _turnSparkMax.getEncoder();

        _turnSparkMax.setInverted(_isTurnMotorInverted);
        _turnSparkMax.setSmartCurrentLimit(30);
        _turnSparkMax.enableVoltageCompensation(12.0);

        _turnRelativeEncoder.setPosition(0.0);
        _turnRelativeEncoder.setMeasurementPeriod(10);
        _turnRelativeEncoder.setAverageDepth(2);

        _turnSparkMax.setCANTimeout(0);

        _turnSparkMax.burnFlash();

        _drivePosition     = _driveTalonFX.getPosition();
        _driveVelocity     = _driveTalonFX.getVelocity();
        _driveAppliedVolts = _driveTalonFX.getMotorVoltage();
        _driveCurrent      = _driveTalonFX.getStatorCurrent();

        var driveCurrentConfig = new CurrentLimitsConfigs();
        driveCurrentConfig.StatorCurrentLimit       = 40.0;
        driveCurrentConfig.StatorCurrentLimitEnable = true;
        _driveTalonFX.getConfigurator().apply(driveCurrentConfig);
        setDriveBrakeMode(true);

        var driveOutputConfig = new MotorOutputConfigs();
        driveOutputConfig.Inverted = InvertedValue.Clockwise_Positive;
        _driveTalonFX.getConfigurator().apply(driveOutputConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(100.0, _drivePosition); // Required for odometry, use faster rate
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, _driveVelocity, _driveAppliedVolts, _driveCurrent);

        _driveTalonFX.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs)
    {
        BaseStatusSignal.refreshAll(_drivePosition, _driveVelocity, _driveAppliedVolts, _driveCurrent);

        inputs.drivePositionRad       = Units.rotationsToRadians(_drivePosition.getValueAsDouble()) / Constants.Drive.DRIVE_GEAR_RATIO;
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(_driveVelocity.getValueAsDouble()) / Constants.Drive.DRIVE_GEAR_RATIO;
        inputs.driveAppliedVolts      = _driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps       = new double[] { _driveCurrent.getValueAsDouble() };

        inputs.turnAbsolutePosition  = Rotation2d.fromRotations(_turnAbsoluteEncoder.getAbsolutePosition()).minus(_absoluteEncoderOffset);
        inputs.turnPosition          = Rotation2d.fromRotations(_turnRelativeEncoder.getPosition() / Constants.Drive.TURN_GEAR_RATIO);
        inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(_turnRelativeEncoder.getVelocity()) / Constants.Drive.TURN_GEAR_RATIO;
        inputs.turnAppliedVolts      = _turnSparkMax.getAppliedOutput() * _turnSparkMax.getBusVoltage();
        inputs.turnCurrentAmps       = new double[] { _turnSparkMax.getOutputCurrent() };
    }

    @Override
    public void setDriveVoltage(double volts)
    {
        _driveTalonFX.setVoltage(volts);
    }

    @Override
    public void setTurnVoltage(double volts)
    {
        _turnSparkMax.setVoltage(volts);
    }

    @Override
    public void setDriveBrakeMode(boolean enable)
    {
        var config = new MotorOutputConfigs();
        config.Inverted    = InvertedValue.CounterClockwise_Positive;
        config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        _driveTalonFX.getConfigurator().apply(config);
    }

    @Override
    public void setTurnBrakeMode(boolean enable)
    {
        _turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }
}
