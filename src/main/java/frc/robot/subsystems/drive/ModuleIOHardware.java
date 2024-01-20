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
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import frc.robot.Constants;

public class ModuleIOHardware implements ModuleIO
{
    private final StatusSignal<Double> _drivePosition;
    private final StatusSignal<Double> _driveVelocity;
    private final StatusSignal<Double> _driveAppliedVolts;
    private final StatusSignal<Double> _driveCurrent;

    private final TalonFX _driveTalon;

    private final CANSparkMax _turnSparkMax;

    private final RelativeEncoder _turnRelativeEncoder;
    private final AnalogPotentiometer _turnAbsoluteEncoder;

    private final Rotation2d _absoluteEncoderOffset;

    public ModuleIOHardware(int driveCanId, int turnCanId, int absoluteEncoderChannel, double absoluteEncoderOffset)
    {
        _driveTalon = new TalonFX(driveCanId);
        _turnSparkMax = new CANSparkMax(turnCanId, MotorType.kBrushless);
        _turnAbsoluteEncoder = new AnalogPotentiometer(absoluteEncoderChannel, Constants.Drive.HALL_EFFECT_SCALE,
                Constants.Drive.HALL_EFFECT_OFFSET);
        _absoluteEncoderOffset = Rotation2d.fromDegrees(absoluteEncoderOffset);

        var driveConfig = new TalonFXConfiguration();
        driveConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        _driveTalon.getConfigurator().apply(driveConfig);
        setDriveBrakeMode(true);

        _drivePosition = _driveTalon.getPosition();
        _driveVelocity = _driveTalon.getVelocity();
        _driveAppliedVolts = _driveTalon.getMotorVoltage();
        _driveCurrent = _driveTalon.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, _driveVelocity, _driveAppliedVolts, _driveCurrent);
        _driveTalon.optimizeBusUtilization();

        _turnSparkMax.restoreFactoryDefaults();
        _turnSparkMax.setCANTimeout(250);
        _turnRelativeEncoder = _turnSparkMax.getEncoder();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs)
    {
        BaseStatusSignal.refreshAll(_drivePosition, _driveVelocity, _driveAppliedVolts, _driveCurrent);

        inputs.drivePositionRad = Units.rotationsToRadians(_drivePosition.getValueAsDouble())
                / Constants.Drive.DRIVE_GEAR_RATIO;
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(_driveVelocity.getValueAsDouble())
                / Constants.Drive.DRIVE_GEAR_RATIO;
        inputs.driveAppliedVolts = _driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = new double[]
        { _driveCurrent.getValueAsDouble() };

        inputs.turnAbsolutePosition = Rotation2d
                .fromDegrees(Math.IEEEremainder(_turnAbsoluteEncoder.get() - _absoluteEncoderOffset.getDegrees(), 360));
        inputs.turnPosition = Rotation2d
                .fromRotations(_turnRelativeEncoder.getPosition() / Constants.Drive.TURN_GEAR_RATIO);
        inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(_turnRelativeEncoder.getVelocity())
                / Constants.Drive.TURN_GEAR_RATIO;
        inputs.turnAppliedVolts = _turnSparkMax.getAppliedOutput() * _turnSparkMax.getBusVoltage();
        inputs.turnCurrentAmps = new double[]
        { _turnSparkMax.getOutputCurrent() };
        inputs.turnPositionDegrees = inputs.turnPosition.getDegrees();
        inputs.turnAbsolutePositionDegrees = inputs.turnAbsolutePosition.getDegrees();
    }

    @Override
    public void setDriveVoltage(double volts)
    {
        _driveTalon.setControl(new VoltageOut(volts));
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
        config.Inverted = InvertedValue.CounterClockwise_Positive;
        config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        _driveTalon.getConfigurator().apply(config);
    }

    @Override
    public void setTurnBrakeMode(boolean enable)
    {
        _turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }
}
