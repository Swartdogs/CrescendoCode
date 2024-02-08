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
public class ModuleTalonFX implements ModuleIO
{
    private final TalonFX         _driveSparkMax;
    private final TalonFX         _turnSparkMax;
    private final RelativeEncoder _driveEncoder;
    private final RelativeEncoder _turnRelativeEncoder;
    private final AnalogEncoder   _turnAbsoluteEncoder;
    private final boolean         _isTurnMotorInverted = true;
    private final Rotation2d      _absoluteEncoderOffset;

    public ModuleTalonFX(int driveCanId, int turnCanId, int absoluteEncoderChannel, Rotation2d absoluteEncoderOffset)
    {
        _driveSparkMax         = new TalonFX(driveCanId);
        _turnSparkMax          = new TalonFX(turnCanId);
        _turnAbsoluteEncoder   = new AnalogEncoder(absoluteEncoderChannel);
        _absoluteEncoderOffset = absoluteEncoderOffset;

        _driveSparkMax.restoreFactoryDefaults();
        _turnSparkMax.restoreFactoryDefaults();

        _driveSparkMax.setCANTimeout(250);
        _turnSparkMax.setCANTimeout(250);

        _driveEncoder        = _driveSparkMax.getEncoder();
        _turnRelativeEncoder = _turnSparkMax.getEncoder();

        _turnSparkMax.setInverted(_isTurnMotorInverted);
        _driveSparkMax.setInverted(true);
        _driveSparkMax.setSmartCurrentLimit(40);
        _turnSparkMax.setSmartCurrentLimit(30);
        _driveSparkMax.enableVoltageCompensation(12.0);
        _turnSparkMax.enableVoltageCompensation(12.0);

        _driveEncoder.setPosition(0.0);
        _driveEncoder.setMeasurementPeriod(10);
        _driveEncoder.setAverageDepth(2);

        _turnRelativeEncoder.setPosition(0.0);
        _turnRelativeEncoder.setMeasurementPeriod(10);
        _turnRelativeEncoder.setAverageDepth(2);

        _driveSparkMax.setCANTimeout(0);
        _turnSparkMax.setCANTimeout(0);

        _driveSparkMax.burnFlash();
        _turnSparkMax.burnFlash();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs)
    {
        inputs.drivePositionRad       = Units.rotationsToRadians(_driveEncoder.getPosition()) / Constants.Drive.DRIVE_GEAR_RATIO;
        inputs.driveVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(_driveEncoder.getVelocity()) / Constants.Drive.DRIVE_GEAR_RATIO;
        inputs.driveAppliedVolts      = _driveSparkMax.getAppliedOutput() * _driveSparkMax.getMotorVoltage();
        inputs.driveCurrentAmps       = new double[] { _driveSparkMax.getValueAsDouble };

        inputs.turnAbsolutePosition  = Rotation2d.fromRotations(_turnAbsoluteEncoder.getAbsolutePosition()).minus(_absoluteEncoderOffset);
        inputs.turnPosition          = Rotation2d.fromRotations(_turnRelativeEncoder.getPosition() / Constants.Drive.TURN_GEAR_RATIO);
        inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(_turnRelativeEncoder.getVelocity()) / Constants.Drive.TURN_GEAR_RATIO;
        inputs.turnAppliedVolts      = _turnSparkMax.getAppliedOutput() * _turnSparkMax.getMotorVoltage();
        inputs.turnCurrentAmps       = new double[] { _turnSparkMax.getOutputCurrent() };
    }

    @Override
    public void setDriveVoltage(double volts)
    {
        _driveSparkMax.setVoltage(volts);
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
        _driveSparkMax.getConfigurator().apply(config);
    }

    @Override
    public void setTurnBrakeMode(boolean enable) {
      var config = new MotorOutputConfigs();
      config.Inverted =
          isTurnMotorInverted
              ? InvertedValue.Clockwise_Positive
              : InvertedValue.CounterClockwise_Positive;
      config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
      _turnSparkMax.getConfigurator().apply(config);
    }

    @Override
    public void setTurnBrakeMode(boolean enable)
    {
        _turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }
}
