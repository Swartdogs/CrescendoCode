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

public class ModuleIOHardware implements ModuleIO
{
    private final TalonFX              _driveMotor;
    private final CANSparkMax          _turnMotor;
    private final StatusSignal<Double> _drivePosition;
    private final StatusSignal<Double> _driveVelocity;
    private final StatusSignal<Double> _driveAppliedVolts;
    private final StatusSignal<Double> _driveCurrent;
    private final RelativeEncoder      _turnEncoder;
    private final AnalogEncoder        _turnPot;
    private Rotation2d                 _potOffset;

    public ModuleIOHardware(int driveCanId, int turnCanId, int potChannel, Rotation2d potOffset)
    {
        _driveMotor = new TalonFX(driveCanId);
        _turnMotor  = new CANSparkMax(turnCanId, MotorType.kBrushless);
        _turnPot    = new AnalogEncoder(potChannel);
        _potOffset  = potOffset;

        _turnMotor.restoreFactoryDefaults();

        _turnMotor.setCANTimeout(250);

        _turnEncoder = _turnMotor.getEncoder();

        _turnMotor.setInverted(true);
        _turnMotor.setSmartCurrentLimit(30);
        _turnMotor.enableVoltageCompensation(Constants.General.MOTOR_VOLTAGE);

        _turnEncoder.setPosition(0.0);
        _turnEncoder.setMeasurementPeriod(10);
        _turnEncoder.setAverageDepth(2);

        _turnMotor.setCANTimeout(0);

        _turnMotor.burnFlash();

        _drivePosition     = _driveMotor.getPosition();
        _driveVelocity     = _driveMotor.getVelocity();
        _driveAppliedVolts = _driveMotor.getMotorVoltage();
        _driveCurrent      = _driveMotor.getStatorCurrent();

        var driveCurrentConfig = new CurrentLimitsConfigs();
        driveCurrentConfig.StatorCurrentLimit       = 40.0;
        driveCurrentConfig.StatorCurrentLimitEnable = true;
        _driveMotor.getConfigurator().apply(driveCurrentConfig);
        setDriveBrakeMode(true);

        var driveOutputConfig = new MotorOutputConfigs();
        driveOutputConfig.Inverted = InvertedValue.Clockwise_Positive;
        _driveMotor.getConfigurator().apply(driveOutputConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(100.0, _drivePosition); // Required for odometry, use faster rate
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, _driveVelocity, _driveAppliedVolts, _driveCurrent);

        _driveMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs)
    {
        BaseStatusSignal.refreshAll(_drivePosition, _driveVelocity, _driveAppliedVolts, _driveCurrent);

        inputs.drivePositionRad       = Units.rotationsToRadians(_drivePosition.getValueAsDouble()) / Constants.Drive.DRIVE_GEAR_RATIO;
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(_driveVelocity.getValueAsDouble()) / Constants.Drive.DRIVE_GEAR_RATIO;
        inputs.driveVolts             = _driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrent           = new double[] { _driveCurrent.getValueAsDouble() };

        inputs.turnAbsolutePosition  = Rotation2d.fromRotations(_turnPot.getAbsolutePosition()).minus(_potOffset);
        inputs.turnPosition          = Rotation2d.fromRotations(_turnEncoder.getPosition() / Constants.Drive.TURN_GEAR_RATIO);
        inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(_turnEncoder.getVelocity()) / Constants.Drive.TURN_GEAR_RATIO;
        inputs.turnVolts             = _turnMotor.getAppliedOutput() * _turnMotor.getBusVoltage();
        inputs.turnCurrent           = new double[] { _turnMotor.getOutputCurrent() };
    }

    @Override
    public void setDriveVolts(double volts)
    {
        _driveMotor.setVoltage(volts);
    }

    @Override
    public void setTurnVolts(double volts)
    {
        _turnMotor.setVoltage(volts);
    }

    @Override
    public void setDriveBrakeMode(boolean enable)
    {
        var config = new MotorOutputConfigs();
        config.Inverted    = InvertedValue.CounterClockwise_Positive;
        config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        _driveMotor.getConfigurator().apply(config);
    }

    @Override
    public void setTurnBrakeMode(boolean enable)
    {
        _turnMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setAngleOffset(Rotation2d offset)
    {
        _potOffset = offset;
    }
}
