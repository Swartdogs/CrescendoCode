package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;

import frc.robot.Constants;

public class ModuleIOSparkMax implements ModuleIO
{
    private final CANSparkMax     _driveMotor;
    private final CANSparkMax     _turnMotor;
    private final RelativeEncoder _driveEncoder;
    private final RelativeEncoder _turnEncoder;
    private final AnalogEncoder   _turnPot;
    private Rotation2d            _potOffset;

    public ModuleIOSparkMax(int driveCanId, int turnCanId, int potChannel, Rotation2d potOffset)
    {
        _driveMotor = new CANSparkMax(driveCanId, MotorType.kBrushless);
        _turnMotor  = new CANSparkMax(turnCanId, MotorType.kBrushless);
        _turnPot    = new AnalogEncoder(potChannel);
        _potOffset  = potOffset;

        _driveMotor.restoreFactoryDefaults();
        _turnMotor.restoreFactoryDefaults();

        _driveMotor.setCANTimeout(250);
        _turnMotor.setCANTimeout(250);

        _driveEncoder = _driveMotor.getEncoder();
        _turnEncoder  = _turnMotor.getEncoder();

        _turnMotor.setInverted(true);
        _driveMotor.setInverted(true);
        _driveMotor.setSmartCurrentLimit(40);
        _turnMotor.setSmartCurrentLimit(30);
        _driveMotor.enableVoltageCompensation(Constants.General.MOTOR_VOLTAGE);
        _turnMotor.enableVoltageCompensation(Constants.General.MOTOR_VOLTAGE);

        _driveEncoder.setPosition(0.0);
        _driveEncoder.setMeasurementPeriod(10);
        _driveEncoder.setAverageDepth(2);

        _turnEncoder.setPosition(0.0);
        _turnEncoder.setMeasurementPeriod(10);
        _turnEncoder.setAverageDepth(2);

        _driveMotor.setCANTimeout(0);
        _turnMotor.setCANTimeout(0);

        _driveMotor.burnFlash();
        _turnMotor.burnFlash();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs)
    {
        inputs.drivePositionRad       = Units.rotationsToRadians(_driveEncoder.getPosition()) / Constants.Drive.DRIVE_GEAR_RATIO;
        inputs.driveVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(_driveEncoder.getVelocity()) / Constants.Drive.DRIVE_GEAR_RATIO;
        inputs.driveVolts             = _driveMotor.getAppliedOutput() * _driveMotor.getBusVoltage();
        inputs.driveCurrent           = new double[] { _driveMotor.getOutputCurrent() };

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
        _driveMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
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
