package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import frc.robot.Constants;

public class ShooterBedIOVictorSPX implements ShooterBedIO
{
    private final VictorSPX        _leaderMotor;
    private final VictorSPX        _followerMotor;
    private final DutyCycleEncoder _absoluteEncoder;
    private Rotation2d             _absoluteEncoderOffset;

    public ShooterBedIOVictorSPX()
    {
        _leaderMotor           = new VictorSPX(Constants.CAN.SHOOTER_BED_LEADER);
        _followerMotor         = new VictorSPX(Constants.CAN.SHOOTER_BED_FOLLOWER);
        _absoluteEncoder       = new DutyCycleEncoder(Constants.DIO.SHOOTER_BED_SENSOR);
        _absoluteEncoderOffset = Constants.ShooterBed.BED_ANGLE_OFFSET;

        _leaderMotor.setInverted(true);
        _leaderMotor.setNeutralMode(NeutralMode.Brake);

        _followerMotor.follow(_leaderMotor);
    }

    @Override
    public void updateInputs(ShooterBedIOInputs inputs)
    {
        inputs.leaderVolts   = _leaderMotor.getMotorOutputVoltage();
        inputs.followerVolts = _followerMotor.getMotorOutputVoltage();
        inputs.bedAngle      = Rotation2d.fromDegrees(_absoluteEncoder.getAbsolutePosition() * Constants.ShooterBed.BED_SCALE - _absoluteEncoderOffset.getDegrees());
        // inputs.bedAngle =
        // Rotation2d.fromRotations(_absoluteEncoder.getAbsolutePosition()).times(Constants.ShooterBed.BED_SCALE).minus(_absoluteEncoderOffset);
        inputs.bedAngleDegrees = inputs.bedAngle.getDegrees();
    }

    @Override
    public void setVolts(double volts)
    {
        _leaderMotor.set(VictorSPXControlMode.PercentOutput, volts / Constants.General.MOTOR_VOLTAGE);
    }

    @Override
    public void setOffset(Rotation2d offset)
    {
        _absoluteEncoderOffset = offset;
    }
}
