package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants;

public class ShooterBedIOSparkMax implements ShooterBedIO
{
    private CANSparkMax   _bedLeaderSparkMax;
    private CANSparkMax   _bedFollowerSparkMax;
    private AnalogEncoder _bedAbsoluteEncoder;
    private Rotation2d    _bedAbsoluteEncoderOffset;

    @SuppressWarnings("resource")
    public ShooterBedIOSparkMax(Rotation2d bedAbsoluteEncoderOffset)
    {
        _bedLeaderSparkMax = new CANSparkMax(Constants.CAN.SHOOTER_BED_LEADER, MotorType.kBrushless);

        _bedFollowerSparkMax = new CANSparkMax(Constants.CAN.SHOOTER_BED_FOLLOWER, MotorType.kBrushless);
        _bedFollowerSparkMax.follow(_bedLeaderSparkMax, true);

        _bedAbsoluteEncoder       = new AnalogEncoder(Constants.AIO.SHOOTER_BED_SENSOR);
        _bedAbsoluteEncoderOffset = bedAbsoluteEncoderOffset;
    }

    @Override
    public void updateInputs(ShooterBedIOInputs inputs)
    {
        inputs.bedLeaderAppliedVolts = _bedLeaderSparkMax.getAppliedOutput() * _bedLeaderSparkMax.getBusVoltage();
        inputs.bedLeaderCurrentAmps  = new double[] { _bedLeaderSparkMax.getOutputCurrent() };
        
        inputs.bedFollowerAppliedVolts = _bedFollowerSparkMax.getAppliedOutput() * _bedFollowerSparkMax.getBusVoltage();
        inputs.bedFollowerCurrentAmps  = new double[] { _bedFollowerSparkMax.getOutputCurrent() };

        inputs.bedAngle = Rotation2d.fromRotations(_bedAbsoluteEncoder.getAbsolutePosition()).minus(_bedAbsoluteEncoderOffset);
    }

    @Override
    public void setVoltage(double volts)
    {
        _bedLeaderSparkMax.setVoltage(volts);
    }

    @Override
    public void setAngleOffset(Rotation2d bedAbsoluteEncoderOffset)
    {
        _bedAbsoluteEncoderOffset = bedAbsoluteEncoderOffset;
    }
}
