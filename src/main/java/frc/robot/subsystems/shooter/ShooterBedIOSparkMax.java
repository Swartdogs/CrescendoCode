package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;

public class ShooterBedIOSparkMax implements ShooterBedIO
{
    private CANSparkMax _bedSparkMax;

    private AnalogEncoder _bedAbsoluteEncoder;
    private Rotation2d _bedAbsoluteEncoderOffset;

    @SuppressWarnings("resource")
    public ShooterBedIOSparkMax(int bedCanID, int followerBedCanID, int absoluteEncoderChannel,
                    Rotation2d bedAbsoluteEncoderOffset)
    {
        _bedSparkMax = new CANSparkMax(bedCanID, MotorType.kBrushless);

        CANSparkMax followerBedSparkMax = new CANSparkMax(followerBedCanID, MotorType.kBrushless);
        followerBedSparkMax.follow(_bedSparkMax, true);

        _bedAbsoluteEncoder = new AnalogEncoder(absoluteEncoderChannel);
        _bedAbsoluteEncoderOffset = bedAbsoluteEncoderOffset;
    }

    @Override
    public void updateInputs(ShooterBedIOInputs inputs)
    {
        inputs.bedAppliedVolts = _bedSparkMax.getAppliedOutput() * _bedSparkMax.getBusVoltage();
        inputs.bedCurrentAmps = new double[]
        { _bedSparkMax.getOutputCurrent() };

        inputs.bedAngle = Rotation2d.fromRotations(_bedAbsoluteEncoder.getAbsolutePosition())
                        .minus(_bedAbsoluteEncoderOffset);
    }

    @Override
    public void setVoltage(double volts)
    {
        _bedSparkMax.setVoltage(volts);
    }

    @Override
    public void setAngleOffset(Rotation2d bedAbsoluteEncoderOffset)
    {
        _bedAbsoluteEncoderOffset = bedAbsoluteEncoderOffset;
    }
}
