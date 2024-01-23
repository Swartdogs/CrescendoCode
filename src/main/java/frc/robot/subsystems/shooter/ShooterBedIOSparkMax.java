package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;

public class ShooterBedIOSparkMax implements ShooterBedIO
{
    private CANSparkMax _bedSparkMax;

    private RelativeEncoder _bedRelativeEncoder;

    private AnalogEncoder _bedAbsoluteEncoder;

    private Rotation2d _bedAbsoluteEncoderOffset;

    private double _shooterAngle;

    public ShooterBedIOSparkMax(int bedCanID, int absoluteEncoderChannel, Rotation2d bedAbsoluteEncoderOffset)
    {
        _bedSparkMax = new CANSparkMax(bedCanID, MotorType.kBrushless);
        _bedRelativeEncoder = _bedSparkMax.getEncoder();
        _bedAbsoluteEncoder = new AnalogEncoder(absoluteEncoderChannel);
        _bedAbsoluteEncoderOffset = bedAbsoluteEncoderOffset;
    }

    @Override
    public void updateInputs(ShooterBedIOInputs inputs)
    {
        inputs.positionRad = Units.rotationsToDegrees(_bedRelativeEncoder.getPosition());
        inputs.absolutePosition = Rotation2d.fromRotations(_bedAbsoluteEncoder.getAbsolutePosition())
        .minus(_bedAbsoluteEncoderOffset);
        inputs.shooterAngle = _shooterAngle;
        inputs.bedAppliedVolts = _bedSparkMax.getAppliedOutput() * _bedSparkMax.getBusVoltage();
        inputs.bedCurrentAmps = new double[]{_bedSparkMax.getOutputCurrent()};
    }

    @Override
    public void setVoltage(double volts)
    {
        _bedSparkMax.setVoltage(volts);
    }
}
