package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class ShooterBedIOVictorSPX implements ShooterBedIO
{
    private VictorSPX   _bedLeaderVictorSPX;
    private VictorSPX   _bedFollowerVictorSPX;
    private DutyCycleEncoder _bedAbsoluteEncoder;
    private Rotation2d    _bedAbsoluteEncoderOffset;

    public ShooterBedIOVictorSPX()
    {
        _bedLeaderVictorSPX = new VictorSPX(Constants.CAN.SHOOTER_BED_LEADER);

        _bedFollowerVictorSPX = new VictorSPX(Constants.CAN.SHOOTER_BED_FOLLOWER);
        _bedFollowerVictorSPX.follow(_bedLeaderVictorSPX);
        _bedFollowerVictorSPX.setInverted(true);

        _bedAbsoluteEncoder       = new DutyCycleEncoder(Constants.AIO.SHOOTER_BED_SENSOR);
        _bedAbsoluteEncoderOffset = Constants.ShooterBed.BED_ANGLE_OFFSET;
    }

    @Override
    public void updateInputs(ShooterBedIOInputs inputs)
    {
        inputs.bedLeaderAppliedVolts = _bedLeaderVictorSPX.getMotorOutputVoltage();
        //inputs.bedLeaderCurrentAmps  = new double[] { _bedLeaderVictorSPX.getSupplyCurrent() };
        inputs.bedLeaderCurrentAmps = new double[]{0};
        
        inputs.bedFollowerAppliedVolts = _bedFollowerVictorSPX.getMotorOutputVoltage();
        //inputs.bedFollowerCurrentAmps  = new double[] { _bedFollowerVictorSPX.getOutputCurrent() };
        inputs.bedFollowerCurrentAmps = new double[]{0};

        inputs.bedAngle = Rotation2d.fromRotations(_bedAbsoluteEncoder.getAbsolutePosition()).minus(_bedAbsoluteEncoderOffset);
    }

    @Override
    public void setVoltage(double volts)
    {
        _bedLeaderVictorSPX.set(VictorSPXControlMode.PercentOutput, volts / Constants.General.MOTOR_VOLTAGE);
    }

    @Override
    public void setAngleOffset(Rotation2d bedAbsoluteEncoderOffset)
    {
        _bedAbsoluteEncoderOffset = bedAbsoluteEncoderOffset;
    }
}
