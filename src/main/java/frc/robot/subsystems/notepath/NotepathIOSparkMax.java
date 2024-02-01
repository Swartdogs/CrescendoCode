package frc.robot.subsystems.notepath;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class NotepathIOSparkMax implements NotepathIO
{
    private CANSparkMax _notepathSparkMax;
    private CANSparkMax _followerSparkMax;

    @SuppressWarnings("resource")
    public NotepathIOSparkMax()
    {
        _notepathSparkMax = new CANSparkMax(Constants.CAN.NOTEPATH_LEADER, MotorType.kBrushless);
        _followerSparkMax = new CANSparkMax(Constants.CAN.NOTEPATH_FOLLOWER, MotorType.kBrushless);
        _followerSparkMax.follow(_notepathSparkMax, true);
    }

    @Override
    public void updateInputs(NotepathInputs inputs)
    {
        inputs.leaderNotepathAppliedVolts = _notepathSparkMax.getAppliedOutput() * _notepathSparkMax.getBusVoltage();
        inputs.leaderNotepathCurrentAmps  = new double[] {_notepathSparkMax.getOutputCurrent() };

        inputs.followerNotepathAppliedVolts = _followerSparkMax.getAppliedOutput() * _followerSparkMax.getBusVoltage();
        inputs.followerNotepathCurrentAmps  = new double[] {_followerSparkMax.getOutputCurrent() };
    }

    @Override
    public void setVoltage(double volts)
    {
        _notepathSparkMax.setVoltage(volts);
    }
}
