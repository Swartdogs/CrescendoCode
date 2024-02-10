package frc.robot.subsystems.notepath;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class NotepathIOSparkMax implements NotepathIO
{
    private CANSparkMax  _notepathSparkMax;
    private CANSparkMax  _followerSparkMax;
    private DigitalInput _noteSensor;

    public NotepathIOSparkMax()
    {
        _notepathSparkMax = new CANSparkMax(Constants.CAN.NOTEPATH_LEADER, MotorType.kBrushless);
        _followerSparkMax = new CANSparkMax(Constants.CAN.NOTEPATH_FOLLOWER, MotorType.kBrushless);
        _followerSparkMax.follow(_notepathSparkMax, true);

        _noteSensor = new DigitalInput(Constants.DIO.NOTE_SENSOR);
    }

    @Override
    public void updateInputs(NotepathInputs inputs)
    {
        inputs.leaderNotepathAppliedVolts = _notepathSparkMax.getAppliedOutput() * _notepathSparkMax.getBusVoltage();
        inputs.leaderNotepathCurrentAmps  = new double[] { _notepathSparkMax.getOutputCurrent() };

        inputs.followerNotepathAppliedVolts = _followerSparkMax.getAppliedOutput() * _followerSparkMax.getBusVoltage();
        inputs.followerNotepathCurrentAmps  = new double[] { _followerSparkMax.getOutputCurrent() };

        inputs.sensorTripped = !_noteSensor.get();
    }

    @Override
    public void setVoltage(double volts)
    {
        _notepathSparkMax.setVoltage(volts);
    }
}
