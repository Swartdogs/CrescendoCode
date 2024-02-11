package frc.robot.subsystems.notepath;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants;

public class NotepathIOSparkMax implements NotepathIO
{
    private final CANSparkMax  _leaderMotor;
    private final CANSparkMax  _followerMotor;
    private final DigitalInput _lightSensor;

    public NotepathIOSparkMax()
    {
        _leaderMotor   = new CANSparkMax(Constants.CAN.NOTEPATH_LEADER, MotorType.kBrushless);
        _followerMotor = new CANSparkMax(Constants.CAN.NOTEPATH_FOLLOWER, MotorType.kBrushless);

        _leaderMotor.restoreFactoryDefaults();
        _followerMotor.restoreFactoryDefaults();

        _leaderMotor.setCANTimeout(250);
        _followerMotor.setCANTimeout(250);

        _leaderMotor.setSmartCurrentLimit(20);
        _followerMotor.setSmartCurrentLimit(20);

        _leaderMotor.setCANTimeout(0);
        _followerMotor.setCANTimeout(0);

        _leaderMotor.burnFlash();
        _followerMotor.burnFlash();

        _followerMotor.follow(_leaderMotor, true);

        _lightSensor = new DigitalInput(Constants.DIO.NOTE_SENSOR);
    }

    @Override
    public void updateInputs(NotepathInputs inputs)
    {
        inputs.leaderVolts   = _leaderMotor.getAppliedOutput() * _leaderMotor.getBusVoltage();
        inputs.leaderCurrent = new double[] { _leaderMotor.getOutputCurrent() };

        inputs.followerVolts   = _followerMotor.getAppliedOutput() * _followerMotor.getBusVoltage();
        inputs.followerCurrent = new double[] { _followerMotor.getOutputCurrent() };

        inputs.sensorTripped = !_lightSensor.get();
    }

    @Override
    public void setVolts(double volts)
    {
        _leaderMotor.setVoltage(volts);
    }
}
