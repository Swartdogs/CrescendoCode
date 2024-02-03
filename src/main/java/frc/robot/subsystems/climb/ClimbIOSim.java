// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.simulation.AnalogEncoderSim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SolenoidSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

public class ClimbIOSim implements ClimbIO
{
    private DCMotorSim _leftSim  = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.003); // Find values
    private DCMotorSim _rightSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.003); // Find values

    private AnalogEncoderSim _leftEncoderSim  = new AnalogEncoderSim(new AnalogEncoder(Constants.AIO.CLIMB_LEFT_SENSOR));
    private AnalogEncoderSim _rightEncoderSim = new AnalogEncoderSim(new AnalogEncoder(Constants.AIO.CLIMB_RIGHT_SENSOR));

    private SolenoidSim _leftSolenoidSim  = new SolenoidSim(Constants.Pnuematics.MODULE_TYPE, Constants.Pnuematics.SOLENOID_LEFT);
    private SolenoidSim _rightSolenoidSim = new SolenoidSim(Constants.Pnuematics.MODULE_TYPE, Constants.Pnuematics.SOLENOID_RIGHT);

    private double _leftAppliedVolts  = 0.0;
    private double _rightAppliedVolts = 0.0;

    private final MechanismLigament2d _climbLeft;
    private final MechanismLigament2d _climbRight;

    public ClimbIOSim()
    {
        Mechanism2d mechanismLeft  = new Mechanism2d(3, 3);
        Mechanism2d mechanismRight = new Mechanism2d(3, 3);

        MechanismRoot2d robotLeft  = mechanismLeft.getRoot("Climb1", 1, 0);
        MechanismRoot2d robotRight = mechanismRight.getRoot("Climb2", 1.75, 0);

        MechanismLigament2d ligLeft  = robotLeft.append(new MechanismLigament2d("lig1", 0.01, 90, 20, new Color8Bit(Color.kOrange)));
        MechanismLigament2d ligRight = robotRight.append(new MechanismLigament2d("lig2", 0.01, 90, 20, new Color8Bit(Color.kOrange)));

        _climbLeft  = ligLeft.append(new MechanismLigament2d("SubClimb1", 2, 10, 10, new Color8Bit(Color.kOrange)));
        _climbRight = ligRight.append(new MechanismLigament2d("SubClimb2", 2, 10, 10, new Color8Bit(Color.kOrange)));

        _climbLeft.append(new MechanismLigament2d("HookLeft", 0.4, 20, 10, new Color8Bit(Color.kOrange)));
        _climbRight.append(new MechanismLigament2d("HookLeft", 0.4, 20, 10, new Color8Bit(Color.kOrange)));

        SmartDashboard.putData("Climb1", mechanismLeft);
        SmartDashboard.putData("Climb2", mechanismRight);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs)
    {
        _leftSim.update(Constants.General.LOOP_PERIOD_SECS);
        _rightSim.update(Constants.General.LOOP_PERIOD_SECS);

        double leftDelta  = Constants.Climb.CLIMB_SENSOR_RATE_DEG_PER_SEC * Constants.General.LOOP_PERIOD_SECS * (_leftAppliedVolts / Constants.Climb.MOTOR_VOLTAGE_LIMIT); 
        double rightDelta = Constants.Climb.CLIMB_SENSOR_RATE_DEG_PER_SEC * Constants.General.LOOP_PERIOD_SECS * (_rightAppliedVolts / Constants.Climb.MOTOR_VOLTAGE_LIMIT);

        _leftEncoderSim.setPosition(_leftEncoderSim.getPosition().plus(Rotation2d.fromDegrees(leftDelta)));
        _rightEncoderSim.setPosition(_rightEncoderSim.getPosition().plus(Rotation2d.fromDegrees(rightDelta)));

        inputs.extensionLeft  = _leftEncoderSim.getPosition().getDegrees() / Constants.Climb.CLIMB_SENSOR_DEG_PER_INCH;
        inputs.extensionRight = _rightEncoderSim.getPosition().getDegrees() / Constants.Climb.CLIMB_SENSOR_DEG_PER_INCH;

        inputs.lockStateLeft  = !_leftSolenoidSim.getOutput();
        inputs.lockStateRight = !_rightSolenoidSim.getOutput();

        inputs.appliedVoltsLeft  = _leftAppliedVolts;
        inputs.appliedVoltsRight = _rightAppliedVolts;

        inputs.currentAmpsLeft = new double[] { Math.abs(_leftSim.getCurrentDrawAmps()) };
        inputs.currentAmpsRight = new double[] { Math.abs(_rightSim.getCurrentDrawAmps()) };

        _climbLeft.setLength(inputs.extensionLeft);
        _climbRight.setLength(inputs.extensionRight);
        
        if (inputs.extensionLeft > 1.6)
        {
            inputs.extensionLeft = 1.6;
            _climbLeft.setLength(inputs.extensionLeft);
            _leftEncoderSim.setPosition(Rotation2d.fromRadians(1.6));
        }
        else if (inputs.extensionLeft < 0)
        {
            inputs.extensionLeft = 0;
            _climbLeft.setLength(inputs.extensionLeft);
            _leftEncoderSim.setPosition(Rotation2d.fromRadians(0));
        }

        if (inputs.extensionRight > 1.6)
        {
            inputs.extensionRight = 1.6;
            _climbRight.setLength(inputs.extensionRight);
            _rightEncoderSim.setPosition(Rotation2d.fromRadians(1.6));
        }
        else if (inputs.extensionRight < 0)
        {
            inputs.extensionRight = 0;
            _climbRight.setLength(inputs.extensionRight);
            _rightEncoderSim.setPosition(Rotation2d.fromRadians(0));
        }
    }

    @Override
    public void setVoltageLeft(double volts)
    {
        _leftAppliedVolts = MathUtil.clamp(volts, -Constants.Climb.MOTOR_VOLTAGE_LIMIT, Constants.Climb.MOTOR_VOLTAGE_LIMIT);
        _leftSim.setInputVoltage(_leftAppliedVolts);
    }

    @Override
    public void setVoltageRight(double volts)
    {
        _rightAppliedVolts = MathUtil.clamp(volts, -Constants.Climb.MOTOR_VOLTAGE_LIMIT, Constants.Climb.MOTOR_VOLTAGE_LIMIT);
        _rightSim.setInputVoltage(_rightAppliedVolts);
    }

    @Override
    public void setLockStateLeft(boolean enable, ClimbIOInputs inputs)
    {
        if (inputs.lockStateLeft != enable)
        {
            _leftSolenoidSim.setOutput(!enable);
        }
    }

    @Override
    public void setLockStateRight(boolean enable, ClimbIOInputs inputs)
    {
        if (inputs.lockStateRight != enable)
        {
            _rightSolenoidSim.setOutput(!enable);
        }
    }
}
