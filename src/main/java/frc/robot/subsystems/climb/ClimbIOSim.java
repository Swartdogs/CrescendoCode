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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

public class ClimbIOSim implements ClimbIO
{
    private DCMotorSim                _leftSim           = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.003); // Find values
    private DCMotorSim                _rightSim          = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.003); // Find values
    private AnalogEncoderSim          _leftEncoderSim    = new AnalogEncoderSim(new AnalogEncoder(Constants.AIO.CLIMB_LEFT_SENSOR));
    private AnalogEncoderSim          _rightEncoderSim   = new AnalogEncoderSim(new AnalogEncoder(Constants.AIO.CLIMB_RIGHT_SENSOR));
    private double                    _leftAppliedVolts  = 0.0;
    private double                    _rightAppliedVolts = 0.0;
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

        double leftDelta  = Constants.Climb.CLIMB_SENSOR_RATE_DEG_PER_SEC * Constants.General.LOOP_PERIOD_SECS * (_leftAppliedVolts / Constants.General.MOTOR_VOLTAGE);
        double rightDelta = Constants.Climb.CLIMB_SENSOR_RATE_DEG_PER_SEC * Constants.General.LOOP_PERIOD_SECS * (_rightAppliedVolts / Constants.General.MOTOR_VOLTAGE);

        _leftEncoderSim.setPosition(_leftEncoderSim.getPosition().plus(Rotation2d.fromDegrees(leftDelta)));
        _rightEncoderSim.setPosition(_rightEncoderSim.getPosition().plus(Rotation2d.fromDegrees(rightDelta)));

        inputs.extensionLeft  = _leftEncoderSim.getPosition().getDegrees() / Constants.Climb.CLIMB_SENSOR_DEG_PER_INCH;
        inputs.extensionRight = _rightEncoderSim.getPosition().getDegrees() / Constants.Climb.CLIMB_SENSOR_DEG_PER_INCH;

        inputs.appliedVoltsLeft  = _leftAppliedVolts;
        inputs.appliedVoltsRight = _rightAppliedVolts;

        _climbLeft.setLength(inputs.extensionLeft);
        _climbRight.setLength(inputs.extensionRight);

        inputs.extensionLeft = MathUtil.clamp(inputs.extensionLeft, 0, 1.6);
        _climbLeft.setLength(inputs.extensionLeft);
        _leftEncoderSim.setPosition(Rotation2d.fromRadians(inputs.extensionLeft));

        inputs.extensionRight = MathUtil.clamp(inputs.extensionRight, 0, 1.6);
        _climbRight.setLength(inputs.extensionRight);
        _rightEncoderSim.setPosition(Rotation2d.fromRadians(inputs.extensionRight));
    }

    @Override
    public void setVoltageLeft(double volts)
    {
        _leftAppliedVolts = MathUtil.clamp(volts, -Constants.General.MOTOR_VOLTAGE, Constants.General.MOTOR_VOLTAGE);
        _leftSim.setInputVoltage(_leftAppliedVolts);
    }

    @Override
    public void setVoltageRight(double volts)
    {
        _rightAppliedVolts = MathUtil.clamp(volts, -Constants.General.MOTOR_VOLTAGE, Constants.General.MOTOR_VOLTAGE);
        _rightSim.setInputVoltage(_rightAppliedVolts);
    }
}
