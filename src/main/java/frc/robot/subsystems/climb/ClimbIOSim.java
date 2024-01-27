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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ClimbIOSim implements ClimbIO
{
    private static final double CLIMB_SENSOR_RATE_DEG_PER_SEC = 360;
    private static final double MOTOR_VOLTAGE_LIMIT = 12;
    private static final double CLIMB_SENSOR_DEG_PER_INCH = 60;

    private DCMotorSim _leftSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.003); // Find values
    private DCMotorSim _rightSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.003); // Find values

    private AnalogEncoderSim _leftEncoderSim = new AnalogEncoderSim(new AnalogEncoder(Constants.AIO.CLIMB_LEFT_SENSOR));
    private AnalogEncoderSim _rightEncoderSim = new AnalogEncoderSim(
            new AnalogEncoder(Constants.AIO.CLIMB_RIGHT_SENSOR));

    private SolenoidSim _leftSolenoidSim = new SolenoidSim(Constants.Pnuematics.MODULE_TYPE,
            Constants.Pnuematics.SOLENOID_LEFT);
    private SolenoidSim _rightSolenoidSim = new SolenoidSim(Constants.Pnuematics.MODULE_TYPE,
            Constants.Pnuematics.SOLENOID_RIGHT);

    private double _leftAppliedVolts = 0.0;
    private double _rightAppliedVolts = 0.0;

    private final MechanismLigament2d _climb;

    public ClimbIOSim()
    {
        Mechanism2d mechanism = new Mechanism2d(3, 3);
        MechanismRoot2d robot = mechanism.getRoot("Climb", 2, 0);
        MechanismLigament2d lig = robot.append(new MechanismLigament2d("lig", 1, 90, 20, new Color8Bit(Color.kOrange)));

        _climb = lig.append(new MechanismLigament2d("SubClimb", 0.5, 45, 10, new Color8Bit(Color.kOrange)));

        Logger.recordOutput("ClimbMech", mechanism);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs)
    {
        _leftSim.update(Constants.General.LOOP_PERIOD_SECS);
        _rightSim.update(Constants.General.LOOP_PERIOD_SECS);

        double leftDelta = CLIMB_SENSOR_RATE_DEG_PER_SEC * Constants.General.LOOP_PERIOD_SECS
                * (_leftAppliedVolts / MOTOR_VOLTAGE_LIMIT);
        double rightDelta = CLIMB_SENSOR_RATE_DEG_PER_SEC * Constants.General.LOOP_PERIOD_SECS
                * (_rightAppliedVolts / MOTOR_VOLTAGE_LIMIT);

        _leftEncoderSim.setPosition(_leftEncoderSim.getPosition().plus(Rotation2d.fromDegrees(leftDelta)));
        _rightEncoderSim.setPosition(_rightEncoderSim.getPosition().plus(Rotation2d.fromDegrees(rightDelta)));

        inputs.extensionLeft = _leftEncoderSim.getPosition().getDegrees() / CLIMB_SENSOR_DEG_PER_INCH;
        inputs.extensionRight = _rightEncoderSim.getPosition().getDegrees() / CLIMB_SENSOR_DEG_PER_INCH;

        inputs.lockStateLeft = !_leftSolenoidSim.getOutput();
        inputs.lockStateRight = !_rightSolenoidSim.getOutput();

        inputs.appliedVoltsLeft = _leftAppliedVolts;
        inputs.appliedVoltsRight = _rightAppliedVolts;

        inputs.currentAmpsLeft = new double[]
        { Math.abs(_leftSim.getCurrentDrawAmps()) };
        inputs.currnetAmpsRight = new double[]
        { Math.abs(_rightSim.getCurrentDrawAmps()) };

        _climb.setAngle(inputs.extensionLeft);
    }

    @Override
    public void setVoltageLeft(double volts)
    {
        _leftAppliedVolts = MathUtil.clamp(volts, -MOTOR_VOLTAGE_LIMIT, MOTOR_VOLTAGE_LIMIT); // Figure out how this
        // // transfers to simulation
        _leftSim.setInputVoltage(_leftAppliedVolts);
    }

    @Override
    public void setVoltageRight(double volts)
    {
        _rightAppliedVolts = MathUtil.clamp(volts, -MOTOR_VOLTAGE_LIMIT, MOTOR_VOLTAGE_LIMIT);
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
