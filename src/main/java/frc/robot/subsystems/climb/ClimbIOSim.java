// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

public class ClimbIOSim implements ClimbIO
{
    private DCMotor                   _leftArm           = new DCMotor(12, 9.2, 16.3, 1.6, Units.rotationsPerMinuteToRadiansPerSecond(90), 1);
    private DCMotor                   _rightArm          = new DCMotor(12, 9.2, 16.3, 1.6, Units.rotationsPerMinuteToRadiansPerSecond(90), 1);
    private DCMotorSim                _leftSim           = new DCMotorSim(_leftArm, 2.5, 0.003); // Find values
    private DCMotorSim                _rightSim          = new DCMotorSim(_rightArm, 2.5, 0.003); // Find values
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
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs)
    {
        _leftSim.update(Constants.General.LOOP_PERIOD_SECS);
        _rightSim.update(Constants.General.LOOP_PERIOD_SECS);

        inputs.extensionLeft  = _leftSim.getAngularPositionRad(); // in per rad = 1
        inputs.extensionRight = _rightSim.getAngularPositionRad(); // in per rad = 1

        inputs.appliedVoltsLeft  = _leftAppliedVolts;
        inputs.appliedVoltsRight = _rightAppliedVolts;

        _climbLeft.setLength(inputs.extensionLeft);
        _climbRight.setLength(inputs.extensionRight);
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
        _rightAppliedVolts = MathUtil.clamp(volts, -Constants.Climb.MOTOR_VOLTAGE_LIMIT, Constants.Climb.MOTOR_VOLTAGE_LIMIT);
        _rightSim.setInputVoltage(_rightAppliedVolts);
    }
}
