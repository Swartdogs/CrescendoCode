// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

public class ShooterBedIOSim implements ShooterBedIO
{
    private DCMotorSim _leftBedSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);
    private DCMotorSim _rightBedSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);

    private double _bedAppliedVolts = 0.0;

    private Rotation2d _angleOffset = Constants.ShooterBed.BED_ANGLE_OFFSET;

    private Mechanism2d _mechanism;
    private MechanismRoot2d _robot;
    private MechanismLigament2d _bed;

    public ShooterBedIOSim()
    {
        _mechanism = new Mechanism2d(3, 3);
        _robot = _mechanism.getRoot("Shooter Bed", 2, 0);
        _bed = _robot.append(new MechanismLigament2d("Bed", 1, 90, 20, new Color8Bit(Color.kOrange)));

    }

    @Override
    public void updateInputs(ShooterBedIOInputs inputs)
    {
        _leftBedSim.update(Constants.General.LOOP_PERIOD_SECS);
        _rightBedSim.update(Constants.General.LOOP_PERIOD_SECS);

        Logger.recordOutput("Shooter/Mech", _mechanism);

        inputs.bedAngle = new Rotation2d(_leftBedSim.getAngularPositionRad() - _angleOffset.getRadians());
        inputs.bedAppliedVolts = _bedAppliedVolts;
        inputs.bedCurrentAmps = new double[]
        { Math.abs(_leftBedSim.getCurrentDrawAmps()) };

        _bed.setAngle(inputs.bedAngle);
    }

    @Override
    public void setVoltage(double volts)
    {
        _leftBedSim.setInputVoltage(volts);
        _rightBedSim.setInputVoltage(volts);
        _bedAppliedVolts = volts;
    }

    @Override
    public void setAngleOffset(Rotation2d angleOffset)
    {
        _angleOffset = angleOffset;
    }
}
