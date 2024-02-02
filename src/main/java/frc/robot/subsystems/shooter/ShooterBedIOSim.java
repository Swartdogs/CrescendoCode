// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

public class ShooterBedIOSim implements ShooterBedIO
{
    private DCMotorSim          _leftBedSim           = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);
    private DCMotorSim          _rightBedSim          = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);
    private double              _leftBedAppliedVolts  = 0.0;
    private double              _rightBedAppliedVolts = 0.0;
    private Rotation2d          _angleOffset          = Constants.ShooterBed.BED_ANGLE_OFFSET;
    private MechanismLigament2d _bedSim;

    public ShooterBedIOSim()
    {
        Mechanism2d     mechanism = new Mechanism2d(3, 3);
        MechanismRoot2d robot     = mechanism.getRoot("Shooter Bed", 2, 0);
        _bedSim = robot.append(new MechanismLigament2d("Bed", 1, 90, 20, new Color8Bit(Color.kOrange)));
        SmartDashboard.putData("Shooter Bed", mechanism);
    }

    @Override
    public void updateInputs(ShooterBedIOInputs inputs)
    {
        _leftBedSim.update(Constants.General.LOOP_PERIOD_SECS);
        _rightBedSim.update(Constants.General.LOOP_PERIOD_SECS);

        inputs.bedAngle              = new Rotation2d(_leftBedSim.getAngularPositionRad() - _angleOffset.getRadians());
        inputs.bedLeaderAppliedVolts = _leftBedAppliedVolts;
        inputs.bedLeaderCurrentAmps  = new double[] { Math.abs(_leftBedSim.getCurrentDrawAmps()) };

        inputs.bedFollowerAppliedVolts = _rightBedAppliedVolts;
        inputs.bedFollowerCurrentAmps  = new double[] { Math.abs(_rightBedSim.getCurrentDrawAmps()) };

        _bedSim.setAngle(inputs.bedAngle);
    }

    @Override
    public void setVoltage(double volts)
    {
        _leftBedAppliedVolts  = volts;
        _rightBedAppliedVolts = -volts;
        _leftBedSim.setInputVoltage(_leftBedAppliedVolts);
        _rightBedSim.setInputVoltage(_rightBedAppliedVolts);
    }

    @Override
    public void setAngleOffset(Rotation2d angleOffset)
    {
        _angleOffset = angleOffset;
    }
}
