package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.robot.Constants;

public class ModuleIOSim implements ModuleIO
{
    private final DCMotorSim _driveSim                 = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);
    private final DCMotorSim _turnSim                  = new DCMotorSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004);
    private Rotation2d       _turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
    private double           _driveVolts               = 0.0;
    private double           _turnVolts                = 0.0;

    @Override
    public void updateInputs(ModuleIOInputs inputs)
    {
        _driveSim.update(Constants.General.LOOP_PERIOD_SECS);
        _turnSim.update(Constants.General.LOOP_PERIOD_SECS);

        inputs.drivePositionRad       = _driveSim.getAngularPositionRad();
        inputs.driveVelocityRadPerSec = _driveSim.getAngularVelocityRadPerSec();
        inputs.driveVolts             = _driveVolts;
        inputs.driveCurrent           = new double[] { Math.abs(_driveSim.getCurrentDrawAmps()) };

        inputs.turnAbsolutePosition  = new Rotation2d(_turnSim.getAngularPositionRad()).plus(_turnAbsoluteInitPosition);
        inputs.turnPosition          = new Rotation2d(_turnSim.getAngularPositionRad());
        inputs.turnVelocityRadPerSec = _turnSim.getAngularVelocityRadPerSec();
        inputs.turnVolts             = _turnVolts;
        inputs.turnCurrent           = new double[] { Math.abs(_turnSim.getCurrentDrawAmps()) };
    }

    @Override
    public void setDriveVolts(double volts)
    {
        _driveVolts = MathUtil.clamp(volts, -Constants.General.MOTOR_VOLTAGE, Constants.General.MOTOR_VOLTAGE);
        _driveSim.setInputVoltage(_driveVolts);
    }

    @Override
    public void setTurnVolts(double volts)
    {
        _turnVolts = MathUtil.clamp(volts, -Constants.General.MOTOR_VOLTAGE, Constants.General.MOTOR_VOLTAGE);
        _turnSim.setInputVoltage(_turnVolts);
    }

    @Override
    public void setAngleOffset(Rotation2d angleOffset)
    {
        _turnAbsoluteInitPosition = angleOffset;
    }
}
