package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.util.Simulation;

public class ShooterBedIOSim implements ShooterBedIO
{
    private final DCMotorSim          _leftMotorSim  = new DCMotorSim(Simulation.windowMotor(), 6.75, 0.025);
    private final DCMotorSim          _rightMotorSim = new DCMotorSim(Simulation.windowMotor(), 6.75, 0.025);
    private final MechanismLigament2d _bedLigament;
    private double                    _leftVolts     = 0.0;
    private double                    _rightVolts    = 0.0;
    private Rotation2d                _angleOffset   = Constants.ShooterBed.BED_ANGLE_OFFSET;

    public ShooterBedIOSim()
    {
        Mechanism2d     mechanism = new Mechanism2d(3, 3);
        MechanismRoot2d bedRoot   = mechanism.getRoot("Shooter Bed", 2, 0);

        _bedLigament = bedRoot.append(new MechanismLigament2d("Bed", 1, 90, 20, new Color8Bit(Color.kOrange)));

        SmartDashboard.putData("Shooter Bed", mechanism);
    }

    @Override
    public void updateInputs(ShooterBedIOInputs inputs)
    {
        _leftMotorSim.update(Constants.General.LOOP_PERIOD_SECS);
        _rightMotorSim.update(Constants.General.LOOP_PERIOD_SECS);

        inputs.bedAngle      = new Rotation2d(_leftMotorSim.getAngularPositionRad()).minus(_angleOffset);
        inputs.leaderVolts   = _leftVolts;
        inputs.followerVolts = _rightVolts;

        _bedLigament.setAngle(inputs.bedAngle);
    }

    @Override
    public void setVolts(double volts)
    {
        volts       = MathUtil.clamp(volts, -Constants.General.MOTOR_VOLTAGE, Constants.General.MOTOR_VOLTAGE);
        _leftVolts  = volts;
        _rightVolts = volts;

        _leftMotorSim.setInputVoltage(_leftVolts);
        _rightMotorSim.setInputVoltage(_rightVolts);
    }

    @Override
    public void setOffset(Rotation2d angleOffset)
    {
        _angleOffset = angleOffset;
    }
}
