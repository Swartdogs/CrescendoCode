package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.util.Simulation;

public class ClimbIOSim implements ClimbIO
{
    private final DCMotorSim          _leftArmSim  = new DCMotorSim(Simulation.windowMotor(), 2.5, 0.003); // Find values
    private final DCMotorSim          _rightArmSim = new DCMotorSim(Simulation.windowMotor(), 2.5, 0.003); // Find values
    private final MechanismLigament2d _leftArmLigament;
    private final MechanismLigament2d _rightArmLigament;
    private double                    _leftVolts   = 0.0;
    private double                    _rightVolts  = 0.0;

    public ClimbIOSim()
    {
        Mechanism2d mechanism = new Mechanism2d(30, 33);

        MechanismRoot2d robotLeft  = mechanism.getRoot("LeftArmRoot", 10, 0);
        MechanismRoot2d robotRight = mechanism.getRoot("RightArmRoot", 20, 0);

        _leftArmLigament  = robotLeft.append(new MechanismLigament2d("LeftArm", 2, 90, 15, new Color8Bit(Color.kOrange)));
        _rightArmLigament = robotRight.append(new MechanismLigament2d("RightArm", 2, 90, 15, new Color8Bit(Color.kOrange)));

        _leftArmLigament.append(new MechanismLigament2d("LeftHook", 4, 20, 15, new Color8Bit(Color.kOrange)));
        _rightArmLigament.append(new MechanismLigament2d("RightHook", 4, 20, 15, new Color8Bit(Color.kOrange)));

        SmartDashboard.putData("Climb", mechanism);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs)
    {
        _leftArmSim.update(Constants.General.LOOP_PERIOD_SECS);
        _rightArmSim.update(Constants.General.LOOP_PERIOD_SECS);

        inputs.leftExtension  = _leftArmSim.getAngularPositionRad(); // in per rad = 1
        inputs.rightExtension = _rightArmSim.getAngularPositionRad(); // in per rad = 1

        inputs.leftVolts  = _leftVolts;
        inputs.rightVolts = _rightVolts;

        _leftArmLigament.setLength(inputs.leftExtension);
        _rightArmLigament.setLength(inputs.rightExtension);
    }

    @Override
    public void setLeftVolts(double volts)
    {
        _leftVolts = MathUtil.clamp(volts, -Constants.General.MOTOR_VOLTAGE, Constants.General.MOTOR_VOLTAGE);
        _leftArmSim.setInputVoltage(_leftVolts);
    }

    @Override
    public void setRightVolts(double volts)
    {
        _rightVolts = MathUtil.clamp(volts, -Constants.General.MOTOR_VOLTAGE, Constants.General.MOTOR_VOLTAGE);
        _rightArmSim.setInputVoltage(_rightVolts);
    }
}
