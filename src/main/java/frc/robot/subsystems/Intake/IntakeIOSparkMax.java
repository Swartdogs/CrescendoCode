package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class IntakeIOSparkMax implements IntakeIO {
  private CANSparkMax _intakeSparkMax;
  private RelativeEncoder _intakeEncoder;


  public IntakeIOSparkMax(int canId) {
    _intakeSparkMax = new CANSparkMax(canId, MotorType.kBrushless);
    _intakeEncoder = _intakeSparkMax.getEncoder();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeSpeed = _intakeEncoder.getVelocity();
  }

  @Override
  public void setVoltage(double volts) {
    _intakeSparkMax.setVoltage(volts);
  }
}
