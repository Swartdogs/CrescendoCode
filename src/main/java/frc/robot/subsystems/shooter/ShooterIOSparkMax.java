package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.AnalogEncoder;

public class ShooterIOSparkMax implements ShooterIO
{
    private CANSparkMax _leftBaseSparkMax;
    private CANSparkMax _rightBaseSparkMax;
    private CANSparkMax _leftShooterSparkMax;
    private CANSparkMax _rightShooterSparkMax;

    private RelativeEncoder _leftBaseRelativeEncoder;
    private RelativeEncoder _rightBaseRelativeEncoder;
    private RelativeEncoder _leftShooterRelativeEncoder;
    private RelativeEncoder _rightShooterRelativeEncoder;

    private AnalogEncoder _leftBaseAbsoluteEncoder;
    private AnalogEncoder _rightBaseAbsoluteEncoder;
    private AnalogEncoder _leftShooterAbsoluteEncoder;
    private AnalogEncoder _rightShooterAbsoluteEncoder;
}
