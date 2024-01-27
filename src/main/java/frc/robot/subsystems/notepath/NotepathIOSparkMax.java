package frc.robot.subsystems.notepath;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class NotepathIOSparkMax implements NotepathIO
{
    private CANSparkMax _notepathSparkMax;
    public NotepathIOSparkMax(int notePathCanId)
    {
        _notepathSparkMax = new CANSparkMax(notePathCanId, MotorType.kBrushless);
        
    }
    
}
