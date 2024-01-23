package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase
{
    private ShooterBedIO _leftBedIO;
    private ShooterBedIO _rightBedIO;
    
    private final ShooterBedIOInputsAutoLogged _bedInputs = new ShooterBedIOInputsAutoLogged();
    private final 


    /** Creates a new Shooter. */
    public Shooter(ShooterBedIO leftBedIO, ShooterBedIO rightBedIO)
    {
        _leftBedIO = leftBedIO;
        _rightBedIO = rightBedIO;
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }
}
