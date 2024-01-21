package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase
{
    private ShooterIO _io; 
    private final ShooterIOInputsAutoLogged _inputs = new ShooterIOInputsAutoLogged();


    /** Creates a new Shooter. */
    public Shooter()
    {

    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }
}
