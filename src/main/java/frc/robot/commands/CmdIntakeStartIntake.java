package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class CmdIntakeStartIntake extends Command
{
    private final Intake _intakeSubsystem;

    public CmdIntakeStartIntake(Intake intakeSubsystem)
    {
        _intakeSubsystem = intakeSubsystem;

        addRequirements(_intakeSubsystem);
    }

    @Override
    public void initialize()
    {
        _intakeSubsystem.setIntakeOn();
    }

    @Override
    public void end(boolean interrupted)
    {
        _intakeSubsystem.setIntakeOff();
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
