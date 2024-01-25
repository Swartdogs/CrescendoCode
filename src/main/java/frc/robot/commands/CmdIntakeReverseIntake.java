package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.Intake;

public class CmdIntakeReverseIntake extends Command
{
    private final Intake _intakeSubsystem;

    public CmdIntakeReverseIntake(Intake intakeSubsystem)
    {
        _intakeSubsystem = intakeSubsystem;

        addRequirements(_intakeSubsystem);
    }

    @Override
    public void initialize()
    {
        _intakeSubsystem.setIntakeVoltage(-Constants.Intake.INTAKE_VOLTAGE);
    }

    @Override
    public void end(boolean interrupted)
    {
        _intakeSubsystem.setIntakeVoltage(0);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
