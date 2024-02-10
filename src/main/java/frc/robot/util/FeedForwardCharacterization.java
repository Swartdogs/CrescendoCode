// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class FeedForwardCharacterization extends Command
{
    private final Consumer<Double>          _voltageConsumer;
    private final Supplier<Double>          _velocitySupplier;
    private final Timer                     _timer = new Timer();
    private FeedForwardCharacterizationData _data;

    /** Creates a new FeedForwardCharacterization command. */
    public FeedForwardCharacterization(Subsystem subsystem, Consumer<Double> voltageConsumer, Supplier<Double> velocitySupplier)
    {
        addRequirements(subsystem);

        _voltageConsumer  = voltageConsumer;
        _velocitySupplier = velocitySupplier;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        _data = new FeedForwardCharacterizationData();

        _timer.reset();
        _timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        if (_timer.get() < Constants.Characterization.START_DELAY_SECS)
        {
            _voltageConsumer.accept(0.0);
        }
        else
        {
            double voltage = (_timer.get() - Constants.Characterization.START_DELAY_SECS) * Constants.Characterization.RAMP_VOLTS_PER_SEC;
            _voltageConsumer.accept(voltage);
            _data.add(_velocitySupplier.get(), voltage);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        _voltageConsumer.accept(0.0);
        _timer.stop();
        _data.print();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }

    public static class FeedForwardCharacterizationData
    {
        private final List<Double> _velocityData = new LinkedList<>();
        private final List<Double> _voltageData  = new LinkedList<>();

        public void add(double velocity, double voltage)
        {
            if (Math.abs(velocity) > 1E-4)
            {
                _velocityData.add(Math.abs(velocity));
                _voltageData.add(Math.abs(voltage));
            }
        }

        public void print()
        {
            if (_velocityData.size() == 0 || _voltageData.size() == 0)
            {
                return;
            }

            PolynomialRegression regression = new PolynomialRegression(_velocityData.stream().mapToDouble(Double::doubleValue).toArray(), _voltageData.stream().mapToDouble(Double::doubleValue).toArray(), 1);

            System.out.println("FF Characterization Results:");
            System.out.println("\tCount=" + Integer.toString(_velocityData.size()) + "");
            System.out.println(String.format("\tR2=%.5f", regression.R2()));
            System.out.println(String.format("\tkS=%.5f", regression.beta(0)));
            System.out.println(String.format("\tkV=%.5f", regression.beta(1)));
        }
    }
}
