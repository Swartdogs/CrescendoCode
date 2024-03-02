package frc.robot.subsystems.gyro;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

public class GyroIOSim implements GyroIO
{
    private final Supplier<ChassisSpeeds> _chassisSupplier;
    private Rotation2d                    _yaw = Rotation2d.fromDegrees(0);

    public GyroIOSim(Supplier<ChassisSpeeds> chassisSupplier)
    {
        _chassisSupplier = chassisSupplier;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs)
    {
        var chassisSpeeds = _chassisSupplier.get();

        _yaw = _yaw.plus(Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond * Constants.General.LOOP_PERIOD_SECS));

        inputs.rollPosition         = Rotation2d.fromDegrees(0);
        inputs.yawPosition          = _yaw;
        inputs.yawVelocityRadPerSec = chassisSpeeds.omegaRadiansPerSecond;
    }
}
