package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Utilities
{
    private Utilities()
    {

    }

    public static Pose2d getAutoPose(Pose2d bluePose)
    {
        var                finalPose = bluePose;
        Optional<Alliance> alliance  = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
        {
            var distanceToMiddle = Units.inchesToMeters(325.625) - bluePose.getX();
            finalPose = new Pose2d(bluePose.getX() + (2 * distanceToMiddle), bluePose.getY(), bluePose.getRotation().unaryMinus().plus(Rotation2d.fromDegrees(180)));
        }

        return finalPose;
    }
}
