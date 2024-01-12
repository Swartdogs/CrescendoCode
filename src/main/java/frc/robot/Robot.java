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

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.IOException;
import java.util.Optional;
import org.littletonrobotics.junction.LoggedRobot;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private PhotonCamera camera;
  private PhotonPoseEstimator poseEstimator;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // // Record metadata
    // Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    // Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    // Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    // Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    // Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    // switch (BuildConstants.DIRTY) {
    //   case 0:
    //     Logger.recordMetadata("GitDirty", "All changes committed");
    //     break;
    //   case 1:
    //     Logger.recordMetadata("GitDirty", "Uncomitted changes");
    //     break;
    //   default:
    //     Logger.recordMetadata("GitDirty", "Unknown");
    //     break;
    // }

    AprilTagFieldLayout aprilTagFieldLayout = null;

    try {
      // Load the AprilTag field layout from the resource file
      aprilTagFieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      System.out.println("Loaded the AprilTag field layout from the resource file");
    } catch (IOException e) {
      // Handle IOException specifically
      System.out.println("IOException encountered: " + e.getMessage());
      e.printStackTrace();
    } catch (Exception e) {
      // Handle any other exceptions
      System.out.println("An unexpected error occurred: " + e.getMessage());
      e.printStackTrace();
    }

    // Initialize the camera
    camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

    // Define the transform from the robot to the camera (assuming the camera is mounted facing
    // forward)
    Transform3d robotToCam =
        new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

    // Construct the PhotonPoseEstimator
    poseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);

    // Set up data receivers & replay source
    // switch (Constants.currentMode) {
    //   case REAL:
    //     // Running on a real robot, log to a USB stick ("/U/logs")
    //     Logger.addDataReceiver(new WPILOGWriter());
    //     Logger.addDataReceiver(new NT4Publisher());
    //     break;

    //   case SIM:
    //     // Running a physics simulator, log to NT
    //     Logger.addDataReceiver(new NT4Publisher());
    //     break;

    //   case REPLAY:
    //     // Replaying a log, set up replay source
    //     setUseTiming(false); // Run as fast as possible
    //     String logPath = LogFileUtil.findReplayLog();
    //     Logger.setReplaySource(new WPILOGReader(logPath));
    //     Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    //     break;
    // }

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
    // Logger.disableDeterministicTimestamps()

    // Start AdvantageKit logger
    // Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    // robotContainer = new RobotContainer();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    // CommandScheduler.getInstance().run();
    // Get the estimated global pose of the robot
    Optional<EstimatedRobotPose> estimatedPose = getEstimatedGlobalPose(memory);

    if (estimatedPose.isPresent()) {
      memory = estimatedPose.get().estimatedPose.toPose2d();
      // Do something with the estimated pose, like updating the robot's position on a dashboard
      System.out.println("Estimated Pose: " + memory);
    }
  }

  Pose2d memory = new Pose2d();

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    // Set the reference pose for the estimator
    poseEstimator.setReferencePose(prevEstimatedRobotPose);

    // Update the estimator and get the estimated robot pose
    // System.out.println("Updated the estimator");
    return poseEstimator.update();
  }
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
