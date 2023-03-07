package frc.robot.commands.swervedrive2.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.ArrayList;
import java.util.List;

public class GoToPose {
  private PPSwerveControllerCommand ppSwerveCommand;
  private PathPlannerTrajectory traj;
  private Pose2d currentPose;
  private Pose2d pose;

  public GoToPose(
      Pose2d poseIn, Rotation2d heading, PathConstraints constraints, SwerveSubsystem drive) {
    this.pose = poseIn;
    PathPoint currentPathPoint;
    currentPose = drive.getPose();
    Alliance ally = DriverStation.getAlliance();
    if (ally == Alliance.Red) {
      currentPose =
          new Pose2d(
              currentPose.getX(), 8.02 - currentPose.getY(), currentPose.getRotation().times(-1));
    }
    if (Math.hypot(
            drive.getFieldVelocity().vxMetersPerSecond, drive.getFieldVelocity().vyMetersPerSecond)
        > 0.2) {
      currentPathPoint = PathPoint.fromCurrentHolonomicState(currentPose, drive.getFieldVelocity());
    } else {
      currentPathPoint =
          new PathPoint(
              currentPose.getTranslation(),
              pose.getTranslation().minus(currentPose.getTranslation()).getAngle(),
              currentPose.getRotation());
    }

    List<PathPoint> path =
        new ArrayList<PathPoint>() {
          {
            add(currentPathPoint);
            add(new PathPoint(pose.getTranslation(), heading, pose.getRotation()));
          }
        };

    traj = PathPlanner.generatePath(constraints, path);
    if (ally == Alliance.Red) {
      drive.swerveDrive.postTrajectory(
          PathPlannerTrajectory.transformTrajectoryForAlliance(traj, ally));
    } else {
      drive.swerveDrive.postTrajectory(traj);
    }
    // position, heading(direction of travel), holonomic rotation

    ppSwerveCommand =
        new PPSwerveControllerCommand(
            traj,
            drive::getPose, // Pose supplier
            Auton.xAutoPID.createPIDController(),
            Auton.yAutoPID.createPIDController(),
            Auton.angleAutoPID.createPIDController(),
            // Rotation controller.
            drive::setChassisSpeeds, // Module states consumer
            true,
            drive // Requires this drive subsystem
            );
  }

  public GoToPose(Pose2d pose, PathConstraints constraints, SwerveSubsystem drive) {
    this(
        pose,
        pose.getTranslation().minus(drive.getPose().getTranslation()).getAngle(),
        constraints,
        drive);
  }

  public PPSwerveControllerCommand getCommand() {
    return ppSwerveCommand;
  }
}
