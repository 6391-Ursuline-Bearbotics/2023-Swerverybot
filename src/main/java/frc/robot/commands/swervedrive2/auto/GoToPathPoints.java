package frc.robot.commands.swervedrive2.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.ArrayList;
import java.util.List;

public class GoToPathPoints {
  private PPSwerveControllerCommand ppSwerveCommand;
  private PathPlannerTrajectory traj;
  private Pose2d currentPose;
  private FollowPathWithEvents eventCommand;

  public GoToPathPoints(
      List<PathPoint> points,
      Pose2d firstPose,
      PathConstraints constraints,
      SwerveSubsystem drive,
      List<EventMarker> markers,
      AutoMap eventMap) {
    PathPoint currentPathPoint;
    currentPose = drive.getPose();
    Alliance ally = DriverStation.getAlliance();
    if (ally == Alliance.Red) {
      // firstPose = flipPose(firstPose);
      currentPose = flipPose(currentPose);
    }
    if (Math.hypot(
            drive.getFieldVelocity().vxMetersPerSecond, drive.getFieldVelocity().vyMetersPerSecond)
        > 0.2) {
      currentPathPoint = PathPoint.fromCurrentHolonomicState(currentPose, drive.getFieldVelocity());
    } else {
      currentPathPoint =
          new PathPoint(
              currentPose.getTranslation(),
              firstPose.getTranslation().minus(currentPose.getTranslation()).getAngle(),
              currentPose.getRotation());
    }

    List<PathPoint> path =
        new ArrayList<PathPoint>() {
          {
            add(currentPathPoint);
            addAll(points);
          }
        };

    traj = PathPlanner.generatePath(constraints, path, markers);
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
    eventCommand = new FollowPathWithEvents(ppSwerveCommand, markers, eventMap.getMap());
  }

  public FollowPathWithEvents getCommand() {
    return eventCommand;
  }

  private Pose2d flipPose(Pose2d pose) {
    return new Pose2d(pose.getX(), 8.02 - pose.getY(), pose.getRotation().times(-1));
  }
}
