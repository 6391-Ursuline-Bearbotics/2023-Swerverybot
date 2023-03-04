package frc.robot.commands.swervedrive2.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
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
  private List<EventMarker> markers;

  public GoToPathPoints(
      List<PathPoint> points,
      Pose2d firstPose,
      PathConstraints constraints,
      SwerveSubsystem drive,
      List<EventMarker> markers) {
    PathPoint currentPathPoint;
    currentPose = drive.getPose();
    if (DriverStation.getAlliance() == Alliance.Red) {
        firstPose = new Pose2d(
            firstPose.getX(),
            8.02 - firstPose.getY(),
            firstPose.getRotation().times(-1));
        currentPose = new Pose2d(
            currentPose.getX(),
            8.02 - currentPose.getY(),
            currentPose.getRotation().times(-1));
    }
    if (Math.hypot(
            drive.getFieldVelocity().vxMetersPerSecond, drive.getFieldVelocity().vyMetersPerSecond)
        > 0.2) {
      currentPathPoint =
          PathPoint.fromCurrentHolonomicState(currentPose, drive.getFieldVelocity());
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

  public PPSwerveControllerCommand getCommand() {
    return ppSwerveCommand;
  }
}
