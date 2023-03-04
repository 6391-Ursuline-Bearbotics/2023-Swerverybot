package frc.robot.commands.swervedrive2.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.ArrayList;
import java.util.List;

public class GoToScoring {
  private final SwerveSubsystem drive;
  private final SCORING_SIDE selectedPosition;
  private final int column;

  public enum POSITION {
    LEFT,
    MIDDLE,
    RIGHT
  }

  public enum SCORING_SIDE {
    BARRIER,
    MIDDLE,
    BUMP
  }

  public GoToScoring(SwerveSubsystem drive, SCORING_SIDE selectedPosition, int column) {
    this.drive = drive;
    this.selectedPosition = selectedPosition;
    this.column = column;
  }

  public Command getCommand() {
    Command command;
    Pose2d currentPose = drive.getPose();
    if (DriverStation.getAlliance() == Alliance.Red) {
      currentPose = new Pose2d(
            currentPose.getX(),
            8.02 - currentPose.getY(),
            currentPose.getRotation().times(-1));
    }
    if (Auton.loadingArea.isPoseWithinLoadingArea(drive.getPose())) {
      // If within Loadin area navigate through selected corridor
      GoToPathPoints goToPathPoints;
      switch (selectedPosition) {
        case BARRIER:
          List<PathPoint> pointsBarrier =
              new ArrayList<PathPoint>() {
                {
                  add(Auton.stationWaypointOut);
                  addAll(Auton.barrierCorridorPPIn);
                  addAll(Auton.scoringPP.get(column - 1));
                }
              };
          goToPathPoints =
              new GoToPathPoints(
                  pointsBarrier,
                  Auton.stationWaypoint,
                  new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
                  drive);
          command = goToPathPoints.getCommand();
          break;
        case BUMP:
          List<PathPoint> pointsBump =
              new ArrayList<PathPoint>() {
                {
                  add(Auton.stationWaypointOut);
                  addAll(Auton.bumpCorridorPPIn);
                  addAll(Auton.scoringPP.get(column - 1));
                }
              };
          goToPathPoints =
              new GoToPathPoints(
                  pointsBump,
                  Auton.stationWaypoint,
                  new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
                  drive);
          command = goToPathPoints.getCommand();
          break;
        case MIDDLE:
          List<PathPoint> pointsMid =
              new ArrayList<PathPoint>() {
                {
                  add(Auton.stationWaypointOut);
                  addAll(Auton.midCorridorPPIn);
                  addAll(Auton.scoringPP.get(column - 1));
                }
              };
          goToPathPoints =
              new GoToPathPoints(
                  pointsMid,
                  Auton.stationWaypoint,
                  new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
                  drive);
          command = goToPathPoints.getCommand();
          break;
        default:
          throw new IllegalArgumentException("Loading station enum not supported.");
      }
    } else if (Auton.scoreArea.isPoseWithinArea(drive.getPose())) {
      List<PathPoint> pointsScoring =
          new ArrayList<PathPoint>() {
            {
              addAll(Auton.scoringPP.get(column - 1));
            }
          };
      GoToPathPoints goToPathPoints =
          new GoToPathPoints(
              pointsScoring,
              Auton.leadingPoses.get(column - 1),
              new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
              drive);
      command = goToPathPoints.getCommand();
    } else {
      // If within Loadin area navigate through selected corridor
      GoToPathPoints goToPathPoints;
      switch (selectedPosition) {
        case BARRIER:
          List<PathPoint> pointsBarrier =
              new ArrayList<PathPoint>() {
                {
                  addAll(Auton.barrierCorridorPPIn);
                  addAll(Auton.scoringPP.get(column - 1));
                }
              };
          goToPathPoints =
              new GoToPathPoints(
                  pointsBarrier,
                  Auton.barrierCorridor.get(1),
                  new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
                  drive);
          command = goToPathPoints.getCommand();
          break;
        case BUMP:
          List<PathPoint> pointsBump =
              new ArrayList<PathPoint>() {
                {
                  addAll(Auton.bumpCorridorPPIn);
                  addAll(Auton.scoringPP.get(column - 1));
                }
              };
          goToPathPoints =
              new GoToPathPoints(
                  pointsBump,
                  Auton.bumpCorridor.get(1),
                  new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
                  drive);
          command = goToPathPoints.getCommand();
          break;
        case MIDDLE:
          List<PathPoint> pointsMid =
              new ArrayList<PathPoint>() {
                {
                  addAll(Auton.midCorridorPPIn);
                  addAll(Auton.scoringPP.get(column - 1));
                }
              };
          goToPathPoints =
              new GoToPathPoints(
                  pointsMid,
                  Auton.midCorridor.get(1),
                  new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
                  drive);
          command = goToPathPoints.getCommand();
          break;
        default:
          throw new IllegalArgumentException("Loading station enum not supported.");
      }
    }
    return command;
  }
}
