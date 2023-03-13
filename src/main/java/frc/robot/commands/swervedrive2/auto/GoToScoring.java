package frc.robot.commands.swervedrive2.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
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
  private final List<EventMarker> markers = new ArrayList<>();
  Command command;
  GoToPathPoints goToPathPoints;

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

  public GoToScoring(
      SwerveSubsystem drive,
      SCORING_SIDE selectedPosition,
      int column,
      String event,
      AutoMap autoMap) {

    Pose2d currentPose = drive.getPose();
    if (DriverStation.getAlliance() == Alliance.Red) {
      currentPose =
          new Pose2d(
              currentPose.getX(), 8.02 - currentPose.getY(), currentPose.getRotation().times(-1));
    }
    if (Auton.loadingArea.isPoseWithinLoadingArea(currentPose)) {
      // If within Loadin area navigate through selected corridor
      switch (selectedPosition) {
        case BARRIER:
          List<PathPoint> pointsBarrier =
              new ArrayList<PathPoint>() {
                {
                  add(Auton.stationWaypointOut);
                  add(Auton.barrierCorridorOuter);
                  add(Auton.barrierCorridorInner);
                  addAll(Auton.scoringPP.get(column - 1));
                }
              };
          markers.add(
              new EventMarker(
                  new ArrayList<String>() {
                    {
                      add(event);
                    }
                  },
                  4.0));
          goToPathPoints =
              new GoToPathPoints(
                  pointsBarrier,
                  Auton.stationWaypoint,
                  new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
                  drive,
                  markers,
                  autoMap);
          command = goToPathPoints.getCommand();
          break;
        case BUMP:
          List<PathPoint> pointsBump =
              new ArrayList<PathPoint>() {
                {
                  add(Auton.stationWaypointOut);
                  add(Auton.bumpCorridorOuter);
                  add(Auton.bumpCorridorInner);
                  addAll(Auton.scoringPP.get(column - 1));
                }
              };
          markers.add(
              new EventMarker(
                  new ArrayList<String>() {
                    {
                      add(event);
                    }
                  },
                  4.0));
          goToPathPoints =
              new GoToPathPoints(
                  pointsBump,
                  Auton.stationWaypoint,
                  new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
                  drive,
                  markers,
                  autoMap);
          command = goToPathPoints.getCommand();
          break;
        case MIDDLE:
          List<PathPoint> pointsMid =
              new ArrayList<PathPoint>() {
                {
                  add(Auton.stationWaypointOut);
                  add(Auton.midCorridorOuter);
                  add(Auton.midCorridorInner);
                  addAll(Auton.scoringPP.get(column - 1));
                }
              };
          markers.add(
              new EventMarker(
                  new ArrayList<String>() {
                    {
                      add(event);
                    }
                  },
                  4.0));
          goToPathPoints =
              new GoToPathPoints(
                  pointsMid,
                  Auton.stationWaypoint,
                  new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
                  drive,
                  markers,
                  autoMap);
          command = goToPathPoints.getCommand();
          break;
        default:
          throw new IllegalArgumentException("Loading station enum not supported.");
      }
    } else if (Auton.scoreArea.isPoseWithinArea(currentPose)) {
      // Within the scoring area so go direct to the leading / scoring spot
      List<PathPoint> pointsScoring =
          new ArrayList<PathPoint>() {
            {
              addAll(Auton.scoringPP.get(column - 1));
            }
          };
      markers.add(
          new EventMarker(
              new ArrayList<String>() {
                {
                  add(event);
                }
              },
              0.5));
      goToPathPoints =
          new GoToPathPoints(
              pointsScoring,
              Auton.leadingPoses.get(column - 1),
              new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
              drive,
              markers,
              autoMap);
      command = goToPathPoints.getCommand();
    } else if (Auton.chargeArea.isPoseWithinArea(currentPose)) {
      // Within the charge area so go to only last corridor point then scoring
      switch (selectedPosition) {
        case BARRIER:
          List<PathPoint> chargeBarrier =
              new ArrayList<PathPoint>() {
                {
                  add(Auton.barrierCorridorInner);
                  addAll(Auton.scoringPP.get(column - 1));
                }
              };
          markers.add(
              new EventMarker(
                  new ArrayList<String>() {
                    {
                      add(event);
                    }
                  },
                  2.0));
          goToPathPoints =
              new GoToPathPoints(
                  chargeBarrier,
                  Auton.barrierCorridor.get(0),
                  new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
                  drive,
                  markers,
                  autoMap);
          command = goToPathPoints.getCommand();
          break;
        case BUMP:
          List<PathPoint> chargeBump =
              new ArrayList<PathPoint>() {
                {
                  add(Auton.bumpCorridorInner);
                  addAll(Auton.scoringPP.get(column - 1));
                }
              };
          markers.add(
              new EventMarker(
                  new ArrayList<String>() {
                    {
                      add(event);
                    }
                  },
                  2.0));
          goToPathPoints =
              new GoToPathPoints(
                  chargeBump,
                  Auton.bumpCorridor.get(0),
                  new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
                  drive,
                  markers,
                  autoMap);
          command = goToPathPoints.getCommand();
          break;
        case MIDDLE:
          List<PathPoint> chargeMid =
              new ArrayList<PathPoint>() {
                {
                  add(Auton.midCorridorInner);
                  addAll(Auton.scoringPP.get(column - 1));
                }
              };
          markers.add(
              new EventMarker(
                  new ArrayList<String>() {
                    {
                      add(event);
                    }
                  },
                  2.0));
          goToPathPoints =
              new GoToPathPoints(
                  chargeMid,
                  Auton.midCorridor.get(0),
                  new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
                  drive,
                  markers,
                  autoMap);
          command = goToPathPoints.getCommand();
          break;
        default:
          throw new IllegalArgumentException("Loading station enum not supported.");
      }
    } else {
      // This must be in the "middle" of the field
      switch (selectedPosition) {
        case BARRIER:
          List<PathPoint> pointsBarrier =
              new ArrayList<PathPoint>() {
                {
                  add(Auton.barrierCorridorOuter);
                  add(Auton.barrierCorridorInner);
                  addAll(Auton.scoringPP.get(column - 1));
                }
              };
          markers.add(
              new EventMarker(
                  new ArrayList<String>() {
                    {
                      add(event);
                    }
                  },
                  3.0));
          goToPathPoints =
              new GoToPathPoints(
                  pointsBarrier,
                  Auton.barrierCorridor.get(1),
                  new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
                  drive,
                  markers,
                  autoMap);
          command = goToPathPoints.getCommand();
          break;
        case BUMP:
          List<PathPoint> pointsBump =
              new ArrayList<PathPoint>() {
                {
                  add(Auton.bumpCorridorOuter);
                  add(Auton.bumpCorridorInner);
                  addAll(Auton.scoringPP.get(column - 1));
                }
              };
          markers.add(
              new EventMarker(
                  new ArrayList<String>() {
                    {
                      add(event);
                    }
                  },
                  3.0));
          goToPathPoints =
              new GoToPathPoints(
                  pointsBump,
                  Auton.bumpCorridor.get(1),
                  new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
                  drive,
                  markers,
                  autoMap);
          command = goToPathPoints.getCommand();
          break;
        case MIDDLE:
          List<PathPoint> pointsMid =
              new ArrayList<PathPoint>() {
                {
                  add(Auton.midCorridorOuter);
                  add(Auton.midCorridorInner);
                  addAll(Auton.scoringPP.get(column - 1));
                }
              };
          markers.add(
              new EventMarker(
                  new ArrayList<String>() {
                    {
                      add(event);
                    }
                  },
                  3.0));
          goToPathPoints =
              new GoToPathPoints(
                  pointsMid,
                  Auton.midCorridor.get(1),
                  new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
                  drive,
                  markers,
                  autoMap);
          command = goToPathPoints.getCommand();
          break;
        default:
          throw new IllegalArgumentException("Loading station enum not supported.");
      }
    }
  }

  public Command getCommand() {
    return command;
  }
}
