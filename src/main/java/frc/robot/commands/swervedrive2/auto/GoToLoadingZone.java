package frc.robot.commands.swervedrive2.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.ArrayList;
import java.util.List;
import webblib.util.RectanglePoseArea;

public class GoToLoadingZone extends CommandBase {
  private final SwerveSubsystem drive;
  private final RectanglePoseArea loadingArea = Auton.loadingArea;
  private final LOADING_SIDE selectedLoadingSide;
  private Command currentCommand;
  private Alliance ally;
  private List<PathPoint> corridor;
  private Pose2d corridorFirst;
  private final List<EventMarker> markers = new ArrayList<>();
  private final AutoMap autoMap;
  private final String piece;

  public enum LOADING_SIDE {
    BARRIER,
    RAIL
  }

  public GoToLoadingZone(
      boolean left, SwerveSubsystem drive, Alliance ally, AutoMap autoMap, String piece) {
    this.drive = drive;
    this.ally = ally;
    addRequirements(drive);
    this.selectedLoadingSide = getLoadingSide(left);
    currentCommand = Commands.none();
    this.autoMap = autoMap;
    this.piece = piece;
  }

  private LOADING_SIDE getLoadingSide(Boolean left) {
    if (ally == Alliance.Blue) {
      if (left) {
        corridor = Auton.barrierCorridorPPOut;
        corridorFirst = Auton.barrierCorridor.get(0);
        return LOADING_SIDE.RAIL;
      } else {
        corridor = Auton.bumpCorridorPPOut;
        corridorFirst = Auton.bumpCorridor.get(0);
        return LOADING_SIDE.BARRIER;
      }
    } else {
      if (left) {
        corridor = Auton.bumpCorridorPPOut;
        corridorFirst = Auton.bumpCorridor.get(0);
        return LOADING_SIDE.BARRIER;
      } else {
        corridor = Auton.barrierCorridorPPOut;
        corridorFirst = Auton.barrierCorridor.get(0);
        return LOADING_SIDE.RAIL;
      }
    }
  }

  public Command getCommand() {
    Command command;
    Pose2d currentPose = drive.getPose();
    markers.add(
        new EventMarker(
            new ArrayList<String>() {
              {
                add("ArmStow");
              }
            },
            0.0));
    if (ally == Alliance.Red) {
      currentPose =
          new Pose2d(
              currentPose.getX(), 8.02 - currentPose.getY(), currentPose.getRotation().times(-1));
    }
    // If we are within the loading area go direct
    if (loadingArea.isPoseWithinArea(currentPose)) {
      // In loading area so just run the PathGroup that happens after this.
      GoToPathPoints goToPathPoints;
      List<PathPoint> points;
      Pose2d leadPoint;
      switch (selectedLoadingSide) {
        case BARRIER:
          leadPoint = Auton.loadingLead.get(1);
          points =
              new ArrayList<PathPoint>() {
                {
                  addAll(Auton.loadingBarrier);
                }
              };
          break;
        default:
          leadPoint = Auton.loadingLead.get(0);
          points =
              new ArrayList<PathPoint>() {
                {
                  addAll(Auton.loadingRail);
                }
              };
      }
      markers.add(
          new EventMarker(
              new ArrayList<String>() {
                {
                  add(piece);
                  add("ArmHigh");
                }
              },
              1.0));
      goToPathPoints =
          new GoToPathPoints(
              points,
              leadPoint,
              new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
              drive,
              markers,
              autoMap);
      command = goToPathPoints.getCommand();
    } else if (Auton.scoreArea.isPoseWithinArea(currentPose)) {
      // If we are within scoring area find the left / right corridor
      GoToPathPoints goToPathPoints;
      switch (selectedLoadingSide) {
        case BARRIER:
          List<PathPoint> pointsBarrier =
              new ArrayList<PathPoint>() {
                {
                  addAll(corridor);
                  add(Auton.stationWaypointIn);
                  addAll(Auton.loadingBarrier);
                }
              };
          markers.add(
              new EventMarker(
                  new ArrayList<String>() {
                    {
                      add(piece);
                      add("ArmHigh");
                    }
                  },
                  4.0));
          goToPathPoints =
              new GoToPathPoints(
                  pointsBarrier,
                  corridorFirst,
                  new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
                  drive,
                  markers,
                  autoMap);
          command = goToPathPoints.getCommand();
          break;
        case RAIL:
          List<PathPoint> pointsRail =
              new ArrayList<PathPoint>() {
                {
                  addAll(corridor);
                  add(Auton.stationWaypointIn);
                  addAll(Auton.loadingRail);
                }
              };
          markers.add(
              new EventMarker(
                  new ArrayList<String>() {
                    {
                      add(piece);
                      add("ArmHigh");
                    }
                  },
                  4.0));
          goToPathPoints =
              new GoToPathPoints(
                  pointsRail,
                  corridorFirst,
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
      // We are in the middle of the field so go to the stationWaypointIn
      GoToPathPoints goToPathPoints;
      List<PathPoint> points =
          new ArrayList<PathPoint>() {
            {
              add(Auton.stationWaypointIn);
            }
          };
      if (selectedLoadingSide == LOADING_SIDE.BARRIER) {
        points.addAll(Auton.loadingBarrier);
      } else {
        points.addAll(Auton.loadingRail);
      }
      markers.add(
          new EventMarker(
              new ArrayList<String>() {
                {
                  add(piece);
                  add("ArmHigh");
                }
              },
              2.0));
      goToPathPoints =
          new GoToPathPoints(
              points,
              Auton.stationWaypoint,
              new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
              drive,
              markers,
              autoMap);
      command = goToPathPoints.getCommand();
    }
    return command;
  }

  @Override
  public void initialize() {
    currentCommand = getCommand();
    currentCommand.schedule();
  }

  @Override
  public void execute() {
    if (currentCommand.isFinished()) {
      currentCommand = getCommand();
      currentCommand.schedule();
    }
  }

  @Override
  public void end(boolean interrupted) {
    currentCommand.cancel();
  }
}
