package frc.robot.commands.swervedrive2.auto;

import com.pathplanner.lib.PathConstraints;
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
import webblib.util.chargedup.LoadingArea;

public class GoToLoadingZone extends CommandBase {
  private final SwerveSubsystem drive;
  private final LoadingArea loadingArea = Auton.loadingArea;
  private final LOADING_SIDE selectedLoadingSide;
  private Command currentCommand;
  private Alliance ally;

  public enum LOADING_SIDE {
    BARRIER,
    RAIL
  }

  public GoToLoadingZone(LOADING_SIDE selectedLoadingSide, SwerveSubsystem drive, Alliance ally) {
    this.drive = drive;
    this.ally = ally;
    addRequirements(drive);
    this.selectedLoadingSide = selectedLoadingSide;
    currentCommand = Commands.none();
  }

  public Command getCommand() {
    Command command;
    // If we are within the loading area go direct
    if (loadingArea.isPoseWithinLoadingArea(drive.getPose())) {
      // In loading area so just run the PathGroup that happens after this.
      command = Commands.none();
    } else if (Auton.scoreArea.isPoseWithinArea(drive.getPose())) {
      // If we are within scoring area find the left / right corridor
      GoToPathPoints goToPathPoints;
      List<PathPoint> barrierCorridor;
      List<PathPoint> railCorridor;
      Pose2d barrierFirst;
      Pose2d railFirst;
      if (ally == Alliance.Blue) {
        barrierCorridor = Auton.bumpCorridorPPOut;
        railCorridor = Auton.barrierCorridorPPOut;
        barrierFirst = Auton.bumpCorridor.get(0);
        railFirst = Auton.barrierCorridor.get(0);
      } else {
        barrierCorridor = Auton.barrierCorridorPPOut;
        railCorridor = Auton.bumpCorridorPPOut;
        barrierFirst = Auton.barrierCorridor.get(0);
        railFirst = Auton.bumpCorridor.get(0);
      }
      switch (selectedLoadingSide) {
        case BARRIER:
          List<PathPoint> pointsBarrier =
              new ArrayList<PathPoint>() {
                {
                  addAll(barrierCorridor);
                  add(Auton.stationWaypointIn);
                }
              };
          goToPathPoints =
              new GoToPathPoints(
                  pointsBarrier,
                  barrierFirst,
                  new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
                  drive);
          command = goToPathPoints.getCommand();
          break;
        case RAIL:
          List<PathPoint> pointsRail =
              new ArrayList<PathPoint>() {
                {
                  addAll(railCorridor);
                  add(Auton.stationWaypointIn);
                }
              };
          goToPathPoints =
              new GoToPathPoints(
                  pointsRail,
                  railFirst,
                  new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
                  drive);
          command = goToPathPoints.getCommand();
          break;
        default:
          throw new IllegalArgumentException("Loading station enum not supported.");
      }
    } else {
      // We are in the middle of the field so go to the stationWaypointIn
      GoToPathPoints goToPathPoints;
      List<PathPoint> pointsRail =
          new ArrayList<PathPoint>() {
            {
              add(Auton.stationWaypointIn);
            }
          };
      goToPathPoints =
          new GoToPathPoints(
              pointsRail,
              Auton.stationWaypoint,
              new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
              drive);
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
