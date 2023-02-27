package frc.robot.commands.swervedrive2.auto;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import webblib.util.chargedup.LoadingArea;

public class GoToLoadingZone extends CommandBase {
  private final SwerveSubsystem drive;
  private final LoadingArea loadingArea = Auton.loadingArea;
  private final LOADING_SIDE selectedLoadingSide;
  private Command currentCommand;

  public enum LOADING_SIDE {
    BARRIER,
    RAIL
  }

  public GoToLoadingZone(LOADING_SIDE selectedLoadingSide, SwerveSubsystem drive) {
    this.drive = drive;
    addRequirements(drive);
    this.selectedLoadingSide = selectedLoadingSide;
    currentCommand = Commands.none();
  }

  public Command getCommand() {
    Command command;
    // If we are within the loading area go direct
    if (loadingArea.isPoseWithinLoadingArea(drive.getPose())) {
      GoToPose goToPose;
      switch (selectedLoadingSide) {
        case BARRIER:
          goToPose =
              new GoToPose(
                  loadingArea.getDoubleSubstationBarrier().getPoseMeters(),
                  new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
                  drive);
          command = goToPose.getCommand();
          break;
        case RAIL:
          goToPose =
              new GoToPose(
                  loadingArea.getDoubleSubstationRail().getPoseMeters(),
                  new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
                  drive);
          command = goToPose.getCommand();
          break;
        default:
          throw new IllegalArgumentException("Loading station enum not supported.");
      }
    } else if (Auton.scoreArea.isPoseWithinArea(drive.getPose())) {
      // If we are within scoring area find the left / right exit
      GoToPathPoints goToPathPoints;
      switch (selectedLoadingSide) {
        case BARRIER:
          List<PathPoint> pointsBarrier = new ArrayList<PathPoint>() {
            {
              addAll(Auton.barrierCorridorPPOut);
              add(Auton.stationWaypointIn);
            }
          };
          goToPathPoints =
              new GoToPathPoints(
                  pointsBarrier,
                  Auton.barrierCorridor.get(0),
                  new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
                  drive);
          command = goToPathPoints.getCommand();
          break;
        case RAIL:
          List<PathPoint> pointsRail = new ArrayList<PathPoint>() {
            {
              addAll(Auton.bumpCorridorPPOut);
              add(Auton.stationWaypointIn);
            }
          };
          goToPathPoints =
              new GoToPathPoints(
                  pointsRail,
                  Auton.bumpCorridor.get(0),
                  new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
                  drive);
          command = goToPathPoints.getCommand();
          break;
        default:
          throw new IllegalArgumentException("Loading station enum not supported.");
      }
    } else {
      command = Commands.none();
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
