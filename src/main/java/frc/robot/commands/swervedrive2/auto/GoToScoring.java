package frc.robot.commands.swervedrive2.auto;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

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
    if (Auton.loadingArea.isPoseWithinLoadingArea(drive.getPose())) {
      command = Commands.none();
    } else if (Auton.scoreArea.isPoseWithinArea(drive.getPose())) {
      GoToPose goToPose =
          new GoToPose(
              Auton.scoringPoses.get(column),
              Auton.scoringPoses.get(column).getRotation(),
              new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS),
              drive);
      command = goToPose.getCommand();
    } else {
      command = Commands.none();
    }
    return command;
  }
}
