package frc.robot.commands.swervedrive2.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.HashMap;
import java.util.Map;

public class TeleopBuilder {
  private SwerveAutoBuilder teleopBuilder;
  private SwerveSubsystem drivebase;
  public Map<String, Command> pathMap = new HashMap<>();

  public TeleopBuilder(SwerveSubsystem drivebase, HashMap<String, Command> eventMap) {
    this.drivebase = drivebase;

    teleopBuilder =
        new SwerveAutoBuilder(
            drivebase::getPose, // Functional interface to feed supplier
            (Pose2d) -> {},
            // Position controllers
            new PIDConstants(Auton.xAutoPID.p, Auton.xAutoPID.i, Auton.xAutoPID.d),
            new PIDConstants(Auton.xAutoPID.p, Auton.xAutoPID.i, Auton.xAutoPID.d),
            drivebase::setChassisSpeeds,
            eventMap,
            false,
            drivebase);
  }

  public Command getSwerveCommand(String path) {
    return pathMap.get(path);
  }

  public void loadPath(String filename) {
    pathMap.put(
        filename,
        teleopBuilder.fullAuto(
            PathPlanner.loadPathGroup(
                filename, new PathConstraints(Auton.maxSpeedMPS, Auton.maxAccelerationMPS))));
  }
}
