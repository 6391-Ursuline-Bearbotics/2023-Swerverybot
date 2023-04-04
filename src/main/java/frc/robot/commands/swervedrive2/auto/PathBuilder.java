package frc.robot.commands.swervedrive2.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.HashMap;
import java.util.Map;

public class PathBuilder {
  private SwerveAutoBuilder autoBuilder;
  private SwerveSubsystem drivebase;
  public Map<String, Command> pathMap = new HashMap<>();

  public PathBuilder(SwerveSubsystem drivebase, HashMap<String, Command> eventMap) {
    this.drivebase = drivebase;

    autoBuilder =
        new SwerveAutoBuilder(
            drivebase::getPose, // Functional interface to feed supplier
            drivebase::resetOdometry,
            // Position controllers
            new PIDConstants(Auton.xAutoPID.p, Auton.xAutoPID.i, Auton.xAutoPID.d),
            new PIDConstants(Auton.xAutoPID.p, Auton.xAutoPID.i, Auton.xAutoPID.d),
            drivebase::setChassisSpeeds,
            eventMap,
            true,
            drivebase);
  }

  public Command getSwerveCommand(String path) {
    return pathMap.get(path);
  }

  public void loadAllPaths() {
    File folder = new File(Filesystem.getDeployDirectory(), "pathplanner/");
    File[] listOfFiles = folder.listFiles();

    for (int i = 0; i < listOfFiles.length; i++) {
      String filename = listOfFiles[i].getName();
      filename = filename.substring(0, filename.length() - 5);
      pathMap.put(
          filename,
          autoBuilder.fullAuto(
              PathPlanner.loadPathGroup(
                  filename,
                  new PathConstraints(Auton.autoMaxSpeedMPS, Auton.autoMaxAccelerationMPS))));
    }
  }
}
