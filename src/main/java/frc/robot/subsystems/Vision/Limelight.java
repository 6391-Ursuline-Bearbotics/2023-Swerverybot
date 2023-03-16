// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Limelight extends SubsystemBase {
  SwerveSubsystem drivebase;
  Alliance alliance;
  private Boolean enable = true;
  private int fieldError = 0;
  private int distanceError = 0;

  /** Creates a new Limelight. */
  public Limelight(SwerveSubsystem drivebase) {
    this.drivebase = drivebase;
    SmartDashboard.putNumber("Field Error", fieldError);
    SmartDashboard.putNumber("Limelight Error", distanceError);
  }

  @Override
  public void periodic() {
    if (enable) {
      LimelightHelpers.Results result =
          LimelightHelpers.getLatestResults("limelight").targetingResults;
      if (!(result.botpose[0] == 0 && result.botpose[1] == 0)) {
        if (Auton.field.isPoseWithinArea(LimelightHelpers.toPose2D(result.botpose))) {
            if (alliance == Alliance.Blue) {
              Pose2d botpose = LimelightHelpers.toPose2D(result.botpose_wpiblue);
              if (drivebase.getPose().getTranslation().getDistance(botpose.getTranslation()) < 1.0) {
                drivebase.addVisionMeasurement(
                    botpose,
                    Timer.getFPGATimestamp()
                        - (result.latency_capture / 1000.0)
                        - (result.latency_pipeline / 1000.0),
                    true,
                    1.0);
              } else {
                distanceError++;
                SmartDashboard.putNumber("Limelight Error", distanceError);
              }
            } else if (alliance == Alliance.Red) {
              Pose2d botpose = LimelightHelpers.toPose2D(result.botpose_wpired);
              if (drivebase.getPose().getTranslation().getDistance(botpose.getTranslation()) < 1.0) {
                drivebase.addVisionMeasurement(
                    botpose,
                    Timer.getFPGATimestamp()
                        - (result.latency_capture / 1000.0)
                        - (result.latency_pipeline / 1000.0),
                    true,
                    1.0);
              } else {
                distanceError++;
                SmartDashboard.putNumber("Limelight Error", distanceError);
              }
            }
        } else {
          fieldError++;
          SmartDashboard.putNumber("Field Error", fieldError);
        }
      }
    }
  }

  public void setAlliance(Alliance alliance) {
    this.alliance = alliance;
  }

  public void useLimelight(boolean enable) {
    this.enable = enable;
  }
}
