// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Limelight extends SubsystemBase {
  SwerveSubsystem drivebase;
  Alliance alliance;
  /** Creates a new Limelight. */
  public Limelight(SwerveSubsystem drivebase) {
    this.drivebase = drivebase;
  }

  @Override
  public void periodic() {
    LimelightHelpers.Results result =
        LimelightHelpers.getLatestResults("limelight").targetingResults;
    if (!(result.botpose[0] == 0 && result.botpose[1] == 0)) {
      if (alliance == Alliance.Blue) {
        // double[] botpose = LimelightHelpers.getBotPose_wpiBlue("limelight");
        drivebase.addVisionMeasurement(
            LimelightHelpers.toPose2D(result.botpose_wpiblue),
            Timer.getFPGATimestamp() - (result.latency_capture / 1000.0) - (result.latency_pipeline / 1000.0),
            true,
            1.0);
      } else if (alliance == Alliance.Red) {
        // double[] botpose = LimelightHelpers.getBotPose_wpiRed("limelight");
        drivebase.addVisionMeasurement(
            LimelightHelpers.toPose2D(result.botpose_wpired),
            Timer.getFPGATimestamp() - (result.latency_capture / 1000.0) - (result.latency_pipeline / 1000.0),
            true,
            1.0);
      }
    }
  }

  public void setAlliance(Alliance alliance) {
    this.alliance = alliance;
  }
}
