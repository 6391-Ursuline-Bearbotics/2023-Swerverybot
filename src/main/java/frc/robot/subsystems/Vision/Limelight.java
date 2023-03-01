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
    if (alliance == Alliance.Blue) {
      double[] botpose = LimelightHelpers.getBotPose_wpiBlue("limelight");
      if (botpose.length != 0) {
        drivebase.addVisionMeasurement(
            LimelightHelpers.toPose2D(botpose),
            Timer.getFPGATimestamp() - (botpose[6] / 1000.0),
            true,
            1.0);
      }
    } else if (alliance == Alliance.Red) {
      double[] botpose = LimelightHelpers.getBotPose_wpiRed("limelight");
      if (botpose.length != 0) {
        drivebase.addVisionMeasurement(
            LimelightHelpers.toPose2D(botpose),
            Timer.getFPGATimestamp() - (botpose[6] / 1000.0),
            true,
            1.0);
      }
    }
  }

  public void setAlliance(Alliance alliance) {
    this.alliance = alliance;
  }
}
