// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.opencv.core.Mat;

import swervelib.parser.PIDFConfig;
import webblib.util.Gains;
import webblib.util.HolonomicPose2d;
import webblib.util.RectanglePoseArea;
import webblib.util.chargedup.LoadingArea;
import webblib.util.chargedup.ScoringArea;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final double CHASSIS_MASS = ROBOT_MASS;
  public static final Translation3d CHASSIS_CG = new Translation3d(0, 0, Units.inchesToMeters(8));
  public static final double LOOP_TIME = 0.02; // s, 20ms + 110ms sprk max velocity lag
  public static final double STOP_SECONDS = 5.0;

  public static final class OIConstants {
    public static final int driverID = 0, buttonsID = 1, operatorID = 2;

    public static double xyDeadband = 0.02;
    public static double radDeadband = 0.05;
    public static double radLimiter = 0.5;
  }

  public static final class Auton {
    public static final PIDFConfig xAutoPID = new PIDFConfig(4.0, 0, 0);
    public static final PIDFConfig yAutoPID = new PIDFConfig(4.0, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(2.2, 0, 0.0);

    public static final double maxSpeedMPS = 3; // 4?
    public static final double maxAccelerationMPS = 2;
    public static final double midSpeedMPS = 2;
    public static final double lowSpeedMPS = 1;
    public static final double balanceScale = 1.0, balanceScalePow = 1.0;

    public static final LoadingArea loadingArea =
        new LoadingArea(
            new RectanglePoseArea(new Translation2d(9.91, 6.82), new Translation2d(16.24, 7.97)),
            new RectanglePoseArea(new Translation2d(13.24, 5.66), new Translation2d(16.51, 7.97)),
            new HolonomicPose2d(new Pose2d(15.75, 7.34, new Rotation2d()), new Rotation2d()),
            new HolonomicPose2d(new Pose2d(15.75, 6.00, new Rotation2d()), new Rotation2d()));

    public static final List<ScoringArea> scoreAreaList =
        new ArrayList<>() {
          {
            add(
                new ScoringArea(
                    new RectanglePoseArea(
                        new Translation2d(1.23, 3.53), new Translation2d(2.86, 5.33)),
                    // diagonal y's should not overlap
                    new HolonomicPose2d(new Pose2d(1.62, 4.95, new Rotation2d()), new Rotation2d()),
                    new HolonomicPose2d(new Pose2d(1.62, 4.40, new Rotation2d()), new Rotation2d()),
                    new HolonomicPose2d(
                        new Pose2d(1.62, 3.84, new Rotation2d()), new Rotation2d())));
            add(
                new ScoringArea(
                    new RectanglePoseArea(
                        new Translation2d(1.23, 1.90), new Translation2d(2.92, 3.52)),
                    new HolonomicPose2d(new Pose2d(1.62, 3.30, new Rotation2d()), new Rotation2d()),
                    new HolonomicPose2d(new Pose2d(1.62, 2.72, new Rotation2d()), new Rotation2d()),
                    new HolonomicPose2d(
                        new Pose2d(1.62, 2.19, new Rotation2d()), new Rotation2d())));
            add(
                new ScoringArea(
                    new RectanglePoseArea(
                        new Translation2d(1.23, 0.0), new Translation2d(2.89, 1.89)),
                    new HolonomicPose2d(new Pose2d(1.62, 1.61, new Rotation2d()), new Rotation2d()),
                    new HolonomicPose2d(new Pose2d(1.62, 1.03, new Rotation2d()), new Rotation2d()),
                    new HolonomicPose2d(
                        new Pose2d(1.62, 0.55, new Rotation2d()), new Rotation2d())));
          }
        };
    public static final RectanglePoseArea scoreArea = new RectanglePoseArea(new Translation2d(1.23, 0.0), new Translation2d(2.92, 5.33));
    public static final Translation3d cameraTranslation = new Translation3d(0.5, 0.0, 0.5);
    public static final Rotation3d cameraRotation = new Rotation3d(0, 0, 0);

    public static final List<Pose2d> barrierCorridor =
      new ArrayList<>() {
        {
          add(new Pose2d(2.53, 4.46, new Rotation2d(Math.PI)));
          add(new Pose2d(5.3, 4.46, new Rotation2d(Math.PI)));
        }
      };

    public static final List<PathPoint> barrierCorridorPPOut =
      new ArrayList<>() {
        {
          add(new PathPoint(barrierCorridor.get(1).getTranslation(), new Rotation2d(), barrierCorridor.get(1).getRotation()));
          add(new PathPoint(barrierCorridor.get(2).getTranslation(), new Rotation2d(), barrierCorridor.get(2).getRotation()));
        }
      };

    public static final List<PathPoint> barrierCorridorPPIn =
      new ArrayList<>() {
        {
          add(new PathPoint(barrierCorridor.get(2).getTranslation(), new Rotation2d(Math.PI), barrierCorridor.get(2).getRotation()));
          add(new PathPoint(barrierCorridor.get(1).getTranslation(), new Rotation2d(Math.PI), barrierCorridor.get(1).getRotation()));
        }
      };

    public static final List<Pose2d> bumpCorridor =
      new ArrayList<>() {
        {
          add(new Pose2d(2.53, 1.08, new Rotation2d(Math.PI)));
          add(new Pose2d(5.3, 1.08, new Rotation2d(Math.PI)));
        }
      };

    public static final List<PathPoint> bumpCorridorPPOut =
      new ArrayList<>() {
        {
          add(new PathPoint(barrierCorridor.get(1).getTranslation(), new Rotation2d(), barrierCorridor.get(1).getRotation()));
          add(new PathPoint(barrierCorridor.get(2).getTranslation(), new Rotation2d(), barrierCorridor.get(2).getRotation()));
        }
      };

    public static final List<PathPoint> bumpCorridorPPIn =
      new ArrayList<>() {
        {
          add(new PathPoint(barrierCorridor.get(2).getTranslation(), new Rotation2d(Math.PI), barrierCorridor.get(2).getRotation()));
          add(new PathPoint(barrierCorridor.get(1).getTranslation(), new Rotation2d(Math.PI), barrierCorridor.get(1).getRotation()));
        }
      };

    public static final Pose2d stationWaypoint = new Pose2d(13.22, 6.77, new Rotation2d());
    public static final PathPoint stationWaypointIn = new PathPoint(stationWaypoint.getTranslation(), new Rotation2d(), stationWaypoint.getRotation());
    public static final PathPoint stationWaypointOut = new PathPoint(stationWaypoint.getTranslation(), new Rotation2d(Math.PI), stationWaypoint.getRotation());
  }

  public static final class LEDConstants {
    public static final int PWMPort = 9, length = 10;
  }
}
