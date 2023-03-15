// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;
import webblib.util.HolonomicPose2d;
import webblib.util.RectanglePoseArea;
import webblib.util.chargedup.LoadingArea;

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
  public static final Matter CHASSIS =
      new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double CHASSIS_MASS = ROBOT_MASS;
  public static final Translation3d CHASSIS_CG = new Translation3d(0, 0, Units.inchesToMeters(8));
  public static final double LOOP_TIME = 0.02; // s, 20ms + 110ms sprk max velocity lag
  public static final double STOP_SECONDS = 5.0;

  public static final class OIConstants {
    public static final int driverID = 0, buttonsID = 1, operatorID = 2;

    public static double xyDeadband = 0.02;
    public static double radDeadband = 0.05;
    public static double radLimiter = 0.4;
    public static double radSlow = 0.15;
  }

  public static final class Auton {
    public static final PIDFConfig xAutoPID = new PIDFConfig(4.0, 0, 0);
    public static final PIDFConfig yAutoPID = new PIDFConfig(4.0, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(2.2, 0, 0.0);

    public static final double maxSpeedMPS = 3; // 4?
    public static final double maxAccelerationMPS = 1.5;
    public static final double midSpeedMPS = 2;
    public static final double lowSpeedMPS = 1;

    public static final double balanceScale = 2.5;
    public static final double balanceScalePow = 1.0;
    public static final double balanceLimitDeg = 2.0;

    public static final LoadingArea loadingArea =
        new LoadingArea(
            new RectanglePoseArea(new Translation2d(9.91, 6.82), new Translation2d(16.24, 7.97)),
            new RectanglePoseArea(new Translation2d(13.24, 5.66), new Translation2d(16.51, 7.97)),
            new HolonomicPose2d(new Pose2d(15.75, 7.34, new Rotation2d()), new Rotation2d()),
            new HolonomicPose2d(new Pose2d(15.75, 6.00, new Rotation2d()), new Rotation2d()));

    private static final double scoringX = 1.95;
    public static final List<Pose2d> scoringPoses =
        new ArrayList<>() {
          {
            add(new Pose2d(scoringX, Units.inchesToMeters(196.19), new Rotation2d(Math.PI)));
            add(new Pose2d(scoringX, Units.inchesToMeters(174.19), new Rotation2d(Math.PI)));
            add(new Pose2d(scoringX, Units.inchesToMeters(152.19), new Rotation2d(Math.PI)));
            add(new Pose2d(scoringX, Units.inchesToMeters(130.19), new Rotation2d(Math.PI)));
            add(new Pose2d(scoringX, Units.inchesToMeters(108.19), new Rotation2d(Math.PI)));
            add(new Pose2d(scoringX, Units.inchesToMeters(86.19), new Rotation2d(Math.PI)));
            add(new Pose2d(scoringX, Units.inchesToMeters(64.19), new Rotation2d(Math.PI)));
            add(new Pose2d(scoringX, Units.inchesToMeters(42.19), new Rotation2d(Math.PI)));
            add(new Pose2d(scoringX, Units.inchesToMeters(20.19), new Rotation2d(Math.PI)));
          }
        };

    private static final double leadingX = 2.2;
    public static final List<Pose2d> leadingPoses =
        new ArrayList<>() {
          {
            add(new Pose2d(leadingX, Units.inchesToMeters(196.19), new Rotation2d(Math.PI)));
            add(new Pose2d(leadingX, Units.inchesToMeters(174.19), new Rotation2d(Math.PI)));
            add(new Pose2d(leadingX, Units.inchesToMeters(152.19), new Rotation2d(Math.PI)));
            add(new Pose2d(leadingX, Units.inchesToMeters(130.19), new Rotation2d(Math.PI)));
            add(new Pose2d(leadingX, Units.inchesToMeters(108.19), new Rotation2d(Math.PI)));
            add(new Pose2d(leadingX, Units.inchesToMeters(86.19), new Rotation2d(Math.PI)));
            add(new Pose2d(leadingX, Units.inchesToMeters(64.19), new Rotation2d(Math.PI)));
            add(new Pose2d(leadingX, Units.inchesToMeters(42.19), new Rotation2d(Math.PI)));
            add(new Pose2d(leadingX, Units.inchesToMeters(20.19), new Rotation2d(Math.PI)));
          }
        };

    private static final double leadingVelocity = 0.5;
    public static final List<List<PathPoint>> scoringPP =
        new ArrayList<>() {
          {
            add(
                new ArrayList<>() {
                  {
                    add(
                        new PathPoint(
                            leadingPoses.get(0).getTranslation(),
                            new Rotation2d(Math.PI),
                            leadingPoses.get(0).getRotation(),
                            leadingVelocity));
                    add(
                        new PathPoint(
                            scoringPoses.get(0).getTranslation(),
                            new Rotation2d(Math.PI),
                            scoringPoses.get(0).getRotation()));
                  }
                });
            add(
                new ArrayList<>() {
                  {
                    add(
                        new PathPoint(
                            leadingPoses.get(1).getTranslation(),
                            new Rotation2d(Math.PI),
                            leadingPoses.get(1).getRotation(),
                            leadingVelocity));
                    add(
                        new PathPoint(
                            scoringPoses.get(1).getTranslation(),
                            new Rotation2d(Math.PI),
                            scoringPoses.get(1).getRotation()));
                  }
                });
            add(
                new ArrayList<>() {
                  {
                    add(
                        new PathPoint(
                            leadingPoses.get(2).getTranslation(),
                            new Rotation2d(Math.PI),
                            leadingPoses.get(2).getRotation(),
                            leadingVelocity));
                    add(
                        new PathPoint(
                            scoringPoses.get(2).getTranslation(),
                            new Rotation2d(Math.PI),
                            scoringPoses.get(2).getRotation()));
                  }
                });
            add(
                new ArrayList<>() {
                  {
                    add(
                        new PathPoint(
                            leadingPoses.get(3).getTranslation(),
                            new Rotation2d(Math.PI),
                            leadingPoses.get(3).getRotation(),
                            leadingVelocity));
                    add(
                        new PathPoint(
                            scoringPoses.get(3).getTranslation(),
                            new Rotation2d(Math.PI),
                            scoringPoses.get(3).getRotation()));
                  }
                });
            add(
                new ArrayList<>() {
                  {
                    add(
                        new PathPoint(
                            leadingPoses.get(4).getTranslation(),
                            new Rotation2d(Math.PI),
                            leadingPoses.get(4).getRotation(),
                            leadingVelocity));
                    add(
                        new PathPoint(
                            scoringPoses.get(4).getTranslation(),
                            new Rotation2d(Math.PI),
                            scoringPoses.get(4).getRotation()));
                  }
                });
            add(
                new ArrayList<>() {
                  {
                    add(
                        new PathPoint(
                            leadingPoses.get(5).getTranslation(),
                            new Rotation2d(Math.PI),
                            leadingPoses.get(5).getRotation(),
                            leadingVelocity));
                    add(
                        new PathPoint(
                            scoringPoses.get(5).getTranslation(),
                            new Rotation2d(Math.PI),
                            scoringPoses.get(5).getRotation()));
                  }
                });
            add(
                new ArrayList<>() {
                  {
                    add(
                        new PathPoint(
                            leadingPoses.get(6).getTranslation(),
                            new Rotation2d(Math.PI),
                            leadingPoses.get(6).getRotation(),
                            leadingVelocity));
                    add(
                        new PathPoint(
                            scoringPoses.get(6).getTranslation(),
                            new Rotation2d(Math.PI),
                            scoringPoses.get(6).getRotation()));
                  }
                });
            add(
                new ArrayList<>() {
                  {
                    add(
                        new PathPoint(
                            leadingPoses.get(7).getTranslation(),
                            new Rotation2d(Math.PI),
                            leadingPoses.get(7).getRotation(),
                            leadingVelocity));
                    add(
                        new PathPoint(
                            scoringPoses.get(7).getTranslation(),
                            new Rotation2d(Math.PI),
                            scoringPoses.get(7).getRotation()));
                  }
                });
            add(
                new ArrayList<>() {
                  {
                    add(
                        new PathPoint(
                            leadingPoses.get(8).getTranslation(),
                            new Rotation2d(Math.PI),
                            leadingPoses.get(8).getRotation(),
                            leadingVelocity));
                    add(
                        new PathPoint(
                            scoringPoses.get(8).getTranslation(),
                            new Rotation2d(Math.PI),
                            scoringPoses.get(8).getRotation()));
                  }
                });
          }
        };

    public static final List<Pose2d> loadingLead =
        new ArrayList<>() {
          {
            add(new Pose2d(13.8, 7.34, new Rotation2d()));
            add(new Pose2d(13.8, 6.0, new Rotation2d()));
          }
        };

    public static final List<Pose2d> loadingFinish =
        new ArrayList<>() {
          {
            add(new Pose2d(14.6, 7.34, new Rotation2d()));
            add(new Pose2d(14.6, 6.0, new Rotation2d()));
          }
        };

    public static final List<PathPoint> loadingRail =
        new ArrayList<>() {
          {
            add(
                new PathPoint(
                    loadingLead.get(0).getTranslation(),
                    new Rotation2d(),
                    loadingLead.get(0).getRotation(),
                    leadingVelocity));
            add(
                new PathPoint(
                    loadingFinish.get(0).getTranslation(),
                    new Rotation2d(),
                    loadingFinish.get(0).getRotation()));
          }
        };

    public static final List<PathPoint> loadingBarrier =
        new ArrayList<>() {
          {
            add(
                new PathPoint(
                    loadingLead.get(1).getTranslation(),
                    new Rotation2d(),
                    loadingLead.get(1).getRotation(),
                    leadingVelocity));
            add(
                new PathPoint(
                    loadingFinish.get(1).getTranslation(),
                    new Rotation2d(),
                    loadingFinish.get(1).getRotation()));
          }
        };

    public static final RectanglePoseArea field =
        new RectanglePoseArea(new Translation2d(0.0, 0.0), new Translation2d(16.54, 8.02));
    public static final RectanglePoseArea scoreArea =
        new RectanglePoseArea(new Translation2d(1.23, 0.0), new Translation2d(2.92, 5.33));
    public static final RectanglePoseArea chargeArea =
        new RectanglePoseArea(new Translation2d(2.92, 0.0), new Translation2d(5.3, 5.33));
    public static final Translation3d cameraTranslation = new Translation3d(0.5, 0.0, 0.5);
    public static final Rotation3d cameraRotation = new Rotation3d(0, 0, 0);

    public static final List<Pose2d> barrierCorridor =
        new ArrayList<>() {
          {
            add(new Pose2d(2.53, 4.66, new Rotation2d(Math.PI)));
            add(new Pose2d(5.3, 4.66, new Rotation2d(Math.PI)));
          }
        };

    public static final List<PathPoint> barrierCorridorPPOut =
        new ArrayList<>() {
          {
            add(
                new PathPoint(
                    barrierCorridor.get(0).getTranslation(),
                    new Rotation2d(),
                    barrierCorridor.get(0).getRotation(),
                    3.0));
            add(
                new PathPoint(
                    barrierCorridor.get(1).getTranslation(),
                    new Rotation2d(),
                    barrierCorridor.get(1).getRotation(),
                    3.0));
          }
        };

    public static final PathPoint barrierCorridorOuter =
        new PathPoint(
            barrierCorridor.get(1).getTranslation(),
            new Rotation2d(Math.PI),
            barrierCorridor.get(1).getRotation(),
            3.0);
    public static final PathPoint barrierCorridorInner =
        new PathPoint(
            barrierCorridor.get(0).getTranslation(),
            new Rotation2d(Math.PI),
            barrierCorridor.get(0).getRotation(),
            3.0);

    public static final List<Pose2d> bumpCorridor =
        new ArrayList<>() {
          {
            add(new Pose2d(2.53, 0.88, new Rotation2d(Math.PI)));
            add(new Pose2d(5.3, 0.88, new Rotation2d(Math.PI)));
          }
        };

    public static final List<PathPoint> bumpCorridorPPOut =
        new ArrayList<>() {
          {
            add(
                new PathPoint(
                    bumpCorridor.get(0).getTranslation(),
                    new Rotation2d(),
                    bumpCorridor.get(0).getRotation(),
                    3.0));
            add(
                new PathPoint(
                    bumpCorridor.get(1).getTranslation(),
                    new Rotation2d(),
                    bumpCorridor.get(1).getRotation(),
                    3.0));
          }
        };

    public static final PathPoint bumpCorridorOuter =
        new PathPoint(
            bumpCorridor.get(1).getTranslation(),
            new Rotation2d(Math.PI),
            bumpCorridor.get(1).getRotation(),
            3.0);
    public static final PathPoint bumpCorridorInner =
        new PathPoint(
            bumpCorridor.get(0).getTranslation(),
            new Rotation2d(Math.PI),
            bumpCorridor.get(0).getRotation(),
            3.0);

    public static final List<Pose2d> midCorridor =
        new ArrayList<>() {
          {
            add(new Pose2d(2.53, 2.77, new Rotation2d(Math.PI)));
            add(new Pose2d(5.3, 2.77, new Rotation2d(Math.PI)));
          }
        };

    public static final PathPoint midCorridorOuter =
        new PathPoint(
            midCorridor.get(1).getTranslation(),
            new Rotation2d(Math.PI),
            midCorridor.get(1).getRotation(),
            3.0);
    public static final PathPoint midCorridorInner =
        new PathPoint(
            midCorridor.get(0).getTranslation(),
            new Rotation2d(Math.PI),
            midCorridor.get(0).getRotation(),
            3.0);

    public static final Pose2d stationWaypoint = new Pose2d(12.72, 6.77, new Rotation2d());
    public static final Pose2d centerChargeStation =
        new Pose2d(3.86, 2.73, new Rotation2d(Math.PI));
    public static final List<PathPoint> centerChargePP =
        new ArrayList<>() {
          {
            add(
                new PathPoint(
                    centerChargeStation.getTranslation(),
                    centerChargeStation.getRotation(),
                    centerChargeStation.getRotation(),
                    3.0));
          }
        };
    public static final PathPoint stationWaypointIn =
        new PathPoint(
            stationWaypoint.getTranslation(), new Rotation2d(), stationWaypoint.getRotation(), 3.0);
    public static final PathPoint stationWaypointOut =
        new PathPoint(
            stationWaypoint.getTranslation(),
            new Rotation2d(Math.PI),
            stationWaypoint.getRotation());
  }

  public static final class LED {
    public static final int PWMPORT = 0, LENGTH = 20;
  }
}
