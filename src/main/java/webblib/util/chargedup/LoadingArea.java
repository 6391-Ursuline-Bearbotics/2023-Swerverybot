package webblib.util.chargedup;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.Auton;
import frc.robot.commands.swervedrive2.auto.GoToLoadingZone.LOADING_SIDE;
import webblib.util.HolonomicPose2d;
import webblib.util.RectanglePoseArea;

public class LoadingArea {
  private final RectanglePoseArea largeLoadingRect;
  private final RectanglePoseArea smallLoadingRect;
  private final HolonomicPose2d doubleSubstationBarrier;
  private final HolonomicPose2d doubleSubstationRail;

  public LoadingArea(
      RectanglePoseArea largeLoadingRect,
      RectanglePoseArea smallLoadingRect,
      HolonomicPose2d doubleSubstationRail,
      HolonomicPose2d doubleSubstationBarrier) {
    this.largeLoadingRect = largeLoadingRect;
    this.smallLoadingRect = smallLoadingRect;
    this.doubleSubstationBarrier = doubleSubstationBarrier;
    this.doubleSubstationRail = doubleSubstationRail;
  }

  public RectanglePoseArea getLargeLoadingRectangle() {
    return largeLoadingRect;
  }

  public RectanglePoseArea getSmallLoadingRectangle() {
    return smallLoadingRect;
  }

  public HolonomicPose2d getDoubleSubstationBarrier() {
    return doubleSubstationBarrier;
  }

  public HolonomicPose2d getDoubleSubstationRail() {
    return doubleSubstationRail;
  }

  public boolean isPoseWithinLoadingArea(Pose2d pose) {
    return largeLoadingRect.isPoseWithinArea(pose) || smallLoadingRect.isPoseWithinArea(pose);
  }

  /**
   * Get best scoring area. Assumes scoring area zones do not overlap.
   *
   * @param pose current pose of robot
   * @return either null if not in scoring area, or the scoring are if in scoring area
   */
  public static Pose2d getScoringExitArea(Pose2d pose, LOADING_SIDE side) {
    if (side == LOADING_SIDE.RAIL) {
      
    }
    else {
    }
  }
}
