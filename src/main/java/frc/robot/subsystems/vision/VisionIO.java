/*This file has all the inputs that are saved in from the cameras.
 * Each input is updated in the vision.java file
 * it also has the default commands that are defined in VisionIOPhotonVision
 */
package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d(), 0, 0, 0); // default values...
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
  }

  /** Represents the angle to a simple target, not used for pose estimation. */
  public static record TargetObservation(
      Rotation2d tx,
      Rotation2d ty,

      // width
      double targetHorizontalExtentPixels,
      // height
      double targetVerticalExtentPixels,
      double timestamp) {
    public TargetObservation(Rotation2d tx, Rotation2d ty, double timestamp) {
      this(tx, ty, 0, 0, timestamp);
    }

    public double getTargetHorizontalExtentPixels() {
      return targetHorizontalExtentPixels;
    }

    public double getTargetVerticalExtentPixels() {
      return targetVerticalExtentPixels;
    }
  }

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      List<Short> tagIds) {}

  public default void updateInputs(VisionIOInputs inputs) {}
}
