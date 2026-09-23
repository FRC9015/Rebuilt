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
        new TargetObservation(new Rotation2d(), new Rotation2d(), 0, 0, 0);
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
    public int[] targetIDs = new int[0];
    public boolean hasTargets;
  }

  public static record TargetObservation(
      Rotation2d tx,
      Rotation2d ty,
      double targetHorizontalExtentPixels,
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

  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      List<Short> tagIds) {}

  public default void updateInputs(VisionIOInputs inputs) {}

  // --- THE FIX: Expose camera name to Vision.java for dynamic telemetry matching ---
  public default String getName() {
    return "Camera";
  }
}
