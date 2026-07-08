package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  private final Supplier<Rotation2d> turretAngleSupplier;
  private final int turretCameraIndex;

  /**
   * Main Vision Subsystem Constructor.
   *
   * @param consumer Usually SwerveDrivePoseEstimator::addVisionMeasurement
   * @param turretAngleSupplier Supplier returning the live angle of your turret
   * @param turretCameraIndex The index in the array representing the turret camera (use -1 if none)
   * @param io One or more VisionIO implementations (e.g. VisionIOUmbra or VisionIOPhotonVision)
   */
  public Vision(
      VisionConsumer consumer,
      Supplier<Rotation2d> turretAngleSupplier,
      int turretCameraIndex,
      VisionIO... io) {
    this.consumer = consumer;
    this.turretAngleSupplier = turretAngleSupplier;
    this.turretCameraIndex = turretCameraIndex;
    this.io = io;

    this.inputs = new VisionIOInputsAutoLogged[io.length];
    this.disconnectedAlerts = new Alert[io.length];

    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
      disconnectedAlerts[i] = new Alert("Vision camera " + i + " disconnected", AlertType.kWarning);
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);
      disconnectedAlerts[i].set(!inputs[i].connected);

      List<Pose3d> tagPoses = new ArrayList<>();
      List<Pose3d> acceptedRobotPoses = new ArrayList<>();
      List<Pose3d> rejectedRobotPoses = new ArrayList<>();

      // Collect physical field poses of tags currently being observed
      for (int tagId : inputs[i].tagIds) {
        VisionConstants.aprilTagLayout.getTagPose(tagId).ifPresent(tagPoses::add);
      }

      for (PoseObservation observation : inputs[i].poseObservations) {
        Pose3d rawPose = observation.pose();
        Pose3d robotPose;

        if (i == turretCameraIndex) {
          // --- DYNAMIC TURRET MATH ---
          // Since the static offset is zeroed out, rawPose represents the camera Lens
          Rotation2d turretAngle = turretAngleSupplier.get();

          Transform3d turretRotation =
              new Transform3d(new Translation3d(), new Rotation3d(0, 0, turretAngle.getRadians()));

          // Build the complete spatial translation: RobotCenter -> TurretPivot -> Rotate ->
          // LensOffset
          Transform3d robotToLensDynamic =
              VisionConstants.ROBOT_TO_TURRET
                  .plus(turretRotation)
                  .plus(VisionConstants.TURRET_TO_CAMERA);

          // Apply the inverse matrix to convert Lens back to Robot Center
          robotPose = rawPose.transformBy(robotToLensDynamic.inverse());

        } else {
          // --- STATIC CAMERA MATH ---
          // The static offset is handled inside the IO class (constructor) or C++ config.json,
          // meaning the observation pose is already converted to the Robot Center.
          robotPose = rawPose;
        }

        // Filtering & Safety Rejections
        if (isValid(robotPose, observation)) {
          acceptedRobotPoses.add(robotPose);
          consumer.accept(
              robotPose.toPose2d(), observation.timestamp(), calculateStdDevs(observation));
        } else {
          rejectedRobotPoses.add(robotPose);
        }
      }

      // Visual debugging arrays for AdvantageScope
      Logger.recordOutput("Vision/Camera" + i + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + i + "/AcceptedPoses", acceptedRobotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + i + "/RejectedPoses", rejectedRobotPoses.toArray(new Pose3d[0]));
    }
  }

  private boolean isValid(Pose3d pose, PoseObservation obs) {
    return obs.tagCount() > 0
        && (obs.tagCount() > 1 || obs.ambiguity() < VisionConstants.MAX_AMBIGUITY)
        && pose.getX() > 0
        && pose.getX() < VisionConstants.FIELD_LENGTH
        && pose.getY() > 0
        && pose.getY() < VisionConstants.FIELD_WIDTH;
  }

  private Matrix<N3, N1> calculateStdDevs(PoseObservation obs) {
    var base =
        (obs.tagCount() > 1) ? VisionConstants.kMultiTagStdDevs : VisionConstants.kSingleTagStdDevs;
    double factor = 1.0 + (Math.pow(obs.averageTagDistance(), 2) / VisionConstants.STD_DEV_RANGE);
    return base.times(factor);
  }

  public interface VisionConsumer {
    void accept(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs);
  }
}
