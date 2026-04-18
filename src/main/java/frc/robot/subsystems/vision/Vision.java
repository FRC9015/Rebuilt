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

      List<Pose3d> acceptedRobotPoses = new ArrayList<>();
      List<Pose3d> rejectedRobotPoses = new ArrayList<>();

      for (PoseObservation observation : inputs[i].poseObservations) {
        Pose3d lensPose = observation.pose(); // This is the Lens on the field
        Pose3d robotPose;

        if (i == turretCameraIndex) {
          // --- THE FIX: DYNAMIC TRANSFORM COMPOSITION ---
          Rotation2d turretAngle = turretAngleSupplier.get();

          // 1. Create a transform that represents the turret's current rotation
          Transform3d turretRotation =
              new Transform3d(new Translation3d(), new Rotation3d(0, 0, turretAngle.getRadians()));

          // 2. Build the full chain from Robot Center -> Lens
          // Chain: RobotCenter -> TurretPivot -> TurretRotation -> LensOffset
          Transform3d robotToLensDynamic =
              VisionConstants.ROBOT_TO_TURRET
                  .plus(turretRotation)
                  .plus(VisionConstants.TURRET_TO_CAMERA);

          // 3. RobotPose = LensPose * (RobotToLens)^-1
          robotPose = lensPose.transformBy(robotToLensDynamic.inverse());

        } else {
          // Static camera logic remains the same
          // (Assuming the IO handled the static offset, or you handle it here)
          robotPose = lensPose;
        }

        // Filtering
        if (isValid(robotPose, observation)) {
          acceptedRobotPoses.add(robotPose);
          consumer.accept(
              robotPose.toPose2d(), observation.timestamp(), calculateStdDevs(observation));
        } else {
          rejectedRobotPoses.add(robotPose);
        }
      }
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
