package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
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

  // Telemetry Publishers (One per camera)
  private final DoubleArrayPublisher[] gyroTelemetryPubs;

  // Telemetry suppliers
  private final Supplier<Rotation2d> robotYawSupplier;
  private final Supplier<Rotation2d> turretAngleSupplier;
  private final int turretCameraIndex;

  /**
   * @param consumer Usually SwerveDrivePoseEstimator::addVisionMeasurement
   * @param robotYawSupplier Supplier for base chassis rotation (m_drive::getRotation)
   * @param turretAngleSupplier Supplier for current turret encoder angle
   * @param turretCameraIndex The index in the arrays that is the turret (use -1 if none)
   * @param io Array of VisionIO instances (Umbra or Photon)
   */
  public Vision(
      VisionConsumer consumer,
      Supplier<Rotation2d> robotYawSupplier,
      Supplier<Rotation2d> turretAngleSupplier,
      int turretCameraIndex,
      VisionIO... io) {

    this.consumer = consumer;
    this.robotYawSupplier = robotYawSupplier;
    this.turretAngleSupplier = turretAngleSupplier;
    this.turretCameraIndex = turretCameraIndex;
    this.io = io;

    this.inputs = new VisionIOInputsAutoLogged[io.length];
    this.disconnectedAlerts = new Alert[io.length];
    this.gyroTelemetryPubs = new DoubleArrayPublisher[io.length];

    var baseTable = NetworkTableInstance.getDefault().getTable("Umbra").getSubTable("telemetry");

    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
      disconnectedAlerts[i] = new Alert("Vision camera " + i + " disconnected", AlertType.kWarning);

      // Initialize telemetry publisher for this specific camera
      String cameraName = io[i].getName();
      gyroTelemetryPubs[i] =
          baseTable.getSubTable(cameraName).getDoubleArrayTopic("gyro").publish();
    }
  }

  @Override
  public void periodic() {
    // --- STEP 1: PUBLISH SYNCHRONIZED IMU/TURRET ANGLE TO EACH CAMERA ON THE PI ---
    double timestamp = Timer.getFPGATimestamp();
    Rotation2d robotYaw = robotYawSupplier.get();

    for (int i = 0; i < io.length; i++) {
      double cameraYaw = robotYaw.getRadians();

      // If it's the turret camera, combine chassis gyro yaw with turret encoder angle
      if (i == turretCameraIndex) {
        cameraYaw += turretAngleSupplier.get().getRadians();
      }

      gyroTelemetryPubs[i].set(
          new double[] {
            timestamp,
            0.0, // Roll (Placeholder)
            0.0, // Pitch (Placeholder)
            cameraYaw // Synced heading for Pi solver
          });
    }

    // --- STEP 2: PROCESS CAMERAS AS USUAL ---
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);
      disconnectedAlerts[i].set(!inputs[i].connected);

      List<Pose3d> tagPoses = new ArrayList<>();
      List<Pose3d> acceptedRobotPoses = new ArrayList<>();
      List<Pose3d> rejectedRobotPoses = new ArrayList<>();

      for (int tagId : inputs[i].tagIds) {
        VisionConstants.aprilTagLayout.getTagPose(tagId).ifPresent(tagPoses::add);
      }

      for (PoseObservation observation : inputs[i].poseObservations) {
        Pose3d rawPose = observation.pose();
        Pose3d robotPose;

        if (i == turretCameraIndex) {
          Rotation2d turretAngle = turretAngleSupplier.get();
          Transform3d turretRotation =
              new Transform3d(new Translation3d(), new Rotation3d(0, 0, turretAngle.getRadians()));
          Transform3d robotToLensDynamic =
              VisionConstants.ROBOT_TO_TURRET
                  .plus(turretRotation)
                  .plus(VisionConstants.TURRET_TO_CAMERA);

          robotPose = rawPose.transformBy(robotToLensDynamic.inverse());
        } else {
          robotPose = rawPose;
        }

        if (isValid(robotPose, observation)) {
          acceptedRobotPoses.add(robotPose);
          consumer.accept(
              robotPose.toPose2d(), observation.timestamp(), calculateStdDevs(observation));
        } else {
          rejectedRobotPoses.add(robotPose);
        }
      }

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
