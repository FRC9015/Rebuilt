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

  public Vision(VisionConsumer consumer, Supplier<Rotation2d> turretAngleSupplier, int turretCameraIndex, VisionIO... io) {
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

      for (int tagId : inputs[i].tagIds) {
        VisionConstants.aprilTagLayout.getTagPose(tagId).ifPresent(tagPoses::add);
      }

      for (PoseObservation observation : inputs[i].poseObservations) {
        Pose3d pose = observation.pose();

        // --- TURRET RE-CALCULATION ---
        // If this is the turret camera, the IO gave us the LENS pose 
        // because we passed it an identity transform (0 offset)
        if (i == turretCameraIndex) {
          Rotation2d turretAngle = turretAngleSupplier.get();
          pose = pose
              .transformBy(VisionConstants.TURRET_TO_CAMERA.inverse()) 
              .rotateBy(new Rotation3d(0, 0, -turretAngle.getRadians())) 
              .transformBy(VisionConstants.ROBOT_TO_TURRET.inverse());
        }

        // --- FILTERING ---
        boolean rejected = observation.tagCount() == 0 ||
            (observation.tagCount() == 1 && observation.ambiguity() > VisionConstants.MAX_AMBIGUITY) ||
            pose.getX() < 0 || pose.getX() > VisionConstants.FIELD_LENGTH ||
            pose.getY() < 0 || pose.getY() > VisionConstants.FIELD_WIDTH;

        if (rejected) {
          rejectedRobotPoses.add(pose);
          continue; 
        }

        // --- ACCEPT ---
        acceptedRobotPoses.add(pose);
        Matrix<N3, N1> stdDevs = (observation.tagCount() > 1) 
            ? VisionConstants.kMultiTagStdDevs : VisionConstants.kSingleTagStdDevs;
        double trustFactor = 1.0 + (Math.pow(observation.averageTagDistance(), 2) / VisionConstants.STD_DEV_RANGE);
        
        consumer.accept(pose.toPose2d(), observation.timestamp(), stdDevs.times(trustFactor));
      }

      Logger.recordOutput("Vision/Camera" + i + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput("Vision/Camera" + i + "/AcceptedPoses", acceptedRobotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput("Vision/Camera" + i + "/RejectedPoses", rejectedRobotPoses.toArray(new Pose3d[0]));
    }
  }

  public interface VisionConsumer {
    void accept(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs);
  }
}