package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  private final AprilTagFieldLayout aprilTagLayout;

  public VisionIOPhotonVision(String name, Transform3d robotToCamera, AprilTagFieldLayout layout) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
    this.aprilTagLayout = layout;
  }

  // public static boolean isTagAllowed(int tagId) {
  //   // 1. If it's explicitly blacklisted, drop it
  //   if (VisionConstants.BLACKLISTED_TAGS.contains(tagId)) {
  //     return false;
  //   }

  //   // 2. Check alliance
  //   Optional<Alliance> alliance = DriverStation.getAlliance();
  //   if (alliance.isPresent()) {
  //     if (alliance.get() == Alliance.Blue) {
  //       // Drop tags that belong strictly to the Red side
  //       return !VisionConstants.RED_TAGS.contains(tagId);
  //     } else {
  //       // Drop tags that belong strictly to the Blue side
  //       return !VisionConstants.BLUE_TAGS.contains(tagId);
  //     }
  //   }

  //   // If alliance is not yet known (in the pit/sim), allow non-blacklisted tags
  //   return true;
  // }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new java.util.ArrayList<>();

    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
      // Find the best allowed target for auto-aim / latest observation
      PhotonTrackedTarget bestAllowedTarget = null;
      // for (PhotonTrackedTarget candidate : result.targets) {
      //   if (isTagAllowed(candidate.getFiducialId())) {
      //     bestAllowedTarget = candidate;
      //     break;
      //   }
      // }

      // if (bestAllowedTarget != null) {
      //   var corners = bestAllowedTarget.getDetectedCorners();
      //   double width = 0;
      //   double height = 0;

      //   if (corners.size() >= 4) {
      //     width = corners.get(2).x - corners.get(3).x;
      //     height = corners.get(3).y - corners.get(0).y;
      //   }

      //   inputs.latestTargetObservation =
      //       new TargetObservation(
      //           Rotation2d.fromDegrees(bestAllowedTarget.getYaw()),
      //           Rotation2d.fromDegrees(bestAllowedTarget.getPitch()),
      //           width,
      //           height,
      //           result.getTimestampSeconds());
      // }

      // ---------------- Multi-Tag Result ----------------
      if (result.multitagResult.isPresent()) {
        MultiTargetPNPResult multitagResult = result.multitagResult.get();

        // Check if ANY used tag is blacklisted/opposite alliance
        boolean containsInvalidTag = false;
        for (short id : multitagResult.fiducialIDsUsed) {
          // if (!isTagAllowed((int) id)) {
          //   containsInvalidTag = true;
          //   break;
          // }
        }

        // Drop immediately: skip 3D math and allocations
        if (containsInvalidTag) {
          continue;
        }

        // Calculate robot pose only if tags are valid
        Transform3d fieldToCamera = multitagResult.estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        double totalTagDistance = 0.0;
        for (PhotonTrackedTarget target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }

        tagIds.addAll(multitagResult.fiducialIDsUsed);

        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(),
                robotPose,
                multitagResult.estimatedPose.ambiguity,
                multitagResult.fiducialIDsUsed.size(),
                totalTagDistance / result.targets.size(),
                multitagResult.fiducialIDsUsed));

        // ---------------- Single-Tag Result ----------------
        // } else if (bestAllowedTarget != null) {
        //   // Use the best allowed target found above
        //   Optional<Pose3d> tagPose =
        // aprilTagLayout.getTagPose(bestAllowedTarget.getFiducialId());
        //   if (tagPose.isPresent()) {
        //     Transform3d fieldToTarget =
        //         new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
        //     Transform3d cameraToTarget = bestAllowedTarget.bestCameraToTarget;
        //     Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
        //     Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        //     Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(),
        // fieldToRobot.getRotation());

        //     tagIds.add((short) bestAllowedTarget.getFiducialId());

        //     poseObservations.add(
        //         new PoseObservation(
        //             result.getTimestampSeconds(),
        //             robotPose,
        //             bestAllowedTarget.poseAmbiguity,
        //             1,
        //             cameraToTarget.getTranslation().getNorm(),
        //             List.of((short) bestAllowedTarget.getFiducialId())));
        //   }
        // }
      }

      inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
      inputs.tagIds = tagIds.stream().mapToInt(Short::intValue).toArray();
    }
  }
}
