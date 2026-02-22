// // package frc.robot.subsystems.vision;

// // import edu.wpi.first.math.geometry.Transform3d;
// // import edu.wpi.first.wpilibj2.command.SubsystemBase;
// // import org.littletonrobotics.junction.Logger;
// // import org.photonvision.PhotonCamera;
// // import org.photonvision.targeting.PhotonPipelineResult;
// // import org.photonvision.targeting.PhotonTrackedTarget;

// // public class ObjectDetection extends SubsystemBase {
// //   private final PhotonCamera camera = new PhotonCamera("FrontCamera");

// //   @Override
// //   public void periodic() {
// //     var results = camera.getAllUnreadResults();
// //     if (results.isEmpty()) {
// //       Logger.recordOutput("Vision/ObjectDetection/HasResult", false);
// //       return;
// //     }

// //     Logger.recordOutput("Vision/ObjectDetection/HasResult", true);

// //     PhotonPipelineResult result = results.get(0);

// //     if (!result.hasTargets()) {
// //       Logger.recordOutput("Vision/ObjectDetection/HasTargets", false);
// //       return;
// //     }

// //     Logger.recordOutput("Vision/ObjectDetection/HasTargets", true);

// //     PhotonTrackedTarget target = result.getBestTarget();

// //     int classId = target.getFiducialId();
// //     Transform3d camToTarget = target.getBestCameraToTarget();

// //     Logger.recordOutput("Vision/ObjectDetection/TargetID", classId);
// //     Logger.recordOutput("Vision/ObjectDetection/X", camToTarget.getX());
// //     Logger.recordOutput("Vision/ObjectDetection/Y", camToTarget.getY());
// //     Logger.recordOutput("Vision/ObjectDetection/Z", camToTarget.getZ());
// //     Logger.recordOutput(
// //         "Vision/ObjectDetection/YawDegrees",
// Math.toDegrees(camToTarget.getRotation().getZ()));
// //   }
// // }

// // package frc.robot.subsystems.vision;

// // import edu.wpi.first.math.geometry.Transform3d;
// // import edu.wpi.first.wpilibj2.command.SubsystemBase;
// // import org.photonvision.PhotonCamera;
// // import org.photonvision.targeting.PhotonPipelineResult;
// // import org.photonvision.targeting.PhotonTrackedTarget;

// // public class ObjectDetection extends SubsystemBase {

// //   private final PhotonCamera camera = new PhotonCamera("Wsp");

// //   private int counter = 0; // prevents spam

// //   @Override
// //   public void periodic() {
// //     counter++;
// //     if (counter % 10 != 0) return; // print every ~0.2 sec

// //     var results = camera.getAllUnreadResults();

// //     if (results.isEmpty()) {
// //       System.out.println("[Vision] No result");
// //       return;
// //     }

// //     PhotonPipelineResult result = results.get(results.size() - 1); // get latest

// //     if (!result.hasTargets()) {
// //       System.out.println("[Vision] No targets");
// //       return;
// //     }

// //     PhotonTrackedTarget target = result.getBestTarget();

// //     // ✅ Correct for object detection
// //     int classId = target.getDetectedObjectClassID();

// //     // ⚠️ May be approximate depending on your model
// //     Transform3d camToTarget = target.getBestCameraToTarget();

// //     System.out.println("========== OBJECT ==========");
// //     System.out.println("Class ID: " + classId);

// //     // Position (if available)
// //     System.out.println("X: " + camToTarget.getX());
// //     System.out.println("Y: " + camToTarget.getY());
// //     System.out.println("Z: " + camToTarget.getZ());

// //     // Rotation
// //     double yawDeg = Math.toDegrees(camToTarget.getRotation().getZ());
// //     System.out.println("Yaw (deg): " + yawDeg);

// //     // Useful 2D values (often better than 3D)
// //     System.out.println("Yaw (camera): " + target.getYaw());
// //     System.out.println("Pitch: " + target.getPitch());
// //     System.out.println("Area: " + target.getArea());

// //     System.out.println("============================");
// //   }
// // }

// package frc.robot.subsystems.vision;

// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import org.photonvision.PhotonCamera;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;

// public class ObjectDetection extends SubsystemBase {

//   private final PhotonCamera camera = new PhotonCamera("Wsp");

//   private int counter = 0; // prevents spam

//   // // ===================== ADDED =====================
//   private static final double A = -0.0130946306;
//   private static final double B = 0.73998035;
//   private static final double C = -14.1652845;
//   private static final double D = 139.388747;

//   private double lastDistance = 0;

//   private double getDistance(double area) {
//     double distance = A * area * area * area + B * area * area + C * area + D;
//     distance = distance / 100.0;
//     // clamp to reasonable range
//     distance = Math.max(0.2, Math.min(5.0, distance));

//     // smoothing (reduces noise a LOT)
//     distance = 0.8 * lastDistance + 0.2 * distance;
//     lastDistance = distance;

//     return distance;
//   }
//   // // ===================== END ADDED =====================

//   // private static final double A8 = 5.95232536e-07;
//   // private static final double A7 = -7.57747488e-06;
//   // private static final double A6 = -4.52390888e-04;
//   // private static final double A5 = -8.00352875e-04;
//   // private static final double A4 = 3.63173819e-01;
//   // private static final double A3 = -6.04582102e+00;
//   // private static final double A2 = 4.07002833e+01;
//   // private static final double A1 = -1.31615727e+02;
//   // private static final double A0 = 2.66226329e+02;

//   // private double getDistance(double area) {
//   //   double x = area;

//   //   double distance =
//   //       A8 * Math.pow(x, 8)
//   //           + A7 * Math.pow(x, 7)
//   //           + A6 * Math.pow(x, 6)
//   //           + A5 * Math.pow(x, 5)
//   //           + A4 * Math.pow(x, 4)
//   //           + A3 * Math.pow(x, 3)
//   //           + A2 * Math.pow(x, 2)
//   //           + A1 * x
//   //           + A0;

//   //   // convert cm → meters
//   //   distance = distance / 100.0;

//   //   // clamp (VERY important for high-degree polynomials)
//   //   distance = Math.max(0.2, Math.min(5.0, distance));

//   //   // smoothing
//   //   distance = 0.8 * lastDistance + 0.2 * distance;
//   //   lastDistance = distance;

//   //   return distance;
//   // }

//   @Override
//   public void periodic() {
//     counter++;
//     if (counter % 10 != 0) return; // print every ~0.2 sec

//     var results = camera.getAllUnreadResults();

//     if (results.isEmpty()) {
//       System.out.println("[Vision] No result");
//       return;
//     }

//     PhotonPipelineResult result = results.get(results.size() - 1); // get latest

//     if (!result.hasTargets()) {
//       System.out.println("[Vision] No targets");
//       return;
//     }

//     PhotonTrackedTarget target = result.getBestTarget();

//     // ✅ Correct for object detection
//     int classId = target.getDetectedObjectClassID();

//     // ⚠️ May be approximate depending on your model
//     Transform3d camToTarget = target.getBestCameraToTarget();

//     System.out.println("========== OBJECT ==========");
//     System.out.println("Class ID: " + classId);

//     // Position (if available)
//     System.out.println("X: " + camToTarget.getX());
//     System.out.println("Y: " + camToTarget.getY());
//     System.out.println("Z: " + camToTarget.getZ());

//     // Rotation
//     double yawDeg = Math.toDegrees(camToTarget.getRotation().getZ());
//     System.out.println("Yaw (deg): " + yawDeg);

//     // Useful 2D values (often better than 3D)
//     System.out.println("Yaw (camera): " + target.getYaw());
//     System.out.println("Pitch: " + target.getPitch());
//     System.out.println("Area: " + target.getArea());

//     // ===================== ADDED =====================
//     double distance = getDistance(target.getArea());
//     System.out.println("Estimated Distance: " + distance);
//     // ===================== END ADDED =====================

//     System.out.println("============================");
//   }
// }

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectDetection extends SubsystemBase {

  private final PhotonCamera camera = new PhotonCamera("Wsp");

  private int counter = 0; // prevents spam

  // ================================
  // Pitch (degrees) → Distance (cm)
  // ================================
  private double getDistanceFromPitch(double pitch) {
    double x = pitch;
    return -0.93193701 * Math.pow(x, 3)
        + 2.15170038 * Math.pow(x, 2)
        - 7.41876578 * x
        + 57.61577731;
  }

  @Override
  public void periodic() {
    counter++;
    if (counter % 10 != 0) return; // print every ~0.2 sec

    var results = camera.getAllUnreadResults();

    if (results.isEmpty()) {
      System.out.println("[Vision] No result");
      return;
    }

    PhotonPipelineResult result = results.get(results.size() - 1); // get latest

    if (!result.hasTargets()) {
      System.out.println("[Vision] No targets");
      return;
    }

    PhotonTrackedTarget target = result.getBestTarget();

    // ✅ Correct for object detection
    int classId = target.getDetectedObjectClassID();

    // ⚠️ May be approximate depending on your model
    Transform3d camToTarget = target.getBestCameraToTarget();

    System.out.println("========== OBJECT ==========");
    System.out.println("Class ID: " + classId);
    double distanceCm = 0;
    // Position (if available)
    System.out.println("X: " + camToTarget.getX());
    System.out.println("Y: " + camToTarget.getY());
    System.out.println("Z: " + camToTarget.getZ());

    // Rotation
    double yawDeg = Math.toDegrees(camToTarget.getRotation().getZ());
    System.out.println("Yaw (deg): " + yawDeg);

    // Useful 2D values
    double yaw = target.getYaw();
    double pitch = target.getPitch();
    double area = target.getArea();

    System.out.println("Yaw (camera): " + yaw);
    System.out.println("Pitch: " + pitch);
    System.out.println("Area: " + area);

    // ================================
    // Distance from pitch
    // ================================
    // double lastDistance = distanceCm;
    distanceCm = getDistanceFromPitch(pitch);
    // distanceCm = (0.7 * lastDistance + 0.3 * distanceCm);
    System.out.println("Distance (cm, pitch model): " + distanceCm); // smoothing

    System.out.println("============================");
  }
}
