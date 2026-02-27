package frc.robot.subsystems.vision;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectDetection extends SubsystemBase {

  private final PhotonCamera camera = new PhotonCamera("Wsp");
  private int counter = 0; // prevents spam

  private double getDistanceFromPitch(double pitch) { // cubic interpolation
    double x = pitch;
    return -0.93193701 * Math.pow(x, 3)
        + 2.15170038 * Math.pow(x, 2)
        - 7.41876578 * x
        + 57.61577731;
  }

  private Translation2d lastTranslation = new Translation2d();

  @Override
  public void periodic() {
    counter++;
    if (counter % 10 != 0) return; // print every ~0.2 sec b/c vision can get noisy

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

    // Distance calculations
    PhotonTrackedTarget target = result.getBestTarget();    
    double pitch = target.getPitch();
    double distanceCm = 0;
    double lastDistance = distanceCm;
    distanceCm = getDistanceFromPitch(pitch);
    distanceCm = (0.7 * lastDistance + 0.3 * distanceCm);
    System.out.println("Distance (cm, pitch model): " + distanceCm);

    // Convert distance and yaw into translation2d cus why not
    double yawRad = Math.toRadians(target.getYaw());
    Translation2d lastTranslation = new Translation2d(
      distanceCm * Math.cos(yawRad),
      distanceCm * Math.sin(yawRad)
    );
  }

  public Translation2d getLastTranslation() {
    return lastTranslation;
  }
}