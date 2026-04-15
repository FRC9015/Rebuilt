package frc.robot.subsystems.vision;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectDetection {
  private final PhotonCamera camera = new PhotonCamera("ObjectCamera");
  private Translation2d lastLockedTargetLocation = null;
  private int framesSinceLastSeen = 0;
  private final int maxLostFrames = 10;
  private final MedianFilter lockedTargetPitchFilter = new MedianFilter(5);
  private final InterpolatingTreeMap<Double, Double> distanceInterp =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

  public ObjectDetection() {
    distanceInterp.put(-15.0, 1.0);
    distanceInterp.put(-2.0, 3.5);
  }

  public List<Translation2d> getObjectTranslations() {
    var result = camera.getLatestResult();
    List<Translation2d> translations = new ArrayList<>();

    if (!result.hasTargets()) {
      return translations;
    }

    for (PhotonTrackedTarget target : result.getTargets()) {
      translations.add(calculateTranslationFromBottom(target));
    }
    return translations;
  }

  public Optional<Translation2d> getLockedTarget() {
    List<Translation2d> currentTargets = getObjectTranslations();

    if (currentTargets.isEmpty()) {
      framesSinceLastSeen++;
      if (framesSinceLastSeen > maxLostFrames) {
        lastLockedTargetLocation = null;
      }
      return Optional.ofNullable(lastLockedTargetLocation);
    }

    Translation2d bestTarget = null;

    if (lastLockedTargetLocation == null) {
      double minYaw = Double.MAX_VALUE;
      for (Translation2d t : currentTargets) {
        double yawAbs = Math.abs(t.getAngle().getDegrees());
        if (yawAbs < minYaw) {
          minYaw = yawAbs;
          bestTarget = t;
        }
      }
    } else {
      double minDistance = Double.MAX_VALUE;
      for (Translation2d t : currentTargets) {
        double dist = t.getDistance(lastLockedTargetLocation);
        if (dist < minDistance) {
          minDistance = dist;
          bestTarget = t;
        }
      }
    }

    if (bestTarget != null) {
      framesSinceLastSeen = 0;
      double smoothedDistance = lockedTargetPitchFilter.calculate(bestTarget.getNorm());
      lastLockedTargetLocation = new Translation2d(smoothedDistance, bestTarget.getAngle());
      return Optional.of(lastLockedTargetLocation);
    }

    return Optional.empty();
  }

  private Translation2d calculateTranslationFromBottom(PhotonTrackedTarget target) {
    double bottomPitch = -Double.MAX_VALUE;
    var corners = target.getDetectedCorners();

    if (corners == null || corners.isEmpty()) {
      bottomPitch = target.getPitch();
    } else {
      for (var corner : corners) {
        if (corner.y > bottomPitch) {
          bottomPitch = corner.y;
        }
      }
    }

    double distance = distanceInterp.get(bottomPitch);
    return new Translation2d(distance, Rotation2d.fromDegrees(-target.getYaw()));
  }

  public List<Double> getObjectDistances(List<Translation2d> translations) {
    List<Double> distances = new ArrayList<>();
    for (Translation2d t : translations) {
      distances.add(t.getNorm());
    }
    return distances;
  }

  public List<List<Translation2d>> clusterPoints(List<Translation2d> points, double maxDistance) {
    List<List<Translation2d>> clusters = new ArrayList<>();

    for (Translation2d point : points) {
      boolean added = false;
      for (List<Translation2d> cluster : clusters) {
        if (point.getDistance(cluster.get(0)) < maxDistance) {
          cluster.add(point);
          added = true;
          break;
        }
      }

      if (!added) {
        List<Translation2d> newCluster = new ArrayList<>();
        newCluster.add(point);
        clusters.add(newCluster);
      }
    }
    return clusters;
  }
}
