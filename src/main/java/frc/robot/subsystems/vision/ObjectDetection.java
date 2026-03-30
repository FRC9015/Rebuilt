package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.geometry.Rotation2d;

public class ObjectDetection {
    private final InterpolatingTreeMap<Double, Double> distanceInterp =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
    
    public Translation2d getObjectTranslation(double pitch, Rotation2d yaw) {
        return PhotonUtils.estimateCameraToTargetTranslation(distanceInterp.get(pitch), yaw);
    }

    private final PhotonCamera camera = new PhotonCamera("ObjectCamera");

    public List<Translation2d> getObjectTranslations() {
        List<PhotonPipelineResult> targets = camera.getAllUnreadResults();

        // basically, if there are targets, get the translation for each target and return it as a list
        return targets.stream()
            .filter(result -> result.hasTargets())
            .flatMap(result -> result.getTargets().stream())
            .map(target -> {
                return getObjectTranslation(target.getPitch(), Rotation2d.fromDegrees(target.getYaw()));
            })
            .toList();
    }

    public List<Double> getObjectDistances(List<Translation2d> translations) {
        List<Double> distances = translations.stream()
            .map(translation -> translation.getNorm())
            .toList();
        return distances;
    }

    public List<List<Translation2d>> clusterPoints(List<Translation2d> points, double maxDistance) {
        List<List<Translation2d>> clusters = new ArrayList<>();

        for (Translation2d point : points) {
            boolean added = false;
            for (List<Translation2d> cluster : clusters) {
                // Check distance to the first point or average of the cluster
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