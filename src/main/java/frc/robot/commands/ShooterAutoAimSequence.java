package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.Supplier;

public class ShooterAutoAimSequence extends ParallelCommandGroup {
  public ShooterAutoAimSequence(
      Shooter shooter,
      Hood hood,
      InterpolatingTreeMap<Double, Double> shooterInterp,
      InterpolatingTreeMap<Double, Double> hoodInterp,
      InterpolatingTreeMap<Double, Double> timeOfFlightInterp,
      Supplier<Pose2d> poseSupplier,
      Pose2d targetPose,
      Drive drive) {
    addCommands(
        new HoodAutoAim(hood, poseSupplier, targetPose, hoodInterp, timeOfFlightInterp, drive),
        new ShooterAutoAim(
            shooter, poseSupplier, targetPose, shooterInterp, timeOfFlightInterp, drive));
  }
}
