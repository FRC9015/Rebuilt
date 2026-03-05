package frc.robot.commands;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.Supplier;

public class ShooterAutoAim extends Command {
  private final Shooter shooter;
  private Pose2d pose;
  private Pose2d targetPose;
  private Pose2d flippedTargetPose;
  private InterpolatingTreeMap<Double, Double> interpTable;

  public ShooterAutoAim(
      Shooter shooter,
      Supplier<Pose2d> poseSupplier,
      Pose2d targetPose,
      InterpolatingTreeMap<Double, Double> interp) {
    this.shooter = shooter;
    this.pose = poseSupplier.get();
    this.targetPose = targetPose;
    this.flippedTargetPose = FlippingUtil.flipFieldPose(targetPose);
    this.interpTable = interp;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    Pose2d currentRobotPose = pose;

    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

    Translation2d targetPos =
        isRed ? flippedTargetPose.getTranslation() : targetPose.getTranslation();

    double botToTargetPoseDistance = currentRobotPose.getTranslation().getDistance(targetPos);
    double setpoint = interpTable.get(botToTargetPoseDistance);
    shooter.setShooterSpeed(setpoint);
  }
}
