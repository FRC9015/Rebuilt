package frc.robot.commands;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShooterAutoAim extends Command {
  private final Shooter shooter;
  private Supplier<Pose2d> pose;
  private Supplier<Pose2d> targetPoseSupplier;
  private InterpolatingTreeMap<Double, Double> shooterInterpTable;
  private final Drive drive;

  public ShooterAutoAim(
      Shooter shooter,
      Supplier<Pose2d> poseSupplier,
      Supplier<Pose2d> targetPoseSupplier,
      InterpolatingTreeMap<Double, Double> shooterInterp,
      Drive drive) {
    this.shooter = shooter;
    this.pose = poseSupplier;
    this.targetPoseSupplier = targetPoseSupplier;
    this.shooterInterpTable = shooterInterp;
    this.drive = drive;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    Pose2d currentRobotPose = pose.get();
    Pose2d targetPose = targetPoseSupplier.get();
    Pose2d flippedTargetPose = FlippingUtil.flipFieldPose(targetPose);
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

    Translation2d targetPos =
        isRed ? flippedTargetPose.getTranslation() : targetPose.getTranslation();

    double botToTargetPoseDistance = currentRobotPose.getTranslation().getDistance(targetPos);
    double setpoint = shooterInterpTable.get(botToTargetPoseDistance);
    shooter.setShooterSpeed(setpoint);
    Logger.recordOutput("Shooter/autoSetpoint", setpoint);
    Logger.recordOutput("DistanceEdit", botToTargetPoseDistance);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setShooterSpeed(0);
  }
}
