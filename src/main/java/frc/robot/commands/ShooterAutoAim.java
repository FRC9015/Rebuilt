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
  private Pose2d targetPose;
  private Pose2d flippedTargetPose;
  private InterpolatingTreeMap<Double, Double> shooterInterpTable;
  private final Drive drive;

  public ShooterAutoAim(
      Shooter shooter,
      Supplier<Pose2d> poseSupplier,
      Pose2d targetPose,
      InterpolatingTreeMap<Double, Double> shooterInterp,
      Drive drive) {
    this.shooter = shooter;
    this.pose = poseSupplier;
    this.targetPose = targetPose;
    this.flippedTargetPose = FlippingUtil.flipFieldPose(targetPose);
    this.shooterInterpTable = shooterInterp;
    this.drive = drive;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    Pose2d currentRobotPose = pose.get();

    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

    Translation2d targetPos =
        isRed ? flippedTargetPose.getTranslation() : targetPose.getTranslation();
    double distance = currentRobotPose.getTranslation().getDistance(targetPos);
    // targetPos =
    //     targetPos.minus(
    //         new Translation2d(
    //             drive.getChassisSpeeds().vxMetersPerSecond * timeOfFlightInterp.get(distance),
    //             drive.getChassisSpeeds().vyMetersPerSecond * timeOfFlightInterp.get(distance)));

    double botToTargetPoseDistance = currentRobotPose.getTranslation().getDistance(targetPos);
    double setpoint = shooterInterpTable.get(botToTargetPoseDistance);
    shooter.setShooterSpeed(setpoint);
    shooter.setKickerSpeed(1);
    Logger.recordOutput("Shooter/autoSetpoint", setpoint);
    Logger.recordOutput("DistanceEdit", botToTargetPoseDistance);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopKicker();
    shooter.setShooterSpeed(0);
  }
}
