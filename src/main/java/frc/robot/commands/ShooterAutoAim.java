package frc.robot.commands;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.ShootingUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShooterAutoAim extends Command {
  private final Shooter shooter;
  private Supplier<Pose2d> pose;
  private Supplier<Pose2d> targetPoseSupplier;
  private InterpolatingTreeMap<Double, Double> shooterInterpTable;
  private InterpolatingTreeMap<Double, Double> timeOfFlightInterp;
  private final Drive drive;

  public ShooterAutoAim(
      Shooter shooter,
      Supplier<Pose2d> poseSupplier,
      Supplier<Pose2d> targetPoseSupplier,
      InterpolatingTreeMap<Double, Double> shooterInterp,
      InterpolatingTreeMap<Double, Double> tofInterp,
      Drive drive) {
    this.shooter = shooter;
    this.pose = poseSupplier;
    this.targetPoseSupplier = targetPoseSupplier;
    this.shooterInterpTable = shooterInterp;
    this.drive = drive;
    this.timeOfFlightInterp = tofInterp;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    Pose2d currentRobotPose = pose.get();
    Pose2d targetPose = targetPoseSupplier.get();
    boolean isRed =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Red;

    Translation2d realTargetPos =
        isRed
            ? FlippingUtil.flipFieldPose(targetPose).getTranslation()
            : targetPose.getTranslation();

    // GET VIRTUAL DATA
    var shotData =
        ShootingUtil.calculateVirtualTarget(
            currentRobotPose, realTargetPos, drive.getChassisSpeeds(), timeOfFlightInterp);

    // Use VIRTUAL distance for LUT lookup
    double setpoint = shooterInterpTable.get(shotData.virtualDistance);

    shooter.setShooterSpeed(setpoint);
    Logger.recordOutput("Shooter/autoSetpoint", setpoint);
    Logger.recordOutput("Shooter/VirtualDistance", shotData.virtualDistance);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setShooterSpeed(0);
  }
}
