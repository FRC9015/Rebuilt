package frc.robot.commands;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.ShootingUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TurretAngleAim extends Command {

  private final Supplier<Pose2d> poseSupplier;
  private final Turret turret;
  private final Supplier<Pose2d> targetPoseSupplier;
  private final Drive drive;
  private final InterpolatingTreeMap<Double, Double> timeOfFlightInterp;

  public TurretAngleAim(
      Supplier<Pose2d> poseSupplier,
      Turret turret,
      Supplier<Pose2d> targetPose,
      Drive drive,
      InterpolatingTreeMap<Double, Double> timeOfFlightInterp) {
    this.poseSupplier = poseSupplier;
    this.turret = turret;
    this.targetPoseSupplier = targetPose;
    this.drive = drive;
    this.timeOfFlightInterp = timeOfFlightInterp;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    Pose2d robotPose = poseSupplier.get();
    Pose2d targetPose = targetPoseSupplier.get();
    Pose2d flippedTargetPose = FlippingUtil.flipFieldPose(targetPose);

    boolean isRed =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Red;
    Translation2d realTargetPos =
        isRed ? flippedTargetPose.getTranslation() : targetPose.getTranslation();

    // GET VIRTUAL DATA
    var shotData =
        ShootingUtil.calculateVirtualTarget(
            robotPose, realTargetPos, drive.getChassisSpeeds(), timeOfFlightInterp);

    // Calculate Turret offset on field
    Translation2d turretOffset =
        new Translation2d(TurretConstants.TURRET_X_OFFSET, TurretConstants.TURRET_Y_OFFSET)
            .rotateBy(robotPose.getRotation());
    Translation2d turretFieldPos = robotPose.getTranslation().plus(turretOffset);

    // AIM AT VIRTUAL TARGET
    Translation2d turretToVirtualTarget = shotData.virtualTargetPos.minus(turretFieldPos);
    Rotation2d fieldAngleToHub = turretToVirtualTarget.getAngle();

    Rotation2d relativeSetpoint = fieldAngleToHub.minus(robotPose.getRotation());
    double headingSetpoint = MathUtil.inputModulus(relativeSetpoint.getDegrees(), 0, 360);

    turret.setTurretSetPoint(headingSetpoint);
    double directionSetpoint = turret.setTurretAngleFastestPath(headingSetpoint);
    turret.setPositionVoid(directionSetpoint);

    Logger.recordOutput("Turret/VirtualTarget", shotData.virtualTargetPos);
    Logger.recordOutput("Turret/VirtualDistance", shotData.virtualDistance);
  }
}
