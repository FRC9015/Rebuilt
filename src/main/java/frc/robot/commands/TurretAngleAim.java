package frc.robot.commands;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.ZoneLogic;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.Turret;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TurretAngleAim extends Command {

  private final Supplier<Pose2d> poseSupplier;
  private final Turret turret;
  private final Supplier<Pose2d> targetPoseSupplier;
  private final Drive drive;
  private final InterpolatingTreeMap<Double, Double> timeOfFlightInterp;
  private final ZoneLogic zone;

  public TurretAngleAim(
      Supplier<Pose2d> poseSupplier,
      Turret turret,
      Supplier<Pose2d> targetPose,
      Drive drive,
      InterpolatingTreeMap<Double, Double> timeOfFlightInterp,
      ZoneLogic zone) {
    this.poseSupplier = poseSupplier;
    this.turret = turret;
    this.targetPoseSupplier = targetPose;
    this.drive = drive;
    this.timeOfFlightInterp = timeOfFlightInterp;
    this.zone = zone;
    addRequirements(turret);
  }

  private void aimAtTarget(Pose2d robotPose, Translation2d targetPos, String logPrefix) {
    // 1. Turret field position
    Translation2d turretOffset =
        new Translation2d(TurretConstants.TURRET_X_OFFSET, TurretConstants.TURRET_Y_OFFSET)
            .rotateBy(robotPose.getRotation());
    Translation2d turretFieldPos = robotPose.getTranslation().plus(turretOffset);

    // 2. Vector and distance
    Translation2d turretToTarget = targetPos.minus(turretFieldPos);
    double distance = turretToTarget.getNorm();

    // 3. Unit vector
    Translation2d unitToTarget = turretToTarget.div(distance);

    // 4. Robot velocity
    ChassisSpeeds speeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), robotPose.getRotation());
    double vx = speeds.vxMetersPerSecond;
    double vy = speeds.vyMetersPerSecond;

    // If robot is stationary — aim directly at target with no compensation
    if (Math.abs(vx) < 0.02 && Math.abs(vy) < 0.02) {
      Rotation2d fieldAngle = turretToTarget.getAngle();
      Rotation2d relative = fieldAngle.minus(robotPose.getRotation());
      double heading = MathUtil.inputModulus(relative.getDegrees(), 0, 360);
      turret.setTurretSetPoint(heading);
      turret.setPositionVoid(turret.setTurretAngleFastestPath(heading));
      return;
    }

    // 5. Dot product
    double robotSpeedAlongShot = (vx * unitToTarget.getX()) + (vy * unitToTarget.getY());

    // 6. Flight time and fuel speed
    double timeOfFlight = timeOfFlightInterp.get(distance);
    double fuelSpeedAlongShot = distance / timeOfFlight;

    // 7. Corrected flight time
    double totalSpeedAlongShot = Math.max(fuelSpeedAlongShot + robotSpeedAlongShot, 0.1);
    double correctedFlightTime = distance / totalSpeedAlongShot;

    // 8. Drift compensation
    double driftX = vx * correctedFlightTime;
    double driftY = vy * correctedFlightTime;
    Translation2d compensatedTarget = targetPos.minus(new Translation2d(driftX, driftY));

    // 9. Angle to compensated target
    Translation2d turretToCompensated = compensatedTarget.minus(turretFieldPos);
    Rotation2d fieldAngleToTarget = turretToCompensated.getAngle();

    // 10. Robot-relative, normalized to 0-360
    Rotation2d relativeSetpoint = fieldAngleToTarget.minus(robotPose.getRotation());
    double headingSetpoint = MathUtil.inputModulus(relativeSetpoint.getDegrees(), 0, 360);

    // 11. Send to subsystem
    turret.setTurretSetPoint(headingSetpoint);
    double directionSetpoint = turret.setTurretAngleFastestPath(headingSetpoint);
    turret.setPositionVoid(directionSetpoint);

    // 12. Logging
    Logger.recordOutput(logPrefix + "/HeadingSetpoint", headingSetpoint);
    Logger.recordOutput(logPrefix + "/TargetPos", targetPos);
    Logger.recordOutput(logPrefix + "/CompensatedTarget", compensatedTarget);
    Logger.recordOutput(logPrefix + "/RawFlightTime", timeOfFlight);
    Logger.recordOutput(logPrefix + "/CorrectedFlightTime", correctedFlightTime);
    Logger.recordOutput(logPrefix + "/FuelSpeedAlongShot", fuelSpeedAlongShot);
    Logger.recordOutput(logPrefix + "/RobotSpeedAlongShot", robotSpeedAlongShot);
    Logger.recordOutput(logPrefix + "/DriftVector", new Translation2d(driftX, driftY));
    Logger.recordOutput(logPrefix + "/Zone", zone.getCurrentFieldZone().toString());
    Logger.recordOutput(
        logPrefix + "/TurretFieldPos", new Pose2d(turretFieldPos, fieldAngleToTarget));
  }

  @Override
  public void execute() {
    Pose2d robotPose = poseSupplier.get();
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

    Pose2d targetPose = targetPoseSupplier.get();
    Pose2d flippedTargetPose = FlippingUtil.flipFieldPose(targetPose);
    Translation2d targetPos =
        isRed ? flippedTargetPose.getTranslation() : targetPose.getTranslation();

    ZoneLogic.FieldZone currentZone = zone.getCurrentFieldZone();
    boolean isInNeutral =
        currentZone == ZoneLogic.FieldZone.NEUTRAL_ZONE_LEFT
            || currentZone == ZoneLogic.FieldZone.NEUTRAL_ZONE_RIGHT;

    aimAtTarget(robotPose, targetPos, isInNeutral ? "Turret/PassAim" : "Turret/ShootAim");
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    turret.setTurretSetPoint(0);
  }
}

// package frc.robot.commands;

// import com.pathplanner.lib.util.FlippingUtil;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.TurretConstants;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.subsystems.turret.Turret;
// import frc.robot.util.ShootingUtil;
// import java.util.function.Supplier;
// import org.littletonrobotics.junction.Logger;

// public class TurretAngleAim extends Command {

//   private final Supplier<Pose2d> poseSupplier;
//   private final Turret turret;
//   private final Supplier<Pose2d> targetPoseSupplier;
//   private final Drive drive;
//   private final InterpolatingTreeMap<Double, Double> timeOfFlightInterp;

//   public TurretAngleAim(
//       Supplier<Pose2d> poseSupplier,
//       Turret turret,
//       Supplier<Pose2d> targetPose,
//       Drive drive,
//       InterpolatingTreeMap<Double, Double> timeOfFlightInterp) {
//     this.poseSupplier = poseSupplier;
//     this.turret = turret;
//     this.targetPoseSupplier = targetPose;
//     this.drive = drive;
//     this.timeOfFlightInterp = timeOfFlightInterp;
//     addRequirements(turret);
//   }

//   @Override
//   public void execute() {
//     Pose2d robotPose = poseSupplier.get();
//     Pose2d targetPose = targetPoseSupplier.get();
//     Pose2d flippedTargetPose = FlippingUtil.flipFieldPose(targetPose);

//     boolean isRed =
//         DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
//             == DriverStation.Alliance.Red;
//     Translation2d realTargetPos =
//         isRed ? flippedTargetPose.getTranslation() : targetPose.getTranslation();

//     // GET VIRTUAL DATA
//     var shotData =
//         ShootingUtil.calculateVirtualTarget(
//             robotPose, realTargetPos, drive.getChassisSpeeds(), timeOfFlightInterp);

//     // Calculate Turret offset on field
//     Translation2d turretOffset =
//         new Translation2d(TurretConstants.TURRET_X_OFFSET, TurretConstants.TURRET_Y_OFFSET)
//             .rotateBy(robotPose.getRotation());
//     Translation2d turretFieldPos = robotPose.getTranslation().plus(turretOffset);

//     // AIM AT VIRTUAL TARGET
//     Translation2d turretToVirtualTarget = shotData.virtualTargetPos.minus(turretFieldPos);
//     Rotation2d fieldAngleToHub = turretToVirtualTarget.getAngle();

//     Rotation2d relativeSetpoint = fieldAngleToHub.minus(robotPose.getRotation());
//     double headingSetpoint = MathUtil.inputModulus(relativeSetpoint.getDegrees(), 0, 360);

//     turret.setTurretSetPoint(headingSetpoint);
//     double directionSetpoint = turret.setTurretAngleFastestPath(headingSetpoint);
//     turret.setPositionVoid(directionSetpoint);

//     Logger.recordOutput("Turret/VirtualTarget", shotData.virtualTargetPos);
//     Logger.recordOutput("Turret/VirtualDistance", shotData.virtualDistance);
//   }
// }
