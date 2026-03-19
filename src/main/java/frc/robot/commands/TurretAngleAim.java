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
    Pose2d filpedTargetPose = FlippingUtil.flipFieldPose(targetPose);
    // 1. Calculate Turret Position on the Field
    // Rotate the offset vector by the robot's current heading
    Translation2d turretOffset =
        new Translation2d(TurretConstants.TURRET_X_OFFSET, TurretConstants.TURRET_Y_OFFSET)
            .rotateBy(robotPose.getRotation());

    // Add that rotated offset to the robot's center position
    Translation2d turretFieldPos = robotPose.getTranslation().plus(turretOffset);

    // 2. Select Target (Hub) based on Alliance
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

    Translation2d targetPosOld =
        isRed ? filpedTargetPose.getTranslation() : targetPose.getTranslation();
    double distance = robotPose.getTranslation().getDistance(targetPosOld);

    // CODE FOR SHOOT ON THE MOVE, NEEDS TO BE FINALIZED AND TESTED WITH PROPER INTERP TABLES
    Translation2d targetPos =
        targetPosOld.minus(
            new Translation2d(
                drive.getChassisSpeeds().vxMetersPerSecond * timeOfFlightInterp.get(distance),
                drive.getChassisSpeeds().vyMetersPerSecond * timeOfFlightInterp.get(distance)));

    // 3. Calculate Angle from Turret to Target (Field Relative)
    Translation2d turretToTarget = targetPos.minus(turretFieldPos);
    Rotation2d fieldAngleToHub = turretToTarget.getAngle();

    // 4. Calculate Robot-Relative Angle
    // Setpoint = (Target Direction) - (Robot Direction)
    Rotation2d relativeSetpoint =
        fieldAngleToHub.minus(robotPose.getRotation().plus(Rotation2d.fromDegrees(360)));

    // 5. Convert to 0-360 range
    double headingSetpoint = MathUtil.inputModulus(relativeSetpoint.getDegrees(), 0, 360);

    // 6. Send to Subsystem
    // The fastestPath logic will take this 0-360 and decide if it's better
    // to go to the positive or negative version based on your -0.7 to 0.7 limit.
    turret.setTurretSetPoint(headingSetpoint);
    double directionSetpoint = turret.setTurretAngleFastestPath(headingSetpoint);
    turret.setPositionVoid(directionSetpoint);
    // Logging for debugging
    Logger.recordOutput("Turret/HeadingSetpoint0to360", headingSetpoint);
    Logger.recordOutput("DISTANCETHING", targetPos.getDistance(robotPose.getTranslation()));
    Logger.recordOutput("Turret/TurretFieldPos", new Pose2d(turretFieldPos, fieldAngleToHub));
    Logger.recordOutput("Turret/Targetpose", targetPos);
  }
}
