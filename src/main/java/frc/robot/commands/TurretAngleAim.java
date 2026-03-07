// package frc.robot.commands;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.FieldConstants;
// import frc.robot.subsystems.turret.Turret;
// import java.util.function.Supplier;
// import org.littletonrobotics.junction.Logger;

// /**
//  * Command that tracks the position of the robot and attempts to keep the turret pointed in the
//  * direction of the hub.
//  */
// public class TurretAngleAim extends Command {

//   private final Supplier<Pose2d> poseSupplier;
//   private Turret turret;
//   private double y = 6.95;
//   private double x = 3.186;

//   /**
//    * Constructs a TrackTargetCommand
//    *
//    * @param poseSupplier supplier to use to get the robot's current pose
//    * @param turret is a instance of the turret subsystem so that the turret can be set to the
//    *     calculated position. the turret from robot container will go into this command
//    */
//   public TurretAngleAim(Supplier<Pose2d> poseSupplier, Turret turret) {
//     this.poseSupplier = poseSupplier;
//     this.turret = turret;
//     addRequirements(turret);
//   }

//   @Override
//   public void execute() {
//     Pose2d turretCurrentPose =
//         new Pose2d(
//             new Translation2d(
//                 poseSupplier.get().getX() + Units.inchesToMeters(x),
//                 poseSupplier.get().getY() - Units.inchesToMeters(y)),
//             new Rotation2d(
//                 poseSupplier.get().getRotation().getRadians() + Units.degreesToRadians(0)));

//     // Get the robot's pose relative to the target.
//     Pose2d relativePose =
//         turretCurrentPose.relativeTo(
//             // switchs which pose(red or blue) based on the allaince
//             DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
//                 ? FieldConstants.HUB_POSE_BLUE
//                 : FieldConstants.HUB_POSE_RED);

//     // Get the angle to the hub relative to the position of the robot
//     double angleToHub = Math.atan2(relativePose.getY(), relativePose.getX());

//     // Calculates the angle the turret needs to be at based on the angle from the robot to the
// hub -
//     // the angle the robot is already at
//     double headingSetpoint =
//         Units.radiansToDegrees(angleToHub) - turretCurrentPose.getRotation().getDegrees();
//     // the atan2 command is -180 to 180 so this gets it to 0,360 because you add 360 and get
//     // positive and then you add another 180 because its -180 t0 180 and then you modulo 360 to
//     // finialize it

//     headingSetpoint = (headingSetpoint +540) % 360;

//     // runs the turret function for setting the angle based on a given degree
//     // turret.setTurretSetPoint(headingSetpoint);
//     turret.setTurretAngleFastestPath(headingSetpoint);
//     Logger.recordOutput("Turret/headingsetpoint", headingSetpoint);
//     Logger.recordOutput("Turret/angleToHub", angleToHub);
//     Logger.recordOutput("Turret/RelativePose", relativePose);
//   }
// }

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
  private final Pose2d targetPose;
  private final Pose2d filpedTargetPose;
  private final Drive drive;
  private final InterpolatingTreeMap<Double, Double> timeOfFlightInterp;

  public TurretAngleAim(
      Supplier<Pose2d> poseSupplier,
      Turret turret,
      Pose2d targetPose,
      Drive drive,
      InterpolatingTreeMap<Double, Double> timeOfFlightInterp) {
    this.poseSupplier = poseSupplier;
    this.turret = turret;
    this.targetPose = targetPose;
    this.filpedTargetPose = FlippingUtil.flipFieldPose(targetPose);
    this.drive = drive;
    this.timeOfFlightInterp = timeOfFlightInterp;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    Pose2d robotPose = poseSupplier.get();

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

    Translation2d targetPos =
        isRed ? filpedTargetPose.getTranslation() : targetPose.getTranslation();
    double distance = robotPose.getTranslation().getDistance(targetPos);
    // targetPos =
    //     targetPos.minus(
    //         new Translation2d(
    //             drive.getChassisSpeeds().vxMetersPerSecond * timeOfFlightInterp.get(distance),
    //             drive.getChassisSpeeds().vyMetersPerSecond * timeOfFlightInterp.get(distance)));
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
    Logger.recordOutput("Turret/TurretFieldPos", new Pose2d(turretFieldPos, fieldAngleToHub));
  }

  @Override
  public void end(boolean interrupted) {
      // TODO Auto-generated method stub
      super.end(interrupted);
  }
}
