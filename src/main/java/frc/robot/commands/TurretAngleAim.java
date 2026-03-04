package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.turret.Turret;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Command that tracks the position of the robot and attempts to keep the turret pointed in the
 * direction of the hub.
 */
public class TurretAngleAim extends Command {

  private final Supplier<Pose2d> poseSupplier;
  private Turret turret;
  private double y = 6.95;
  private double x = 3.186;

  /**
   * Constructs a TrackTargetCommand
   *
   * @param poseSupplier supplier to use to get the robot's current pose
   * @param turret is a instance of the turret subsystem so that the turret can be set to the
   *     calculated position. the turret from robot container will go into this command
   */
  public TurretAngleAim(Supplier<Pose2d> poseSupplier, Turret turret) {
    this.poseSupplier = poseSupplier;
    this.turret = turret;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    Pose2d robotCurrentPose =
        new Pose2d(
            new Translation2d(
                poseSupplier.get().getX() + Units.inchesToMeters(x),
                poseSupplier.get().getY() - Units.inchesToMeters(y)),
            new Rotation2d(
                poseSupplier.get().getRotation().getRadians() + Units.degreesToRadians(90)));

    // Get the robot's pose relative to the target.
    Pose2d relativePose =
        robotCurrentPose.relativeTo(
            // switchs which pose(red or blue) based on the allaince
            DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                ? FieldConstants.HUB_POSE_BLUE
                : FieldConstants.HUB_POSE_RED);

    // Get the angle to the hub relative to the position of the robot
    double angleToHub = Math.atan2(relativePose.getY(), relativePose.getX());

    // Calculates the angle the turret needs to be at based on the angle from the robot to the hub -
    // the angle the robot is already at
    double headingSetpoint =
        Units.radiansToDegrees(angleToHub) - robotCurrentPose.getRotation().getDegrees();
    // the atan2 command is -180 to 180 so this gets it to 0,360 because you add 360 and get
    // positive and then you add another 180 because its -180 t0 180 and then you modulo 360 to
    // finialize it
    headingSetpoint = (headingSetpoint + 540) % 360;

    // runs the turret function for setting the angle based on a given degree
    turret.setTurretSetPoint(headingSetpoint);
    turret.setTurretAngleFastestPath(headingSetpoint);
    Logger.recordOutput("Turret/headingsetpoint", headingSetpoint);
    Logger.recordOutput("Turret/angleToHub", angleToHub);
    Logger.recordOutput("Turret/RelativePose", relativePose);
  }
}
