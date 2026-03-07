package frc.robot.commands;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class HoodAutoAim extends Command {
  private final Hood hood;
  private Supplier<Pose2d> pose;
  private Pose2d targetPose;
  private Pose2d flippedTargetPose;
  private InterpolatingTreeMap<Double, Double> hoodInterpTable;
  private final Drive drive;

  public HoodAutoAim(
      Hood hood,
      Supplier<Pose2d> poseSupplier,
      Pose2d targetPose,
      InterpolatingTreeMap<Double, Double> hoodInterp,
      Drive drive) {
    this.hood = hood;
    this.pose = poseSupplier;
    this.targetPose = targetPose;
    this.flippedTargetPose = FlippingUtil.flipFieldPose(targetPose);
    this.hoodInterpTable = hoodInterp;
    this.drive = drive;
    addRequirements(hood);
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
    double setpoint = hoodInterpTable.get(botToTargetPoseDistance);
    hood.setHoodPos(setpoint);
    Logger.recordOutput("Hood/setpointauto", setpoint);
  }

  @Override
  public void end(boolean interrupted) {
      // TODO Auto-generated method stub
      super.end(interrupted);
  }

  @Override
  public boolean isFinished() {
      // TODO Auto-generated method stub
      return super.isFinished();
  }
}
