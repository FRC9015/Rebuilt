package frc.robot.commands;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;
import java.util.function.Supplier;

public class HoodAutoAim extends Command {
  private final Hood hood;
  private Pose2d pose;
  private Pose2d targetPose;
  private Pose2d flippedTargetPose;
  private InterpolatingTreeMap<Double, Double> interpTable;

  public HoodAutoAim(
      Hood hood,
      Supplier<Pose2d> poseSupplier,
      Pose2d targetPose,
      InterpolatingTreeMap<Double, Double> interp) {
    this.hood = hood;
    this.pose = poseSupplier.get();
    this.targetPose = targetPose;
    this.flippedTargetPose = FlippingUtil.flipFieldPose(targetPose);
    this.interpTable = interp;
    addRequirements(hood);
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
    hood.setHoodPosition(setpoint);
  }
}
