package frc.robot.commands;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.util.ShootingUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class HoodAutoAim extends Command {
  private final Hood hood;
  private Supplier<Pose2d> pose;
  private Supplier<Pose2d> targetPoseSupplier;
  private InterpolatingTreeMap<Double, Double> timeOfFlightInterp;
  private InterpolatingTreeMap<Double, Double> hoodInterpTable;

  private final Drive drive;

  public HoodAutoAim(
      Hood hood,
      Supplier<Pose2d> poseSupplier,
      Supplier<Pose2d> targetPoseSupplier,
      InterpolatingTreeMap<Double, Double> hoodInterp,
      InterpolatingTreeMap<Double, Double> tofInterp,
      Drive drive) {
    this.hood = hood;
    this.pose = poseSupplier;
    this.targetPoseSupplier = targetPoseSupplier;
    this.hoodInterpTable = hoodInterp;
    this.drive = drive;
    this.timeOfFlightInterp = tofInterp;
    addRequirements(hood);
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
    double setpoint = hoodInterpTable.get(shotData.virtualDistance);

    hood.setHoodPos(setpoint);
    Logger.recordOutput("Hood/setpointauto", setpoint);
  }

  @Override
  public void end(boolean interrupted) {
    hood.setHoodPos(0.0);
  }
}
