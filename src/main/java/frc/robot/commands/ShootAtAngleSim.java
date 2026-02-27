package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;

public class ShootAtAngleSim extends Command {
  private final IntakeSimulation simIntake;
  private final SwerveDriveSimulation simDrive;
  private final Hood hoodSim;
  private final Shooter shooterSim;
  private final double maxVelocityRPM = 6000;
  private double velocityRPM;
  private final Distance initialHeight = Distance.ofBaseUnits(0.45, Meters);
  private LinearVelocity launchSpeed = LinearVelocity.ofBaseUnits(8, MetersPerSecond);
  private Angle launchAngle = Angle.ofBaseUnits(15, Degrees);

  private int shotsMade = 0;

  public ShootAtAngleSim(
      IntakeSimulation simIntake,
      SwerveDriveSimulation simDrive,
      Hood hoodSim,
      Shooter shooterSim,
      double velocityRPM,
      Angle launchAngle) {

    this.simIntake = simIntake;
    this.simDrive = simDrive;
    this.hoodSim = hoodSim;
    this.shooterSim = shooterSim;
    this.velocityRPM = velocityRPM;
    this.launchAngle = launchAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launchSpeed =
        LinearVelocity.ofBaseUnits(
            8, MetersPerSecond); // TODO Update Example Launch Speed of the projectile
    launchAngle =
        Angle.ofBaseUnits(15, Degrees); // TODO Update Example Launch Angle of the projectile
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    launchSpeed = LinearVelocity.ofBaseUnits((velocityRPM / maxVelocityRPM) * 8, MetersPerSecond);

    if (!simIntake.obtainGamePieceFromIntake()) {
      return;
    }
    RebuiltFuelOnFly projectile = createProjectile(launchAngle);
    projectile.setHitTargetCallBack(() -> Logger.recordOutput("Shots Made", ++shotsMade));
    SimulatedArena.getInstance()
        .addGamePieceProjectile(
            projectile
                .withTargetPosition(
                    () ->
                        FieldMirroringUtils.toCurrentAllianceTranslation(
                            FieldConstants.HUB_TARGET_TRANSLATION))
                // increase tolerance so shots that visually go through the target register as hits
                .withTargetTolerance(FieldConstants.HUB_TARGET_TOLERANCE)
                .withProjectileTrajectoryDisplayCallBack(
                    (poses) ->
                        Logger.recordOutput(
                            "successfulShotsTrajectory", poses.toArray(Pose3d[]::new)),
                    (poses) ->
                        Logger.recordOutput(
                            "missedShotsTrajectory", poses.toArray(Pose3d[]::new))));
  }

  private RebuiltFuelOnFly createProjectile(Angle launchAngle) {
    return new RebuiltFuelOnFly(
        simDrive.getSimulatedDriveTrainPose().getTranslation(),
        new Translation2d(0, 0), // shooter offet from center
        simDrive.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
        simDrive.getSimulatedDriveTrainPose().getRotation(),
        initialHeight, // initial height of the ball, in meters
        this.getLaunchSpeed(), // initial velocity, in m/s
        launchAngle); // shooter angle
  }

  private LinearVelocity getLaunchSpeed() {
    return launchSpeed;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
