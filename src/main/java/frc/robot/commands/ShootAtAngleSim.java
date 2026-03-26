package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.turret.Turret;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;

public class ShootAtAngleSim {
  private final IntakeSimulation simIntake;
  private final SwerveDriveSimulation simDrive;
  private final Turret turret;
  private final double maxVelocityRPM = 6000;
  private double standardHoodOffset = Math.PI / 2;
  private double velocityRPM;
  private final Distance initialHeight = Distance.ofBaseUnits(0.45, Meters);
  private LinearVelocity launchSpeed = LinearVelocity.ofBaseUnits(8, MetersPerSecond);
  private Angle launchAngle = Angle.ofBaseUnits(0, Degrees);
  private double desiredLaunchAngle;

  private int shotsMade = 0;

  public ShootAtAngleSim(
      IntakeSimulation simIntake,
      SwerveDriveSimulation simDrive,
      Turret turret,
      double velocityRPM,
      double desiredLaunchAngle) {
    this.simIntake = simIntake;
    this.simDrive = simDrive;
    this.turret = turret;
    this.velocityRPM = velocityRPM;
    this.desiredLaunchAngle = desiredLaunchAngle;
  }

  public void initialize() {
    launchSpeed =
        LinearVelocity.ofBaseUnits(
            8, MetersPerSecond); // TODO Update Example Launch Speed of the projectile
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void shootBalls() {

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
        new Translation2d(0, 0), // shooter offet from center,
        simDrive.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
        new Rotation2d(
            simDrive.getSimulatedDriveTrainPose().getRotation().getRadians()
                + turret.getTurretPositionRadians()), // accounting for drivertrain being flipped?
        initialHeight, // initial height of the ball, in meters
        this.getLaunchSpeed(), // initial velocity, in m/s
        launchAngle); // shooter angle
  }

  public Angle getLaunchAngle() {
    return launchAngle;
  }

  public void setLaunchAngle(double desiredLaunchAngle) {
    this.launchAngle =
        Angle.ofBaseUnits(
            standardHoodOffset + desiredLaunchAngle,
            Degrees); // TODO Update Example Launch Angle of the projectile;
  }

  private LinearVelocity getLaunchSpeed() {
    return launchSpeed;
  }
}
