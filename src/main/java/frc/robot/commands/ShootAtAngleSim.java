package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
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
  private final Shooter shooter;
  private final Hood hood;

  private final Distance initialHeight =
      Distance.ofBaseUnits(Constants.SimConstants.PROJECTILE_INITIAL_HEIGHT_METERS, Meters);
  private LinearVelocity launchSpeed = LinearVelocity.ofBaseUnits(8, MetersPerSecond);

  private int shotsMade = 0;

  public ShootAtAngleSim(
      IntakeSimulation simIntake,
      SwerveDriveSimulation simDrive,
      Turret turret,
      Shooter shooter,
      Hood hood) {
    this.simIntake = simIntake;
    this.simDrive = simDrive;
    this.turret = turret;
    this.shooter = shooter;
    this.hood = hood;
  }

  public void initialize() {
    launchSpeed = LinearVelocity.ofBaseUnits(0, MetersPerSecond);
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void shootBalls() {
    double rps = shooter.getTargetSpeed();
    double speedMetersPerSecond = 2 * Math.PI * Constants.SimConstants.FLYWHEEL_RADIUS_METERS * rps;
    double efficiencyMultiplier = 0.65; // accounts for slip
    launchSpeed =
        LinearVelocity.ofBaseUnits(speedMetersPerSecond * efficiencyMultiplier, MetersPerSecond);

    if (!simIntake.obtainGamePieceFromIntake()) {
      return;
    }

    Angle currentLaunchAngle = hood.getLaunchAngle();
    RebuiltFuelOnFly projectile = createProjectile(currentLaunchAngle);

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
    Translation2d turretOffset =
        new Translation2d(
                Constants.TurretConstants.TURRET_X_OFFSET,
                Constants.TurretConstants.TURRET_Y_OFFSET)
            .rotateBy(simDrive.getSimulatedDriveTrainPose().getRotation());

    Translation2d startPos =
        simDrive.getSimulatedDriveTrainPose().getTranslation().plus(turretOffset);

    return new RebuiltFuelOnFly(
        startPos,
        new Translation2d(0, 0), // shooter offset from center
        simDrive.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
        new Rotation2d(
            simDrive.getSimulatedDriveTrainPose().getRotation().getRadians()
                + turret.getTurretPositionRadians()), // accounting for drivetrain being flipped?
        initialHeight, // initial height of the ball, in meters
        launchSpeed, // initial velocity, in m/s
        launchAngle); // shooter angle
  }
}
