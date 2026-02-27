package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;

public class ShooterIOSim implements ShooterIO {
  private final IntakeSimulation intakeSimulation;
  private final SwerveDriveSimulation swerveDriveSimulation;
  private final double velocityRPM = 6000; // TODO Update Example RPM value for the shooter
  private final Distance initialHeight =
      Distance.ofBaseUnits(0.45, Meters); // TODO Update Example Initial Height of
  // the projectile
  private LinearVelocity launchSpeed =
      LinearVelocity.ofBaseUnits(
          8, MetersPerSecond); // TODO Update Example Launch Speed of the projectile
  private Angle launchAngle =
      Angle.ofBaseUnits(15, Degrees); // TODO Update Example Launch Angle of the projectile
  private int shotsMade = 0;

  public ShooterIOSim(IntakeSimulation simIntake, SwerveDriveSimulation simDrive) {
    this.intakeSimulation = simIntake;
    this.swerveDriveSimulation = simDrive;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.flywheelLinearVelocity = this.getLaunchSpeed();
    inputs.launchAngle = this.getLaunchAngle();
  }

  private LinearVelocity getLaunchSpeed() {
    return launchSpeed;
  }

  private Angle getLaunchAngle() {
    return launchAngle;
  }

  public void setHoodPosition(double angle) {
    launchAngle = Angle.ofBaseUnits(Math.PI / 2 + angle, Degrees);
  }

  public void stopHood() {
    launchAngle = Angle.ofBaseUnits(ShooterConstants.HOOD_RESTING_ANGLE, Degrees);
  }

  public void setFlyWheelSpeed(double rpm) {
    // Update the launch speed based on the RPM; assumed to be 16 meters/second at
    // 6000 RPM
    launchSpeed = LinearVelocity.ofBaseUnits((rpm / velocityRPM) * 8, MetersPerSecond);
    // if (!intakeSimulation.obtainGamePieceFromIntake()) {
    //   System.out.println("No game piece obtained from intake, cannot shoot.");
    //   return;
    // }

    if (!intakeSimulation.obtainGamePieceFromIntake()) {
      return;
    }
    RebuiltFuelOnFly projectile = createProjectile();
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

  private RebuiltFuelOnFly createProjectile() {
    return new RebuiltFuelOnFly(
        swerveDriveSimulation.getSimulatedDriveTrainPose().getTranslation(),
        new Translation2d(0, 0), // shooter offet from center
        swerveDriveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
        swerveDriveSimulation.getSimulatedDriveTrainPose().getRotation(),
        initialHeight, // initial height of the ball, in meters
        this.getLaunchSpeed(), // initial velocity, in m/s
        this.getLaunchAngle()); // shooter angle
  }
}
