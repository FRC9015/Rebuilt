package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

public class ShooterIOSim implements ShooterIO {
  private final Intake intakeFuelTrackerSim;
  private final Drive swerveDriveSimulation;
  private final double velocityRPM = 6000; // TODO Update Example RPM value for the shooter
  private final Distance initialHeight =
      Distance.ofBaseUnits(0.45, Meters); // TODO Update Example Initial Height of the projectile
  private final LinearVelocity launchSpeed =
      LinearVelocity.ofBaseUnits(
          20, MetersPerSecond); // TODO Update Example Launch Speed of the projectile
  private final Angle launchAngle =
      Angle.ofBaseUnits(
          55,
          edu.wpi.first.units.Units.Degrees); // TODO Update Example Launch Angle of the projectile

  public ShooterIOSim(Intake intakeIOSim, Drive swerveDriveSim) {
    this.intakeFuelTrackerSim = intakeIOSim;
    this.swerveDriveSimulation = swerveDriveSim;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    double updatedIntakeFuelCount = intakeFuelTrackerSim.getIntakeFuelCount();
    simulateFuelLaunch(updatedIntakeFuelCount);
  }

  public void simulateFuelLaunch(double updatedValue) {
    if (updatedValue > 0) {
      for (int i = 0; i < updatedValue; i++) {
        RebuiltFuelOnFly fuelOnFly =
            new RebuiltFuelOnFly(
                // Specify the position of the chassis when the note is launched
                swerveDriveSimulation.getPose().getTranslation(),
                // Specify the translation of the shooter from the robot center (in the shooter’s
                // reference frame)
                new Translation2d(0.2, 0),
                // Specify the field-relative speed of the chassis, adding it to the initial
                // velocity of the projectile
                swerveDriveSimulation.getChassisSpeeds(),
                // The shooter facing direction is the same as the robot’s facing direction
                swerveDriveSimulation
                    .getPose()
                    .getRotation(), // TODO Add Shooter Rotation2D From Turret
                // Initial height of the flying note
                initialHeight,
                // The launch speed is proportional to the RPM; assumed to be 16 meters/second at
                // 6000 RPM
                launchSpeed,
                // The angle at which the note is launched
                launchAngle);
                
        fuelOnFly.launch();
      }
    }
  }
}
