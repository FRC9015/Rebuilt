package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.util.Units;
import frc.robot.util.PhoenixUtil;
import org.ironmaple.simulation.drivesims.GyroSimulation;

/** Simulation implementation of {@link GyroIO} used in the software simulator. */
public class GyroIOSim implements GyroIO {
  private final GyroSimulation gyro;

  public GyroIOSim(GyroSimulation gyro) {
    this.gyro = gyro;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = gyro.getGyroReading();
    inputs.yawVelocityRadPerSec =
        Units.degreesToRadians(gyro.getMeasuredAngularVelocity().in(RadiansPerSecond));

    inputs.odometryYawTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
    inputs.odometryYawPositions = gyro.getCachedGyroReadings();
  }
}
