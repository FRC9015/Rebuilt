package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

  @AutoLog
  public static class TurretIOInputs {
    public double turretAppliedVolts = 0.0;
    public double turretCurrentAmps = 0.0;
    public double turretMotorPosition = 0.0;
    public boolean turretAtSetpoint = false;
    public double turretSetpoint = 0.0;
    public double turretError = 0.0;

    public boolean hallEffectTriggered = false; // NEW: True when magnet is detected
    public boolean isZeroed = false; // NEW: Tracks if seeded

    public double turretResolvedPosition = 0.0;
    public double turretResolvedPositionDegrees = 0.0;
    public boolean turretResolvedValid = false;
    public double turretMotorVelocity = 0.0;
    public Rotation2d driveSetpoint = new Rotation2d();
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void stop() {}

  public default void setBrakeMode() {}

  public default void setCoastMode() {}

  public default void setTurretPosition(double value) {}

  public default void seedMotorPosition(double positionRotations) {}

  public default void setTurretSetPoint(double value) {}

  public default void setTurretVoltage(double voltage) {}
}