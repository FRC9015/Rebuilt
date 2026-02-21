package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

  @AutoLog
  public static class TurretIOInputs {
    public double turretAppliedVolts = 0.0;
    public double turretCurrentAmps = 0.0;
    public double turretMotorPosition = 0.0;
    public boolean turretAtSetpoint = false;

    public double encoder13PositionRot = 0.0;
    public boolean encoder13Connected = false;

    public double encoder15PositionRot = 0.0;
    public boolean encoder15Connected = false;

    public double turretResolvedPosition = 0.0;
    public boolean turretResolvedValid = false;
    public double turretMotorVelocity = 0.0;

    public double encoderPositionRot = 0.0;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void stop() {}

  public default void setBrakeMode() {}

  public default void setCoastMode() {}

  public default void setTurretPosition(double value) {}

  public default void seedMotorPosition(double positionRotations) {}
}
