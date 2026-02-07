package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

  @AutoLog
  public static class TurretIOInputs {
    // Motor State
    public double turretAppliedVolts = 0.0;
    public double turretCurrentAmps = 0.0;
    public double turretMotorPosition = 0.0; // Internal Motor Position
    public boolean turretAtSetpoint = false;

    // Encoder 1 (13 Tooth Gear)
    public double encoder13PositionRot = 0.0; // Absolute Rotations (0-1)
    public boolean encoder13Connected = false;

    // Encoder 2 (15 Tooth Gear)
    public double encoder15PositionRot = 0.0; // Absolute Rotations (0-1)
    public boolean encoder15Connected = false;

    // The Calculated "True" Angle from the Math (Populated by the IO Layer)
    public double turretResolvedPosition = 0.0;
    public boolean turretResolvedValid = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(TurretIOInputs inputs) {}

  /** Sets the pivot position based on the desired state. */
  public default void setPivotPosition(double value) {}

  /** Moves the Pivot Up. */
  public default void rotate(double speed) {}

  /** Run slam pivot at amps. */
  default void runCurrent(double amps) {}

  /** Stop slam pivot. */
  default void stop() {}

  /** Enable or disable brake mode on the pivot motor(s). */
  default void setBrakeMode() {}

  default void setCoastMode() {}

  default void setTurretPosition(double value) {}

  /**
   * Force the motor to believe it is at a specific position. Used to sync the motor to the Chinese
   * Remainder Theorem result.
   */
  default void seedMotorPosition(double positionRotations) {}
}
