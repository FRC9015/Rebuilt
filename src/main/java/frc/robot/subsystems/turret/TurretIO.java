package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

  @AutoLog
  public static class TurretIOInputs {

    // Fields Representing the pivot state and inputs
    public double turretAppliedVolts = 0.0;
    public double turretCurrentAmps = 0.0;
    public double turretPosition = 0.0;
    public double turretMotorPosition = 0.0;
    public double turretEncoderNoRatioPosition = 0.0;
    public double turretEncoderFinalRatioPosition = 0.0;
    public boolean turretEncoderNoRatioConnected = false;
    public boolean turretEncoderFinalRatioConnected = false;
    public boolean turretAtSetpoint = false;
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

  default void setturretPosition(double value) {}
}
