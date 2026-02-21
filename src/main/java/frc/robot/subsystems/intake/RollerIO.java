package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {

  /** Class representing the inputs for the intake rollers. */
  @AutoLog
  public static class RollerIOInputs {
    public double rollerAppliedVolts = 0.0;
    public double rollerCurentAmps = 0.0;
    public double rollerCurrentSpeed = 0.0;
    public boolean rollerEncoderConnected = false;
    public double rollerRPM = 0.0;
  }

  /** Method to update the inputs of the Intake subsystem. */
  public default void updateInputs(RollerIOInputs inputs) {}

  public default void setRollerSpeed(double speed) {}

  public default void updatePIDFromDashboard() {}

  default void stop() {}

  default void setBrakeMode(boolean enable) {}
}
