package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {

  /** Class representing the inputs for the intake rollers. */
  @AutoLog
  public static class RollerIOInputs {
    public double rollerLeftAppliedVolts = 0.0;
    public double rollerLeftCurentAmps = 0.0;
    public double rollerLeftCurrentSpeed = 0.0;
    public double rollerRightAppliedVolts = 0.0;
    public double rollerRightCurentAmps = 0.0;
    public double rollerRightCurrentSpeed = 0.0;
  }

  /** Method to update the inputs of the Intake subsystem. */
  public default void updateInputs(RollerIOInputs inputs) {}

  public default void setRollerSpeed(double speed) {}

  public default void updatePIDFromDashboard() {}

  default void stop() {}

  default void setBrakeMode(boolean enable) {}
}
