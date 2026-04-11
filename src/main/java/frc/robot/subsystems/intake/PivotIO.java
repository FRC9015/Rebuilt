package frc.robot.subsystems.intake;

import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  /** Class representing the inputs for the intake pivot. */
  @AutoLog
  public static class PivotIOInputs {
    public double pivotLeftAppliedVolts = 0.0;
    public double pivotLeftCurrentAmps = 0.0;
    public double pivotLeftCurrentSpeed = 0.0;
    public double pivotPosition = 0.0;
    public double pivotPosition2 = 0.0;
    public double setpointError = 0.0;
    public double setpoint = 0.0;
    public boolean isDeployed = false;
  }

  public enum PivotPositions {
    STOWED(Constants.IntakeConstants.PIVOT_STOWED_POSITION),
    DEPLOYED(Constants.IntakeConstants.PIVOT_DEPLOYED_POSITION),
    AGITATE(0.25),
    AGITATE_MIDDLE(0.75);

    private final double position;

    private PivotPositions(double position) {
      this.position = position;
    }

    public double getPivotPosition() {
      return position;
    }
  }

  public default void updateInputs(PivotIOInputs inputs) {}

  public default void setPivotPosition(double position) {}

  public default void setPivotPosition(PivotPositions position) {}

  public default void updatePIDFromDashboard() {}

  public default void setVolts(double volts) {}

  public default void seedPivotPosition(double position) {}

  default void stop() {}

  default void setBrakeMode(boolean enable) {}
}
