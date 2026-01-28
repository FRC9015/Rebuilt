package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {

  @AutoLog
  public static class ClimbIOInputs {

    public static enum ClimbPositions {
      ReadyToLatch(0),
      ReadyToClimbL1(0.1),
      ReadyToClimbL2(0.2),
      ReadyToClimbL3(0.3),
      FullyClimbedL1(2),
      FullyClimbedL2(3),
      FullyClimbedL3(4),
      Retracted(0);

      private final double climbEncoderPositions;

      private ClimbPositions(double climbEncoderPositions) {
        this.climbEncoderPositions = climbEncoderPositions;
      }

      public double getClimbEncoderPositions() {
        return climbEncoderPositions;
      }
    }

    public double climberAppliedVolts = 0.0;
    public double climberCurrentAmps = 0.0;
    public double climberRPM = 0.0;
    public double climberPosition = 0.0;
    public double servoPosition = 0.0;
  }

  public default void updateInputs(ClimbIOInputs inputs) {}

  default void stop() {}

  default void setBrakeMode(boolean enable) {}

  default void setTopRPM(double rpm) {}

  default void setClimbRPM(double rpm) {}

  default void setClimbPosition(double position) {}
}
