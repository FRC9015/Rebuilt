package frc.robot.subsystems.climb;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.ClimbIO.ClimbIOInputs;
import org.littletonrobotics.junction.Logger;

/** Subsystem for controlling the robot climb mechanism. */
public class Climb extends SubsystemBase {
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
  private final PIDController pidController;

  // PID constants
  // TODO: Update PID constants during tuning
  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kTolerance = 0.0;

  /**
   * Constructs an Climb subsystem.
   *
   * @param io The input/output interface for the Climb.
   */
  public Climb(ClimbIO io) {
    this.io = io;
    this.pidController = new PIDController(kP, kI, kD);
    pidController.setTolerance(kTolerance);
  }

  /**
   * Sets target position for the climber
   *
   * @param position pre-set climb positions listed in ClimbIO
   */
  public void setPresetPosition(ClimbIOInputs.ClimbPositions position) {
    pidController.setSetpoint(position.getClimbEncoderPositions());
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    // Calculates the power needed to reach setpoint
    double output = pidController.calculate(inputs.climberPosition);

    // Sends calculated power to motor
    if (pidController.atSetpoint()) {
      io.setClimbVoltage(0);
    } else {
      io.setClimbVoltage(output);
    }

    // Records setpoint, output, position input data.
    Logger.recordOutput("Climber/Setpoint", pidController.getSetpoint());
    Logger.recordOutput("Climber/Output", output);
    Logger.recordOutput("Climber/PositionInput", inputs.climberPosition);
  }
}
