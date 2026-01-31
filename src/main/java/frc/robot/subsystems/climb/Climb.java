package frc.robot.subsystems.climb;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.ClimbIO.ClimbIOInputs;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  private final PIDController pidController;

  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kTolerance = 0.0;

  public Climb(ClimbIO io) {
    this.io = io;
    this.pidController = new PIDController(kP, kI, kD);
    pidController.setTolerance(kTolerance);
  }

  public void setPresetPosition(ClimbIOInputs.ClimbPositions position) {
  
    io.setClimbPosition(position.getClimbEncoderPositions());
    io.updateInputs(inputs);

    Logger.recordOutput("Climber/Setpoint", position.getClimbEncoderPositions());
    Logger.recordOutput("Climber/PositionInput", inputs.climberPosition);
  }
}
