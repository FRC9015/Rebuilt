package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.turretConstants;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final TurretIO io;

  public Turret(TurretIO io) {
    this.io = io;
  }

  /**
   * Moves the turret to the fastest version of the target angle (0-360) while strictly staying
   * within the MINROTATION and MAXROTATION bounds.
   *
   * @param targetAngleDegrees The target angle in a 0-360 circle.
   */
  public Command setTurretAngleFastestPath(double targetAngleDegrees) {
    return this.run(
        () -> {
          // 1. Get bounds and current position in degrees
          double minDegrees = turretConstants.MINROTATION * 360.0;
          double maxDegrees = turretConstants.MAXROTATION * 360.0;
          double currentPosDegrees = inputs.turretResolvedPosition * 360.0;

          // 2. Normalize user input to [0, 360)
          double target = targetAngleDegrees % 360.0;
          if (target < 0) target += 360.0;

          // 3. Generate the two possible absolute destinations
          double option1 = target; // e.g. 90
          double option2 = target - 360.0; // e.g. -270

          // 4. Check which options are physically reachable
          boolean opt1Valid = (option1 >= minDegrees && option1 <= maxDegrees);
          boolean opt2Valid = (option2 >= minDegrees && option2 <= maxDegrees);

          double chosenDegrees;

          if (opt1Valid && opt2Valid) {
            // Both are within bounds, pick the one that is closer
            if (Math.abs(option1 - currentPosDegrees) <= Math.abs(option2 - currentPosDegrees)) {
              chosenDegrees = option1;
            } else {
              chosenDegrees = option2;
            }
          } else if (opt1Valid) {
            // Only option 1 is safe
            chosenDegrees = option1;
          } else if (opt2Valid) {
            // Only option 2 is safe
            chosenDegrees = option2;
          } else {
            // Neither is valid (this happens if your range is narrower than 360 total degrees)
            // Fallback: Pick option 1 but clamp it to the turret limits
            chosenDegrees = MathUtil.clamp(option1, minDegrees, maxDegrees);
          }

          // 5. Send the safe, optimized position to the motor
          // (The setTurretPosition method in IO expects degrees based on your last file)
          io.setTurretPosition(chosenDegrees);
        });
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (inputs.turretResolvedValid) {
      if (Math.abs(inputs.turretMotorPosition - inputs.turretResolvedPosition)
          > turretConstants.SYNC_THRESHOLD) {
        io.seedMotorPosition(inputs.turretResolvedPosition);
      }
    }
    Logger.processInputs("Turret", inputs);
  }
}
