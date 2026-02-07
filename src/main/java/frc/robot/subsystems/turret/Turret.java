package frc.robot.subsystems.turret;

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
   * Moves the turret to the fastest version of the target angle.
   * 
   * @param targetAngle0to360 The angle you want (e.g., 90 degrees).
   */
  public Command setTurretAngleFastestPath(double targetAngle0to360) {
    return this.run(() -> {
      // 1. Get current position in degrees
      double currentPosDegrees = inputs.turretResolvedPosition * 360.0;
      
      // 2. Normalize the input to be within 0-360 just in case
      double normalizedTarget = targetAngle0to360 % 360.0;
      if (normalizedTarget < 0) normalizedTarget += 360.0;

      // 3. Calculate the two possible absolute positions for this angle
      // Option 1: The angle in the first rotation (0-360)
      // Option 2: The angle in the second rotation (360-720)
      double option1 = normalizedTarget;
      double option2 = normalizedTarget + 360.0;

      // 4. Determine which option is closer to our current position
      double chosenAngle;
      if (Math.abs(option1 - currentPosDegrees) <= Math.abs(option2 - currentPosDegrees)) {
        chosenAngle = option1;
      } else {
        chosenAngle = option2;
      }

      // 5. Send the chosen position (converted back to rotations) to the IO layer
      io.setTurretPosition(chosenAngle / 360.0);
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