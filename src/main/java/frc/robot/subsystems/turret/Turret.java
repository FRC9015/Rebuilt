package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final TurretIO io;

  private boolean isLocked = false;
  private int lockCounter = 0;
  private double sumResolved = 0.0;

  public Turret(TurretIO io) {
    this.io = io;
  }

  public Command setTurretAngleFastestPath(double targetAngleDegrees) {
    return this.run(
        () -> {
          // While moving, we trust the Motor position for logic. It never jumps.
          double currentPosDegrees = inputs.turretMotorPosition * 360.0;
          double target = targetAngleDegrees % 360.0;
          if (target < 0) target += 360.0;

          double option1 = target;
          double option2 = target + 360.0;

          double chosen =
              (Math.abs(option1 - currentPosDegrees) <= Math.abs(option2 - currentPosDegrees))
                  ? option1
                  : option2;

          io.setTurretPosition(chosen / 360.0);
        });
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // --- COLD BOOT CALIBRATION ---
    // We average 50 frames (1 second) of absolute math before seeding the motor.
    if (!isLocked && inputs.turretResolvedValid) {
      sumResolved += inputs.turretResolvedPosition;
      lockCounter++;

      if (lockCounter >= 50) {
        double averagePos = sumResolved / 50.0;
        io.seedMotorPosition(averagePos);
        isLocked = true;
        System.out.println("TURRET CALIBRATED AND LOCKED AT: " + averagePos);
      }
    }

    // Once locked, we STOP seeding the motor.
    // This makes the motor perfectly smooth even if the absolute math jitters.

    Logger.recordOutput("Turret/IsLocked", isLocked);
    Logger.processInputs("Turret", inputs);
  }
}
