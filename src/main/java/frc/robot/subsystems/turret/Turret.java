package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.turretConstants;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final TurretIO io;

  public Turret(TurretIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (inputs.turretResolvedValid) {
      // Use the constant threshold from TurretConstants
      if (Math.abs(inputs.turretMotorPosition - inputs.turretResolvedPosition)
          > turretConstants.SYNC_THRESHOLD) {
        io.seedMotorPosition(inputs.turretResolvedPosition);
      }
    }

    Logger.processInputs("Turret", inputs);
  }
}
