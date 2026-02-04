package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.turretConstants;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final TurretIO io;
  private int e1Rotation = 0;
  private int e2Rotation = 0;
  private double[] e1n = new double[(int) turretConstants.e1_teeth];
  private double[] e2n = new double[(int) turretConstants.e2_teeth];

  public Turret(TurretIO io) {
    this.io = io;
    for (int x = 0; x < turretConstants.e1_teeth - 1; x++) {
      e1n[x] = x;
    }
    for (int x = 0; x < turretConstants.e2_teeth - 1; x++) {
      e2n[x - 0] = x;
    }
  }


  public double trueAngle() {
    if((inputs.turretEncoderFinalRatioPosition * inputs.turretEncoderNoRatioPosition) == (turretConstants.e1_teeth*turretConstants.e2_teeth)){
      e1Rotation ++;
      e2Rotation ++;
    }
    double e1math = e1n[e1Rotation] + (inputs.turretEncoderFinalRatioPosition/360);
    double e2math = e2n[e2Rotation] + (inputs.turretEncoderNoRatioPosition/360);

    if(e1math == e2math){
      return e1math;
    }

    
    
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
  }
}
