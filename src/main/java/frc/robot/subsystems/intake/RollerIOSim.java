package frc.robot.subsystems.intake;

import org.ironmaple.simulation.IntakeSimulation;

public class RollerIOSim implements RollerIO {
  private final IntakeSimulation intakeSimulation;

  public RollerIOSim(IntakeSimulation simIntake) {
    this.intakeSimulation =
        simIntake; // Assuming the IntakeSimulation is properly initialized and passed in
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.fuelInside = intakeSimulation.getGamePiecesAmount();
  }

  public int returnFuelInside() {
    return intakeSimulation.getGamePiecesAmount();
  }

  @Override
  public void setRollerSpeed(boolean runIntake) {
    if (runIntake) {
      intakeSimulation.startIntake();
    } else {
      intakeSimulation.stopIntake();
    }
  }
}
