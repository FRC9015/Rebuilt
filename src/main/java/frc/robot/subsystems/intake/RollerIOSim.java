package frc.robot.subsystems.intake;

import org.ironmaple.simulation.IntakeSimulation;

public class RollerIOSim implements RollerIO {
  private final IntakeSimulation intakeSimulation;

  public RollerIOSim(IntakeSimulation simIntake) {
    this.intakeSimulation =
        simIntake; // The intake simulation is passed in from RobotContainer, so it can be shared
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.fuelInside = intakeSimulation.getGamePiecesAmount();
  }

  public int returnFuelInside() {
    return intakeSimulation.getGamePiecesAmount();
  }

  @Override
  public void setRollerSpeed(double speed) {
    if (speed > 0) {
      intakeSimulation.startIntake();
    } else {
      intakeSimulation.stopIntake();
    }
  }
}
