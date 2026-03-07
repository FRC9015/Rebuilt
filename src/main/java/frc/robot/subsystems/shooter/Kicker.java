package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Kicker extends SubsystemBase {
  private TalonFX motor;

  public Kicker(int id) {
    motor = new TalonFX(id);

    TalonFXConfiguration kickerConfig =
        new TalonFXConfiguration()
            .withSlot0(Constants.ShooterConstants.kickerSlotVelocityConfigs)
            .withFeedback(Constants.ShooterConstants.kickerFeedbackConfigs);

    motor.getConfigurator().apply(kickerConfig);
  }

  public void setKicker(double value) {
    motor.set(value);
  }

  public Command runKicker(double value) {
    return this.startEnd(() -> setKicker(value), () -> setKicker(0));
  }
}
