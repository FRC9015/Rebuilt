package frc.robot.subsystems.indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;

/** Indexer IO implementation using REV SPARK MAX + NEO (REVLib 2026 style). */
public class IndexerIOSparkFlex implements IndexerIO {

  private final SparkFlex motor1;

  private final double maxSpeed = 6784.0;

  private final SparkClosedLoopController indexerPIDController;

  private final Debouncer encoderConnectedDebounce = new Debouncer(0.5);
  private final SparkClosedLoopController closedLoop;
  private SparkFlexConfig motorConfig;

  public IndexerIOSparkFlex(int motorId1) {
    motor1 = new SparkFlex(motorId1, MotorType.kBrushless);

    // Build a declarative config (REVLib 2026)
    SparkFlexConfig cfg = new SparkFlexConfig();

    cfg.idleMode(SparkBaseConfig.IdleMode.kBrake);
    cfg.inverted(false); // flip to true if your indexer spins backwards
    cfg.smartCurrentLimit(45); // amps
    cfg.voltageCompensation(12.0); // consistent voltage behavior
    cfg.closedLoop.pid(0.1, 0, 0);
    motorConfig = new SparkFlexConfig();
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-12, 12)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
        .feedForward
        // kV is now in Volts, so we multiply by the nominal voltage (12V)
        .kV(12.0 / 6784, ClosedLoopSlot.kSlot1);
    // Apply config:
    // - Reset safe params first (gets you to a known baseline)
    // - Don't persist to flash every boot (saves flash wear + avoids config delays)
    motor1.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    closedLoop = motor1.getClosedLoopController();
    indexerPIDController = motor1.getClosedLoopController();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // Applied volts = applied output (-1..1) * bus voltage
    inputs.indexerAppliedVolts = motor1.getAppliedOutput() * motor1.getBusVoltage();

    // Output current in amps
    inputs.indexerCurrentAmps = motor1.getOutputCurrent();
  }

  @Override
  public void stop() {
    motor1.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean enable) {
    SparkFlexConfig cfg = new SparkFlexConfig();
    cfg.idleMode(enable ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
    motor1.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setVoltage(double percent) {
    // Same as your CTRE code: this is open-loop voltage, name is legacy
    closedLoop.setSetpoint(percent, ControlType.kVoltage);
  }
}
