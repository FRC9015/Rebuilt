package frc.robot.subsystems.indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants;

/** Indexer IO implementation using REV SPARK MAX + NEO (REVLib 2026 style). */
public class IndexerIOSparkFlex implements IndexerIO {

  private final SparkFlex motor1;
  private final RelativeEncoder encoder;

  private final SparkClosedLoopController closedLoop;
  private SparkFlexConfig motorConfig;

  public IndexerIOSparkFlex(int motorId1) {
    motor1 = new SparkFlex(motorId1, MotorType.kBrushless);
    encoder = motor1.getEncoder();

    // Build a declarative config (REVLib 2026)
    SparkFlexConfig cfg = new SparkFlexConfig();

    cfg.idleMode(SparkBaseConfig.IdleMode.kBrake)
        .inverted(false) // flip to true if your indexer spins backwards
        .smartCurrentLimit(45) // amps
        .voltageCompensation(12.0); // consistent voltage behavior
    motorConfig = new SparkFlexConfig();
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(Constants.IndexerConstants.INDEXER_P)
        .i(Constants.IndexerConstants.INDEXER_I)
        .d(Constants.IndexerConstants.INDEXER_D);
    motor1.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    closedLoop = motor1.getClosedLoopController();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // REV RelativeEncoder velocity is RPM by default
    inputs.indexerVelocity = encoder.getVelocity();

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
    closedLoop.setSetpoint(percent, ControlType.kVoltage);
  }
}
