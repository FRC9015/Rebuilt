package frc.robot.subsystems.indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;

/** Indexer IO implementation using REV SPARK MAX + NEO (REVLib 2026 style). */
public class IndexerIOSparkMax implements IndexerIO {

  private final SparkMax motor1;
  private final RelativeEncoder encoder;

  private final double maxSpeed = 6784.0;

  private final SparkClosedLoopController indexerPIDController;

  private final Debouncer encoderConnectedDebounce = new Debouncer(0.5);

  public IndexerIOSparkMax(int motorId1) {
    motor1 = new SparkMax(motorId1, MotorType.kBrushless);
    encoder = motor1.getEncoder();

    // Build a declarative config (REVLib 2026)
    SparkMaxConfig cfg = new SparkMaxConfig();

    cfg.idleMode(SparkBaseConfig.IdleMode.kBrake);
    cfg.inverted(false); // flip to true if your indexer spins backwards
    cfg.smartCurrentLimit(45); // amps
    cfg.voltageCompensation(12.0); // consistent voltage behavior
    cfg.closedLoop.pid(6, 7, 21);

    // Apply config:
    // - Reset safe params first (gets you to a known baseline)
    // - Don't persist to flash every boot (saves flash wear + avoids config delays)
    motor1.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    indexerPIDController = motor1.getClosedLoopController();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // REV RelativeEncoder velocity is RPM by default
    inputs.IndexerRPM = encoder.getVelocity();

    // Applied volts = applied output (-1..1) * bus voltage
    inputs.IndexerAppliedVolts = motor1.getAppliedOutput() * motor1.getBusVoltage();

    // Output current in amps
    inputs.IndexerCurrentAmps = motor1.getOutputCurrent();

    // "Connected" proxy: REV doesn't have Phoenix-style encoder status signals.
    // Best practical proxy is "last error OK" + bus voltage exists.
    boolean commsOk = (motor1.getLastError() == REVLibError.kOk) && (motor1.getBusVoltage() > 1.0);

    inputs.IndexerEncoderConnected = encoderConnectedDebounce.calculate(commsOk);
  }

  @Override
  public void stop() {
    motor1.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean enable) {
    SparkMaxConfig cfg = new SparkMaxConfig();
    cfg.idleMode(enable ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
    motor1.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setPercentage(double rpm) {
    // Same as your CTRE code: this is open-loop voltage, name is legacy
    indexerPIDController.setSetpoint(rpm*maxSpeed, ControlType.kVelocity);
  }
}
