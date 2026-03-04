package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class PivotIOSparkFlex implements PivotIO {

  // ---------------- MOTORS ----------------
  private final SparkFlex pivotMotor;

  private final RelativeEncoder pivotLeftEncoder;

  private final SparkClosedLoopController pivotLeftPID;

  private final SparkFlexConfig pivotLeftConfig;

  private static final double maxFreeSpeed = 6784.0; // RPM

  // ---------------- PID CONSTANTS ----------------
  // Pivot position (rotations)
  private LoggedNetworkNumber kp = new LoggedNetworkNumber("/Pivot/P", 1.0);
  private LoggedNetworkNumber ki = new LoggedNetworkNumber("/Pivot/I", 0.0);
  private LoggedNetworkNumber kd = new LoggedNetworkNumber("/Pivot/D", 0.0);

  public PivotIOSparkFlex(int pivotID) {

    // ---------------- MOTOR INIT ----------------
    pivotMotor = new SparkFlex(pivotID, SparkFlex.MotorType.kBrushless);

    pivotLeftEncoder = pivotMotor.getEncoder();

    pivotLeftPID = pivotMotor.getClosedLoopController();

    pivotLeftConfig = new SparkFlexConfig();

    // ---------------- PIVOT CONFIG ----------------
    pivotLeftConfig.idleMode(IdleMode.kBrake);

    pivotLeftConfig
        .encoder
        .positionConversionFactor(1.0) // rotations
        .velocityConversionFactor(1.0);

    pivotLeftConfig
        .closedLoop
        .pid(kp.getAsDouble(), ki.getAsDouble(), kd.getAsDouble())
        .outputRange(-1.0, 1.0);

    // ---------------- APPLY CONFIG ----------------
    pivotMotor.configure(
        pivotLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // ------------------------------------------------
  // IO METHODS
  // ------------------------------------------------

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.pivotLeftApppliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
    inputs.pivotLeftCurrentAmps = pivotMotor.getOutputCurrent();
    inputs.pivotLeftCurrentSpeed = pivotLeftEncoder.getVelocity();
    inputs.pivotPosition = pivotLeftEncoder.getPosition();
  }

  @Override
  public void stop() {
    pivotMotor.stopMotor();
  }

  // --------- CONTROL ----------------
  @Override
  public void updatePIDFromDashboard() {
    double pivotP = kp.getAsDouble();
    double pivotI = ki.getAsDouble();
    double pivotD = kd.getAsDouble();

    // Update configs
    pivotLeftConfig.closedLoop.pid(pivotP, pivotI, pivotD).outputRange(-1.0, 1.0);

    // Apply without resetting other parameters
    pivotMotor.configure(
        pivotLeftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setPivotPosition(double position) {
    pivotLeftPID.setSetpoint(position, ControlType.kPosition);
  }

  @Override
  public void setPivotPosition(PivotPositions position) {
    pivotLeftPID.setSetpoint(position.getPivotPosition(), ControlType.kPosition);
  }
}
