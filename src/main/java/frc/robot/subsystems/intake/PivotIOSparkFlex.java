package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class PivotIOSparkFlex implements PivotIO {

  // ---------------- MOTORS ----------------
  private final SparkFlex pivotLeft;
  private final SparkFlex pivotRight;

  private final RelativeEncoder pivotLeftEncoder;
  private final RelativeEncoder pivotRightEncoder;

  private final SparkClosedLoopController pivotLeftPID;
  private final SparkClosedLoopController pivotRightPID;

  private final SparkMaxConfig pivotLeftConfig;
  private final SparkMaxConfig pivotRightConfig;

  private static final double maxFreeSpeed = 6784.0; // RPM

  // ---------------- PID CONSTANTS ----------------
  // Pivot position (rotations)
  private LoggedNetworkNumber kp = new LoggedNetworkNumber("/Pivot/P", 1.0);
  private LoggedNetworkNumber ki = new LoggedNetworkNumber("/Pivot/I", 0.0);
  private LoggedNetworkNumber kd = new LoggedNetworkNumber("/Pivot/D", 0.0);

  public PivotIOSparkFlex(int pivotIDLeft, int pivotIDRight) {

    // ---------------- MOTOR INIT ----------------
    pivotLeft = new SparkFlex(pivotIDLeft, SparkFlex.MotorType.kBrushless);
    pivotRight = new SparkFlex(pivotIDRight, SparkFlex.MotorType.kBrushless);

    pivotLeftEncoder = pivotLeft.getEncoder();
    pivotRightEncoder = pivotRight.getEncoder();

    pivotLeftPID = pivotLeft.getClosedLoopController();
    pivotRightPID = pivotRight.getClosedLoopController();

    pivotLeftConfig = new SparkMaxConfig();
    pivotRightConfig = new SparkMaxConfig();

    // ---------------- PIVOT CONFIG ----------------
    pivotLeftConfig.idleMode(IdleMode.kBrake);
    pivotRightConfig.inverted(true).idleMode(IdleMode.kBrake);

    pivotLeftConfig
        .encoder
        .positionConversionFactor(1.0) // rotations
        .velocityConversionFactor(1.0);

    pivotLeftConfig
        .closedLoop
        .pid(kp.getAsDouble(), ki.getAsDouble(), kd.getAsDouble())
        .outputRange(-1.0, 1.0);

    pivotRightConfig
        .encoder
        .positionConversionFactor(1.0) // rotations
        .velocityConversionFactor(1.0);

    pivotRightConfig
        .closedLoop
        .pid(kp.getAsDouble(), ki.getAsDouble(), kd.getAsDouble())
        .outputRange(-1.0, 1.0);
    // ---------------- APPLY CONFIG ----------------
    pivotLeft.configure(
        pivotLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivotRight.configure(
        pivotRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // ------------------------------------------------
  // IO METHODS
  // ------------------------------------------------

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.pivotLeftApppliedVolts = pivotLeft.getAppliedOutput() * pivotLeft.getBusVoltage();
    inputs.pivotLeftCurrentAmps = pivotLeft.getOutputCurrent();
    inputs.pivotLeftCurrentSpeed = pivotLeftEncoder.getVelocity();
    inputs.pivotRightApppliedVolts = pivotRight.getAppliedOutput() * pivotRight.getBusVoltage();
    inputs.pivotRightCurrentAmps = pivotRight.getOutputCurrent();
    inputs.pivotRightCurrentSpeed = pivotRightEncoder.getVelocity();
    inputs.pivotPosition = pivotLeftEncoder.getPosition();
  }

  @Override
  public void stop() {
    pivotLeft.stopMotor();
    pivotRight.stopMotor();
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
    pivotLeft.configure(
        pivotLeftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    pivotRight.configure(
        pivotRightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setPivotPosition(double position) {
    pivotLeftPID.setSetpoint(position, ControlType.kPosition);
    pivotRightPID.setSetpoint(position, ControlType.kPosition);
  }

  @Override
  public void setPivotPosition(PivotPositions position) {
    pivotLeftPID.setSetpoint(position.getPivotPosition(), ControlType.kPosition);
    pivotRightPID.setSetpoint(position.getPivotPosition(), ControlType.kPosition);
  }
}
