package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IntakeIOSparkFlex implements IntakeIO {

  // ---------------- MOTORS ----------------
  private final SparkFlex intakeMotor;
  private final SparkFlex pivotMotor;

  private final RelativeEncoder intakeEncoder;
  private final RelativeEncoder pivotEncoder;

  private final SparkClosedLoopController intakePID;
  private final SparkClosedLoopController pivotPID;

  private final SparkMaxConfig intakeConfig;
  private final SparkMaxConfig pivotConfig;

  private static final double maxFreeSpeed = 6784.0; // RPM

  // ---------------- PID CONSTANTS ----------------
  // Intake velocity (RPM)
  private LoggedNetworkNumber INTAKE_kP = new LoggedNetworkNumber("/Intake/P", 1.0);
  private LoggedNetworkNumber INTAKE_kI = new LoggedNetworkNumber("/Intake/I", 0.0);
  private LoggedNetworkNumber INTAKE_kD = new LoggedNetworkNumber("/Intake/D", 0.0);
  private LoggedNetworkBoolean updatePIDs = new LoggedNetworkBoolean("/Intake/UpdatePIDs", false);
  private double INTAKE_kFF = 0.0;

  // Pivot position (rotations)
  private LoggedNetworkNumber PIVOT_kP = new LoggedNetworkNumber("/Pivot/P", 1.0);
  private LoggedNetworkNumber PIVOT_kI = new LoggedNetworkNumber("/Pivot/I", 0.0);
  private LoggedNetworkNumber PIVOT_kD = new LoggedNetworkNumber("/Pivot/D", 0.0);

  public IntakeIOSparkFlex(int intakeID, int pivotID) {

    // ---------------- MOTOR INIT ----------------
    intakeMotor = new SparkFlex(intakeID, SparkFlex.MotorType.kBrushless);
    pivotMotor = new SparkFlex(pivotID, SparkFlex.MotorType.kBrushless);

    intakeEncoder = intakeMotor.getEncoder();
    pivotEncoder = pivotMotor.getEncoder();

    intakePID = intakeMotor.getClosedLoopController();
    pivotPID = pivotMotor.getClosedLoopController();

    intakeConfig = new SparkMaxConfig();
    pivotConfig = new SparkMaxConfig();

    // ---------------- INTAKE CONFIG ----------------
    intakeConfig.inverted(false).idleMode(IdleMode.kBrake);

    intakeConfig
        .encoder
        .positionConversionFactor(1.0) // rotations
        .velocityConversionFactor(1.0); // RPM

    intakeConfig
        .closedLoop
        .pid(INTAKE_kP.getAsDouble(), INTAKE_kI.getAsDouble(), INTAKE_kD.getAsDouble())
        .velocityFF(INTAKE_kFF)
        .outputRange(-1.0, 1.0);

    // ---------------- PIVOT CONFIG ----------------
    pivotConfig.inverted(true).idleMode(IdleMode.kBrake);

    pivotConfig
        .encoder
        .positionConversionFactor(1.0) // rotations
        .velocityConversionFactor(1.0);

    pivotConfig
        .closedLoop
        .pid(PIVOT_kP.getAsDouble(), PIVOT_kI.getAsDouble(), PIVOT_kD.getAsDouble())
        .outputRange(-1.0, 1.0);

    // ---------------- APPLY CONFIG ----------------
    intakeMotor.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivotMotor.configure(
        pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // ------------------------------------------------
  // IO METHODS
  // ------------------------------------------------

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeAppliedVolts = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
    inputs.intakeCurrentAmps = intakeMotor.getOutputCurrent();
    inputs.intakeRPM = intakeEncoder.getVelocity();
    inputs.intakeCurrentSpeed = intakeEncoder.getVelocity();
    inputs.intakeEncoderPosition = pivotEncoder.getPosition();
    inputs.intakeEncoderConnected = true;
  }

  @Override
  public void stop() {
    intakeMotor.stopMotor();
    pivotMotor.stopMotor();
  }

  public boolean updatePIDsIfNeeded() {
    boolean updatePIDs = this.updatePIDs.getAsBoolean();
    return updatePIDs;
  }

  // --------- CONTROL ----------------
  @Override
  public void updatePIDFromDashboard() {
    double intakeP = INTAKE_kP.getAsDouble();
    double intakeI = INTAKE_kI.getAsDouble();
    double intakeD = INTAKE_kD.getAsDouble();

    double pivotP = PIVOT_kP.getAsDouble();
    double pivotI = PIVOT_kI.getAsDouble();
    double pivotD = PIVOT_kD.getAsDouble();

    // Update configs
    intakeConfig.closedLoop.pid(intakeP, intakeI, intakeD).outputRange(-1.0, 1.0);
    pivotConfig.closedLoop.pid(pivotP, pivotI, pivotD).outputRange(-1.0, 1.0);

    // Apply without resetting other parameters
    intakeMotor.configure(
        intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    pivotMotor.configure(
        pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setIntakeSpeed(double speed) {

    double clamped = MathUtil.clamp(speed, 0.01, 1.0);

    // Velocity control (RPM)
    intakePID.setSetpoint(clamped * maxFreeSpeed, ControlType.kVelocity);
  }

  @Override
  public void setIntakePosition(double position) {

    double clamped = MathUtil.clamp(position, 0.01, 1.0);

    // Position control (rotations)
    pivotPID.setSetpoint(clamped * maxFreeSpeed, ControlType.kVelocity);
  }

  @Override
  public double getVelocity() {
    return intakeEncoder.getVelocity();
  }

  public double getPosition() {
    return pivotEncoder.getPosition();
  }
}
