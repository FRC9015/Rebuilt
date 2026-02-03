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
  private LoggedNetworkNumber intakeKp = new LoggedNetworkNumber("/Intake/P", 1.0);
  private LoggedNetworkNumber intakeKi = new LoggedNetworkNumber("/Intake/I", 0.0);
  private LoggedNetworkNumber intakeKd = new LoggedNetworkNumber("/Intake/D", 0.0);
  private LoggedNetworkBoolean updatePIDs = new LoggedNetworkBoolean("/Intake/UpdatePIDs", false);
  private double intakeKff = 0.0;

  // Pivot position (rotations)
  private LoggedNetworkNumber pivotKp = new LoggedNetworkNumber("/Pivot/P", 1.0);
  private LoggedNetworkNumber pivotKi = new LoggedNetworkNumber("/Pivot/I", 0.0);
  private LoggedNetworkNumber pivotKd = new LoggedNetworkNumber("/Pivot/D", 0.0);

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
        .pid(intakeKp.getAsDouble(), intakeKi.getAsDouble(), intakeKd.getAsDouble())
        .velocityFF(intakeKff)
        .outputRange(-1.0, 1.0);

    // ---------------- PIVOT CONFIG ----------------
    pivotConfig.inverted(true).idleMode(IdleMode.kBrake);

    pivotConfig
        .encoder
        .positionConversionFactor(1.0) // rotations
        .velocityConversionFactor(1.0);

    pivotConfig
        .closedLoop
        .pid(pivotKp.getAsDouble(), pivotKi.getAsDouble(), pivotKd.getAsDouble())
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
    double intakeP = intakeKp.getAsDouble();
    double intakeI = intakeKi.getAsDouble();
    double intakeD = intakeKd.getAsDouble();

    double pivotP = pivotKp.getAsDouble();
    double pivotI = pivotKi.getAsDouble();
    double pivotD = pivotKd.getAsDouble();

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
