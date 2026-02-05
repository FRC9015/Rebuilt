package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
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
  private final SparkFlex intakeMotor2;

  private final RelativeEncoder intakeEncoder;
  private final RelativeEncoder pivotEncoder;

  private final SparkClosedLoopController intakePID;
  private final SparkClosedLoopController pivotPID;

  private final SparkMaxConfig intakeConfig;
  private final SparkMaxConfig pivotConfig;
  private final SparkMaxConfig motorConfig;

  private final SparkClosedLoopController closedLoopController1, closedLoopController2;

  private static final double maxFreeSpeed = 3000.0; // RPM

  // ---------------- PID CONSTANTS ----------------
  // Intake velocity (RPM)
  private double intakeKp = 0.001;
  private double intakeKi = 0.0;
  private double intakeKd = 0.0;
  private LoggedNetworkBoolean updatePIDs = new LoggedNetworkBoolean("/Intake/UpdatePIDs", false);
  private double intakeKff = 0.0;

  // Pivot position (rotations)
  private LoggedNetworkNumber pivotKp = new LoggedNetworkNumber("/Pivot/P", 0.1);
  private LoggedNetworkNumber pivotKi = new LoggedNetworkNumber("/Pivot/I", 0.0);
  private LoggedNetworkNumber pivotKd = new LoggedNetworkNumber("/Pivot/D", 0.0);

  public IntakeIOSparkFlex(int intakeID, int pivotID) {

    // ---------------- MOTOR INIT ----------------
    intakeMotor = new SparkFlex(intakeID, SparkFlex.MotorType.kBrushless);
    intakeMotor2 = new SparkFlex(pivotID, SparkFlex.MotorType.kBrushless);

    intakeEncoder = intakeMotor.getEncoder();
    pivotEncoder = intakeMotor2.getEncoder();
    motorConfig = new SparkMaxConfig();
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

    intakePID = intakeMotor.getClosedLoopController();
    pivotPID = intakeMotor2.getClosedLoopController();

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
        .pid(intakeKp, intakeKi, intakeKd)
        .velocityFF(intakeKff)
        .outputRange(-1.0, 1.0);

    // ---------------- PIVOT CONFIG ----------------
    pivotConfig.inverted(false).idleMode(IdleMode.kBrake);

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
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intakeMotor2.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    closedLoopController1 = intakeMotor.getClosedLoopController();
    closedLoopController2 = intakeMotor2.getClosedLoopController();
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
    intakeMotor2.stopMotor();
  }

  public boolean updatePIDsIfNeeded() {
    boolean updatePIDs = this.updatePIDs.getAsBoolean();
    return updatePIDs;
  }

  // --------- CONTROL ----------------
  @Override
  public void updatePIDFromDashboard() {
    double intakeP = intakeKp;
    double intakeI = intakeKi;
    double intakeD = intakeKd;

    double pivotP = pivotKp.getAsDouble();
    double pivotI = pivotKi.getAsDouble();
    double pivotD = pivotKd.getAsDouble();

    // Update configs
    intakeConfig.closedLoop.pid(intakeP, intakeI, intakeD).outputRange(-1.0, 1.0);
    pivotConfig.closedLoop.pid(pivotP, pivotI, pivotD).outputRange(-1.0, 1.0);

    // Apply without resetting other parameters
    intakeMotor.configure(
        intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    intakeMotor2.configure(
        intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setIntakeSpeed(double speed) {

    // Velocity control (Percentage)
    closedLoopController1.setSetpoint(-speed, ControlType.kVoltage, ClosedLoopSlot.kSlot0);
    closedLoopController2.setSetpoint(speed, ControlType.kVoltage, ClosedLoopSlot.kSlot0);
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
