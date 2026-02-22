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

public class RollerIOSparkFlex implements RollerIO {

  // ---------------- MOTORS ----------------
  private final SparkFlex rollerLeft;
  private final SparkFlex rollerRight;

  private final RelativeEncoder rollerLeftEncoder;
  private final RelativeEncoder rollerRightEncoder;

  private final SparkClosedLoopController rollerLeftPID;
  private final SparkClosedLoopController rollerrightPID;

  private final SparkMaxConfig rollerLeftConfig;
  private final SparkMaxConfig rollerRightConfig;

  private static final double maxFreeSpeed = 6784.0; // RPM

  // ---------------- PID CONSTANTS ----------------
  // Intake velocity (RPM)
  private LoggedNetworkNumber INTAKE_kP = new LoggedNetworkNumber("/Intake/P", 1.0);
  private LoggedNetworkNumber INTAKE_kI = new LoggedNetworkNumber("/Intake/I", 0.0);
  private LoggedNetworkNumber INTAKE_kD = new LoggedNetworkNumber("/Intake/D", 0.0);
  private LoggedNetworkBoolean updatePIDs = new LoggedNetworkBoolean("/Intake/UpdatePIDs", false);

  public RollerIOSparkFlex(int rollerLeftID, int rollerRightID) {

    // ---------------- MOTOR INIT ----------------
    rollerLeft = new SparkFlex(rollerLeftID, SparkFlex.MotorType.kBrushless);
    rollerRight = new SparkFlex(rollerRightID, SparkFlex.MotorType.kBrushless);

    rollerLeftEncoder = rollerLeft.getEncoder();
    rollerRightEncoder = rollerRight.getEncoder();

    rollerLeftPID = rollerLeft.getClosedLoopController();
    rollerrightPID = rollerRight.getClosedLoopController();

    rollerLeftConfig = new SparkMaxConfig();
    rollerRightConfig = new SparkMaxConfig();

    rollerLeftConfig.idleMode(IdleMode.kBrake);
    rollerRightConfig.inverted(true).idleMode(IdleMode.kBrake);

    rollerLeftConfig
        .encoder
        .positionConversionFactor(1.0) // rotations
        .velocityConversionFactor(1.0); // RPM

    rollerLeftConfig
        .closedLoop
        .pid(INTAKE_kP.getAsDouble(), INTAKE_kI.getAsDouble(), INTAKE_kD.getAsDouble())
        .outputRange(-1.0, 1.0);

    rollerRightConfig
        .encoder
        .positionConversionFactor(1.0) // rotations
        .velocityConversionFactor(1.0); // RPM

    rollerRightConfig
        .closedLoop
        .pid(INTAKE_kP.getAsDouble(), INTAKE_kI.getAsDouble(), INTAKE_kD.getAsDouble())
        .outputRange(-1.0, 1.0);

    // ---------------- APPLY CONFIG ----------------
    rollerLeft.configure(
        rollerLeftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rollerRight.configure(
        rollerRightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  // ------------------------------------------------
  // IO METHODS
  // ------------------------------------------------

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.rollerLeftAppliedVolts = rollerLeft.getAppliedOutput() * rollerLeft.getBusVoltage();
    inputs.rollerLeftCurrentSpeed = rollerLeftEncoder.getVelocity();
    inputs.rollerLeftCurentAmps = rollerLeft.getOutputCurrent();
    inputs.rollerRightAppliedVolts = rollerRight.getAppliedOutput() * rollerRight.getBusVoltage();
    inputs.rollerRightCurrentSpeed = rollerRightEncoder.getVelocity();
    inputs.rollerRightCurentAmps = rollerRight.getOutputCurrent();
  }

  @Override
  public void stop() {
    rollerLeft.stopMotor();
    rollerRight.stopMotor();
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

    // Update configs
    rollerLeftConfig.closedLoop.pid(intakeP, intakeI, intakeD).outputRange(-1.0, 1.0);
    rollerRightConfig.closedLoop.pid(intakeP, intakeI, intakeD).outputRange(-1.0, 1.0);

    // Apply without resetting other parameters
    rollerLeft.configure(
        rollerLeftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rollerRight.configure(
        rollerRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setRollerSpeed(double speed) {

    double clamped = MathUtil.clamp(speed, 0.01, 1.0);

    // Velocity control (RPM)
    rollerLeftPID.setSetpoint(clamped * maxFreeSpeed, ControlType.kVelocity);
    rollerrightPID.setSetpoint(clamped * maxFreeSpeed, ControlType.kVelocity);
  }
}
