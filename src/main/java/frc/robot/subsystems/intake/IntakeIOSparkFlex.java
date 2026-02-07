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
import com.revrobotics.spark.config.SparkFlexConfig;

public class IntakeIOSparkFlex implements IntakeIO {

  private final SparkFlex intakeMotor;
  private final RelativeEncoder intakeEncoder;
  private final SparkFlex intakeMotor2;
  private final SparkFlexConfig intakeConfig;
  private final SparkClosedLoopController closedLoopController1, closedLoopController2;

  // ---------------- PID CONSTANTS ----------------
  // Intake velocity (RPM)
  private final double intakeP = 0.001;
  private final double intakeI = 0.0;
  private final double intakeD = 0.02;

  public IntakeIOSparkFlex(int intakeID, int intakeID2) {

    // ---------------- MOTOR INIT ----------------
    intakeMotor = new SparkFlex(intakeID, SparkFlex.MotorType.kBrushless);
    intakeMotor2 = new SparkFlex(intakeID2, SparkFlex.MotorType.kBrushless);

    intakeEncoder = intakeMotor.getEncoder();
    intakeConfig = new SparkFlexConfig();
    intakeConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(intakeP)
        .i(intakeI)
        .d(intakeD)
        .outputRange(-1, 1);

    // ---------------- INTAKE CONFIG ----------------
    intakeConfig.inverted(false).idleMode(IdleMode.kBrake);

    // ---------------- APPLY CONFIG ----------------
    intakeMotor.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotor2.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
    inputs.intakeEncoderConnected = true;
  }

  @Override
  public void stop() {
    intakeMotor.stopMotor();
    intakeMotor2.stopMotor();
  }

  @Override
  public void setIntakeSpeed(double speed) {

    // Velocity control (Percentage)
    closedLoopController1.setSetpoint(-speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    closedLoopController2.setSetpoint(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setIntakePosition(double position) {}

  @Override
  public double getVelocity() {
    return intakeEncoder.getVelocity();
  }
}
