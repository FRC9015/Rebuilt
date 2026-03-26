package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower; // Added Import
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.IntakeConstants;

public class PivotIOTalonFX implements PivotIO {
  public final TalonFX pivotMotorLeft;
  public final TalonFX pivotMotorRight;

  public final CANcoder pivotEncoder;
  public StatusSignal<Voltage> pivotLeftVolts;
  public StatusSignal<Current> pivotLeftAmps;
  public StatusSignal<AngularVelocity> pivotLeftVelocity;

  public StatusSignal<Voltage> pivotRightVolts;
  public StatusSignal<Current> pivotRightAmps;
  public StatusSignal<AngularVelocity> pivotRightVelocity;
  public StatusSignal<Angle> pivotPosition;
  private final MotionMagicExpoVoltage pivotMagicVoltage = new MotionMagicExpoVoltage(0.0);

  public PivotIOTalonFX(int pivotIDLeft, int pivotIDRight, int encoderID) {
    pivotMotorLeft = new TalonFX(pivotIDLeft);
    pivotMotorRight = new TalonFX(pivotIDRight);
    pivotEncoder = new CANcoder(encoderID);

    pivotMotorLeft.getConfigurator().apply(IntakeConstants.pivotConfigLeft);
    pivotMotorRight.getConfigurator().apply(IntakeConstants.pivotConfigRight);

    // Set the Right motor to follow the Left motor and spin in the opposite direction
    pivotMotorRight.setControl(new Follower(pivotIDLeft, MotorAlignmentValue.Opposed));

    pivotLeftVolts = pivotMotorLeft.getMotorVoltage();
    pivotRightVolts = pivotMotorRight.getMotorVoltage();
    pivotLeftAmps = pivotMotorLeft.getStatorCurrent();
    pivotRightAmps = pivotMotorRight.getStatorCurrent();
    pivotLeftVelocity = pivotMotorLeft.getVelocity();
    pivotRightVelocity = pivotMotorRight.getVelocity();
    pivotPosition = pivotEncoder.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        pivotLeftVolts,
        pivotLeftAmps,
        pivotLeftVelocity,
        pivotRightVolts,
        pivotRightAmps,
        pivotRightVelocity,
        pivotPosition);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        pivotLeftVolts,
        pivotRightVolts,
        pivotLeftAmps,
        pivotRightAmps,
        pivotLeftVelocity,
        pivotRightVelocity,
        pivotPosition);
    inputs.pivotLeftAppliedVolts = pivotLeftVolts.getValueAsDouble();
    inputs.pivotRightAppliedVolts = pivotRightVolts.getValueAsDouble();
    inputs.pivotLeftCurrentSpeed = pivotLeftVelocity.getValueAsDouble();
    inputs.pivotRightCurrentSpeed = pivotRightVelocity.getValueAsDouble();
    inputs.pivotLeftCurrentAmps = pivotLeftAmps.getValueAsDouble();
    inputs.pivotRightCurrentAmps = pivotRightAmps.getValueAsDouble();
    inputs.pivotPosition = pivotPosition.getValueAsDouble();
  }

  @Override
  public void stop() {
    // Only need to stop the master; the follower will follow
    pivotMotorLeft.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean enable) {
    pivotMotorLeft.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    pivotMotorRight.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setPivotPosition(double position) {
    final double clampedPosition =
        MathUtil.clamp(position, IntakeConstants.INTAKE_MIN_POS, IntakeConstants.INTAKE_MAX_POS);

    // Only set control on the master; the follower handles the rest
    pivotMotorLeft.setControl(pivotMagicVoltage.withPosition(clampedPosition));
  }

  @Override
  public void setPivotPosition(PivotPositions position) {
    // Only set control on the master; the follower handles the rest
    pivotMotorLeft.setControl(pivotMagicVoltage.withPosition(position.getPivotPosition()));
  }
}
