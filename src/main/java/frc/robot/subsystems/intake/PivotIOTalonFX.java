package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.intakeConstants;

public class PivotIOTalonFX implements PivotIO {
  public final TalonFX pivotMotorLeft;
  public final TalonFX pivotMotorRight;

  public StatusSignal<Voltage> pivotLeftVolts;
  public StatusSignal<Current> pivotLeftAmps;
  public StatusSignal<AngularVelocity> pivotLeftVelocity;
  public StatusSignal<Angle> pivotLeftPosition;
  private final MotionMagicExpoVoltage pivotMagicVoltage = new MotionMagicExpoVoltage(0.0);

  public PivotIOTalonFX(int pivotIDLeft, int pivotIDRight) {
    pivotMotorLeft = new TalonFX(pivotIDLeft);
    pivotMotorRight = new TalonFX(pivotIDRight);

    pivotMotorLeft.getConfigurator().apply(intakeConstants.pivotConfigLeft);
    pivotMotorRight.getConfigurator().apply(intakeConstants.pivotConfigRight);

    pivotLeftVolts = pivotMotorLeft.getMotorVoltage();
    pivotLeftAmps = pivotMotorLeft.getStatorCurrent();
    pivotLeftVelocity = pivotMotorLeft.getVelocity();
    pivotLeftPosition = pivotMotorLeft.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, pivotLeftVolts, pivotLeftAmps, pivotLeftVelocity, pivotLeftPosition);

    ParentDevice.optimizeBusUtilizationForAll(pivotMotorLeft);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {

    BaseStatusSignal.refreshAll(
        pivotLeftVolts, pivotLeftAmps, pivotLeftVelocity, pivotLeftPosition);
    inputs.pivotApppliedVolts = pivotLeftVolts.getValueAsDouble();
    inputs.pivotPosition = pivotLeftPosition.getValueAsDouble();
    inputs.pivotCurrentSpeed = pivotLeftVelocity.getValueAsDouble();
    inputs.pivotCurrentAmps = pivotLeftAmps.getValueAsDouble();
  }

  @Override
  public void stop() {
    pivotMotorLeft.stopMotor();
  }

  @Override
  public void updatePIDFromDashboard() {
    // Implement PID update logic if needed
  }

  @Override
  public void setBrakeMode(boolean enable) {
    pivotMotorLeft.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setPivotPosition(double position) {

    final double clampedPosition =
        MathUtil.clamp(
            position,
            Constants.intakeConstants.INTAKE_MIN_POS,
            Constants.intakeConstants.INTAKE_MAX_POS);

    pivotMotorLeft.setControl(pivotMagicVoltage.withPosition(clampedPosition));
    pivotMotorRight.setControl(pivotMagicVoltage.withPosition(clampedPosition));
  }

  @Override
  public void setPivotPosition(PivotPositions position) {
    pivotMotorLeft.setControl(pivotMagicVoltage.withPosition(position.getPivotPosition()));
    pivotMotorRight.setControl(pivotMagicVoltage.withPosition(position.getPivotPosition()));
  }
}
