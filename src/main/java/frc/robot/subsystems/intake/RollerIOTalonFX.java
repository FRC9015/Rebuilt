package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.IntakeConstants;

public class RollerIOTalonFX implements RollerIO {

  public final TalonFX rollerMotorLeft;
  public final TalonFX rollerMotorRight;

  public StatusSignal<Voltage> rollerLeftVolts;
  public StatusSignal<Current> rollerLeftAmps;
  public StatusSignal<AngularVelocity> rollerLeftspeed;

  public StatusSignal<Voltage> rollerRightVolts;
  public StatusSignal<Current> rollerRightAmps;
  public StatusSignal<AngularVelocity> rollerRightSpeed;

  private final VelocityVoltage intakeVelocityVoltage = new VelocityVoltage(0.0);

  public RollerIOTalonFX(int rollerIDLeft, int rollerIDRight) {
    rollerMotorLeft = new TalonFX(rollerIDLeft);
    rollerMotorRight = new TalonFX(rollerIDRight);

    rollerMotorLeft.getConfigurator().apply(IntakeConstants.rollerConfigLeft);
    rollerMotorRight.getConfigurator().apply(IntakeConstants.rollerConfigRight);

    rollerLeftVolts = rollerMotorLeft.getMotorVoltage();
    rollerLeftAmps = rollerMotorLeft.getStatorCurrent();
    rollerLeftspeed = rollerMotorLeft.getVelocity();

    rollerRightVolts = rollerMotorRight.getMotorVoltage();
    rollerRightAmps = rollerMotorRight.getStatorCurrent();
    rollerRightSpeed = rollerMotorRight.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, rollerLeftVolts, rollerLeftAmps, rollerLeftspeed);

    ParentDevice.optimizeBusUtilizationForAll(rollerMotorLeft, rollerMotorRight);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {

    BaseStatusSignal.refreshAll(rollerLeftVolts, rollerLeftAmps, rollerLeftspeed);

    inputs.rollerLeftAppliedVolts = rollerLeftVolts.getValueAsDouble();
    inputs.rollerLeftCurrentSpeed = rollerLeftspeed.getValueAsDouble();
    inputs.rollerLeftCurentAmps = rollerLeftAmps.getValueAsDouble();
    inputs.rollerRightAppliedVolts = rollerRightVolts.getValueAsDouble();
    inputs.rollerRightCurrentSpeed = rollerRightSpeed.getValueAsDouble();
    inputs.rollerRightCurentAmps = rollerRightAmps.getValueAsDouble();
  }

  @Override
  public void stop() {
    rollerMotorLeft.stopMotor();
    rollerMotorRight.stopMotor();
  }

  @Override
  public void updatePIDFromDashboard() {
    // Implement PID update logic if needed
  }

  @Override
  public void setBrakeMode(boolean enable) {
    rollerMotorLeft.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    rollerMotorRight.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  // Minimum Value of speedValue: -512.0
  // Maximum Value of speedValkue: 511.998046875
  // Unit of output: RPS
  @Override
  public void setRollerSpeed(double speed) {
    rollerMotorLeft.setControl(
        intakeVelocityVoltage.withVelocity(
            MathUtil.clamp(
                speed, IntakeConstants.INTAKE_MIN_SPEED, IntakeConstants.INTAKE_MAX_SPEED)));
    rollerMotorRight.setControl(
        intakeVelocityVoltage.withVelocity(
            MathUtil.clamp(
                speed, IntakeConstants.INTAKE_MIN_SPEED, IntakeConstants.INTAKE_MAX_SPEED)));
  }
}
