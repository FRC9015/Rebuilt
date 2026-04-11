package frc.robot.subsystems.turret;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.sim.CANcoderSimState;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.util.PhoenixUtil.TalonFXMotorControllerSim;

public class TurretIOSim implements TurretIO {

  private final TalonFXMotorControllerSim turretMotorSim;
  private final CANcoderSimState encoder13;
  private final CANcoderSimState encoder15;

  private double setpointDegrees = 0.0;
  private final StatusSignal<Angle> encoder13PosSignal;
  private final StatusSignal<Angle> encoder15PosSignal;
  private final StatusSignal<Voltage> motorAppliedVoltsSignal;
  private final StatusSignal<Current> motorCurrentSignal;
  private final StatusSignal<Angle> motorPositionSignal;

  private final Debouncer encoderConnectedDebounce = new Debouncer(0.5);
  private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
  private final VoltageOut voltageOut = new VoltageOut(0.0);

  private Alert outofSyncAlert =
      new Alert("CRT-Motor desync detected! Resyncing motor.", AlertType.kWarning);
  private Alert outofBoundsAlert =
      new Alert("Turret out of bounds; no longer tracking", AlertType.kError);
  
  private double appliedTurretRotation = 0.0;

  public TurretIOSim(int motorID, int encoderID13, int encoderID15) {
    turretMotorSim = new TalonFXMotorControllerSim(new com.ctre.phoenix6.hardware.TalonFX(motorID), false);
    encoder13 = new CANcoderSimState
    encoder15 = new CANcoderSimState(encoderID15);

    encoder13PosSignal = encoder13.getPosition();
    encoder15PosSignal = encoder15.getPosition();
    motorAppliedVoltsSignal = turretMotorSim.updateControlSignal(Angle.ofDegrees(setpointDegrees), Angle.ofDegrees(0), Angle.ofDegrees(0), Angle.ofDegrees(0));
    motorCurrentSignal = new StatusSignal<Current>() {
      @Override
      public Current getValue() {
        return Current.ofAmps(Math.abs(motorAppliedVoltsSignal.getValueAsDouble()) * 0.1); // Simulate current based on voltage
      }
    };
    motorPositionSignal = new StatusSignal<Angle>() {
      @Override
      public Angle getValue() {
        return Angle.ofDegrees(setpointDegrees); // Simulate position based on setpoint
      }
    };
  }

  public TurretIOInputs inputs = new TurretIOInputs();

  public double getAppliedTurretRotation() {
    return appliedTurretRotation;
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    this.inputs = inputs;
  }

  @Override
  public void stop() {}

  @Override
  public void setBrakeMode() {}

  @Override
  public void setCoastMode() {}

  @Override
  public void setTurretPosition(double value) {
    inputs.turretResolvedPosition = (value);
  }

  @Override
  public void seedMotorPosition(double positionRotations) {}

  @Override
  public void setTurretSetPoint(double value) {
    inputs.turretSetpoint = value;
  }
}
