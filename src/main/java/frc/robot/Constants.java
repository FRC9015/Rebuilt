// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class motorIDConstants {
    // placeholders
    public static final int INTAKE_ROLLER_LEFT_ID = 0;
    public static final int INTAKE_ROLLER_RIGHT_ID = 0;
    public static final int INTAKE_PIVOT_LEFT_ID = 0;
    public static final int INTAKE_PIVOT_RIGHT_ID = 0;

    public static final int EXTEND_INTAKE_MOTOR_ID = 0;
  }

  public static class intakeConstants {
    public static final Slot0Configs PIVOT_SLOT0_CONFIGS =
        new Slot0Configs()
            .withKP(2)
            .withKI(0)
            .withKD(0.05)
            .withKG(0.01)
            .withKA(0)
            .withKS(0)
            .withKV(0);
    public static final Slot0Configs ROLLER_SLOT0_CONFIGS =
        new Slot0Configs().withKP(2).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);

    public static final MotionMagicConfigs PIVOT_MAGIC_CONFIGS =
        new MotionMagicConfigs().withMotionMagicAcceleration(100).withMotionMagicCruiseVelocity(25);

    public static final MotionMagicConfigs ROLLER_MAGIC_CONFIGS =
        new MotionMagicConfigs().withMotionMagicAcceleration(50).withMotionMagicCruiseVelocity(5);

    public static final FeedbackConfigs PIVOT_FEEDBACK_CONFIGS =
        new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    private static final MotorOutputConfigs rollerOutputLeftConfigs =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

    public static final TalonFXConfiguration rollerConfigLeft =
        new TalonFXConfiguration()
            .withMotorOutput(rollerOutputLeftConfigs)
            .withSlot0(ROLLER_SLOT0_CONFIGS)
            .withMotionMagic(ROLLER_MAGIC_CONFIGS);

    private static final MotorOutputConfigs rollerOutputRightConfigs =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

    public static final TalonFXConfiguration rollerConfigRight =
        new TalonFXConfiguration()
            .withMotorOutput(rollerOutputRightConfigs)
            .withSlot0(ROLLER_SLOT0_CONFIGS)
            .withMotionMagic(ROLLER_MAGIC_CONFIGS);

    private static final MotorOutputConfigs pivotOutputLeftConfigs =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
    public static final TalonFXConfiguration pivotConfigLeft =
        new TalonFXConfiguration()
            .withSlot0(Constants.intakeConstants.PIVOT_SLOT0_CONFIGS)
            .withFeedback(Constants.intakeConstants.PIVOT_FEEDBACK_CONFIGS)
            .withMotionMagic(Constants.intakeConstants.PIVOT_MAGIC_CONFIGS)
            .withMotorOutput(pivotOutputLeftConfigs);

    private static final MotorOutputConfigs pivotOutputRightConfigs =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
    public static final TalonFXConfiguration pivotConfigRight =
        new TalonFXConfiguration()
            .withSlot0(Constants.intakeConstants.PIVOT_SLOT0_CONFIGS)
            .withFeedback(Constants.intakeConstants.PIVOT_FEEDBACK_CONFIGS)
            .withMotionMagic(Constants.intakeConstants.PIVOT_MAGIC_CONFIGS)
            .withMotorOutput(pivotOutputRightConfigs);

    public static final double INTAKE_MAX_POS = 300.0;
    public static final double INTAKE_MIN_POS = 0.0;
    public static final double INTAKE_MAX_SPEED = 512.0;
    public static final double INTAKE_MIN_SPEED = -511.0;

    public static final double INTAKE_DEPLOYED_POSITION = 100.0;
    public static final double INTAKE_STOWED_POSITION = 10.0;
  }
}
