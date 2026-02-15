// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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
    public static final int UPPER_INTAKE_MOTOR_ID = 0;
    public static final int EXTEND_INTAKE_MOTOR_ID = 0;
    public static final int TURRET_MOTOR_ID = 0;
  }

  public static class intakeConstants {
    public static final Slot0Configs intakeSlotPositionConfigs =
        new Slot0Configs()
            .withKP(2)
            .withKI(0)
            .withKD(0.05)
            .withKG(0.01)
            .withKA(0)
            .withKS(0)
            .withKV(0);

    public static final Slot1Configs intakeSlotVelocityConfigs =
        new Slot1Configs().withKP(2).withKI(0).withKD(0).withKG(0).withKA(0).withKS(0).withKV(0);

    public static final MotionMagicConfigs GROUND_MAGIC_CONFIGS =
        new MotionMagicConfigs().withMotionMagicAcceleration(100).withMotionMagicCruiseVelocity(25);

    public static final FeedbackConfigs GROUND_FEEDBACK_CONFIGS =
        new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    public static final double INTAKE_MAX_POS = 300.0;
    public static final double INTAKE_MIN_POS = 0.0;
    public static final double INTAKE_MAX_SPEED = 512.0;
    public static final double INTAKE_MIN_SPEED = -511.0;

    public static final double INTAKE_DEPLOYED_POSITION = 100.0;
    public static final double INTAKE_STOWED_POSITION = 10.0;
  }

  public static class turretConstants {
    // --- GEAR TEETH ---
    public static final int T_TEETH = 90;
    public static final int E1_TEETH = 13; // Gear on Encoder 1
    public static final int E2_TEETH = 15; // Gear on Encoder 2

    public static final int ENCODER_13_TOOTH = 35; // Encoder 13 motor id
    public static final int ENCODER_15_TOOTH = 36; // Encoder 15 motor id

    // --- MATH CONSTANTS ---
    /** The error allowance (in turret rotations) when comparing encoder predictions. */
    public static final double CRT_TOLERANCE = 0.01;

    /**
     * The difference threshold between calculated and internal motor position to trigger a re-seed.
     */
    public static final double SYNC_THRESHOLD = 0.05;

    /** Search limit for Encoder 1 (should be equal to e2_teeth). */
    public static final int E1_SEARCH_LIMIT = (int) E2_TEETH;
    /** Search limit for Encoder 2 (should be equal to e1_teeth). */
    public static final int E2_SEARCH_LIMIT = (int) E1_TEETH;

    // --- MOVEMENT LIMITS ---
    public static final double MAXROTATION = 2.0;
    public static final double MINROTATION = 0.0;

    public static final double MOTOR_TO_TURRET_GEAR_RATIO = 37.5;
    // --- MOTOR CONFIGS ---
    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS =
        new MotionMagicConfigs().withMotionMagicAcceleration(150).withMotionMagicCruiseVelocity(50);
    public static final Slot0Configs SLOT0_CONFIGS =
        new Slot0Configs().withKP(1).withKI(0).withKD(0).withKG(0).withKA(0).withKS(0).withKV(0);
    public static final FeedbackConfigs FEEDBACK_CONFIGS =
        new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
            .withSensorToMechanismRatio(MOTOR_TO_TURRET_GEAR_RATIO);
  }
}
