package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public double turretAngle = 0.0;
        public double turretVelocity = 0.0;
        public boolean turretEncoderConnected = false;
        public double turretAppliedVolts = 0.0;
    }

    public default void updateInputs(TurretIOInputs inputs) {

    }

    public default void stop() {

    }

    

}
