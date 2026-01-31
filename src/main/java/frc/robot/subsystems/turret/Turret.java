package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

    private final TurretIO io;

    public Turret(TurretIO io) {
        this.io = io;
    }
    
}
