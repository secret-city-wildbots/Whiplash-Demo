package frc.robot.Actors.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Actors.Motor;
import frc.robot.Utils.MotorType;
import frc.robot.Utils.RotationDir;

public class Indexer extends SubsystemBase {
    public Motor indexerMotor;

    public Indexer() {
        indexerMotor = new Motor(18, MotorType.SPX);

        indexerMotor.motorConfig.direction = RotationDir.CounterClockwise;
        indexerMotor.motorConfig.brake = false;
        indexerMotor.motorConfig.forwardLimitSwitchEnabled = false;
        indexerMotor.motorConfig.reverseLimitSwitchEnabled = false;

        indexerMotor.applyConfig();
    }

    public void shoot() {
        indexerMotor.dc(0.7);
        System.out.println("GOOOOO");
    }

    public void stop() {
        indexerMotor.dc(0.0);
    }
}
