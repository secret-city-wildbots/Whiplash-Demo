package frc.robot.Actors.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Actors.Motor;
import frc.robot.Utils.MotorType;
import frc.robot.Utils.RotationDir;

public class Shooter extends SubsystemBase {
    public Motor shooterMotorLeft;
    public Motor shooterMotorRight;

    public Shooter() {
        shooterMotorLeft = new Motor(16, MotorType.TFX);
        shooterMotorRight = new Motor(15, MotorType.TFX);

        shooterMotorLeft.motorConfig.direction = RotationDir.Clockwise;
        shooterMotorRight.motorConfig.direction = RotationDir.CounterClockwise;
        shooterMotorLeft.motorConfig.brake = false;
        shooterMotorRight.motorConfig.brake = false;

        shooterMotorLeft.applyConfig();
        shooterMotorRight.applyConfig();
    }

    public void spinUp(double power) {
        shooterMotorLeft.dc(power);
        shooterMotorRight.dc(power * 0.8);
    }

    public void stop() {
        shooterMotorLeft.dc(0);
        shooterMotorRight.dc(0);
    }
}