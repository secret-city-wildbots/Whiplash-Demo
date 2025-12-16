package frc.robot.Actors.Subsystems;
import frc.robot.Actors.Motor;
import frc.robot.Utils.MotorType;
import frc.robot.Utils.RotationDir;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    public double gearRatio = 98.0;
    public Motor motor;

    public Wrist(){
        motor= new Motor(14, MotorType.TFX);
        motor.motorConfig.direction = RotationDir.Clockwise;
        motor.pid(0.11, 0.015, 0.025);
    }

    public void goToPos(double angle) {
        motor.pos(angle);
    }
}
