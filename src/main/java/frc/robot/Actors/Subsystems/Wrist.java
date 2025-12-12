package frc.robot.Actors.Subsystems;
import frc.robot.Actors.Motor;
import frc.robot.Utils.MotorType;
import frc.robot.Utils.RotationDir;
public class Wrist {
    public double gearRatio = 98.0;
    public Motor motor;
    public Wrist(){
        motor= new Motor(14, MotorType.TFX);
        motor.pid(0.11, 0.015, 0.025);
        motor.motorConfig.direction = RotationDir.Clockwise;
    }
public void goToPos(double angle){
    motor.pos(angle);
}


    
}
