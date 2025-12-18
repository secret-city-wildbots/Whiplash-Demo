package frc.robot.Actors.Subsystems;
import frc.robot.Actors.Motor;
import frc.robot.Utils.MotorType;
import frc.robot.Utils.RotationDir;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    public double gearRatio = 98.0;
    public Motor motor;
    public double desiredAngle_deg = 0.0;

    public Wrist(){
        motor= new Motor(14, MotorType.TFX);
        motor.motorConfig.direction = RotationDir.Clockwise;
        motor.motorConfig.peakForwardDC = 0.5;
        motor.motorConfig.peakReverseDC = -0.5;
        //motor.pid(0.11, 0.015, 0.025);
        motor.applyConfig();
        motor.pid(0.05, 0.001, 0.0);
    }

    public void goToPos(double angle_deg) {
        motor.pos(angle_deg / 360 * gearRatio);
        desiredAngle_deg = angle_deg;
    }

    @Override
    public void periodic() {
        System.out.println(Math.round(motor.pos()*360/gearRatio)+" : "+Math.round(desiredAngle_deg));
    }
}
