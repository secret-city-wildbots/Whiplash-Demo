package frc.robot.Actors.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Actors.Motor;
import frc.robot.Utils.MotorType;
import frc.robot.Utils.RotationDir;

public class Intake extends SubsystemBase {
    public Motor intakeMotor;
    public Motor frontIntakeMotor;
    public Motor rearIntakeMotor;

    public Intake() {
        intakeMotor = new Motor(19, MotorType.TFX);
        frontIntakeMotor = new Motor(24, MotorType.TFX);
        rearIntakeMotor = new Motor(25, MotorType.TFX);

        intakeMotor.motorConfig.direction = RotationDir.CounterClockwise;
        intakeMotor.motorConfig.brake = false;
        frontIntakeMotor.motorConfig.direction = RotationDir.CounterClockwise;
        frontIntakeMotor.motorConfig.brake = false;
        rearIntakeMotor.motorConfig.direction = RotationDir.CounterClockwise;
        rearIntakeMotor.motorConfig.brake = false;

        intakeMotor.applyConfig();
        frontIntakeMotor.applyConfig();
        rearIntakeMotor.applyConfig();
    }

    public void intake() {
        intakeMotor.dc(0.4);
        frontIntakeMotor.dc(0.7);
        rearIntakeMotor.dc(0.7);
        System.out.println("INTAKING");
    }

    public void stop() {
        intakeMotor.dc(0.0);
        frontIntakeMotor.dc(0.0);
        rearIntakeMotor.dc(0.0);
    }
}
