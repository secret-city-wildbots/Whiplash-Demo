package frc.robot.Commands.Subsystems.Shooter;

// Import WPILib Command Libraries
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Actors.Subsystems.Shooter;

public class SpinUp extends Command {
    // Real Variables
    private final Shooter shooter;

    /**
     * @param shooter The subsystem to be controlled by the command ({@link shooter})
     */
    public SpinUp(Shooter shooter) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.spinUp();
    }

    @Override
    public boolean isFinished() {
        // Do end the command
        return true;
    }
}