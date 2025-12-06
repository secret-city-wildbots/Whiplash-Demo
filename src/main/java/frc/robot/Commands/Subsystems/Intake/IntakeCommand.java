package frc.robot.Commands.Subsystems.Intake;

// Import WPILib Command Libraries
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Actors.Subsystems.Intake;

public class IntakeCommand extends Command {
    // Real Variables
    private final Intake intake;

    /**
     * @param shooter The subsystem to be controlled by the command ({@link shooter})
     */
    public IntakeCommand(Intake intake) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.intake();
        System.out.println("SPINING");
    }

    @Override
    public boolean isFinished() {
        // Do end the command
        return true;
    }
}