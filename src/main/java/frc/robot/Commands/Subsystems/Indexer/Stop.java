package frc.robot.Commands.Subsystems.Indexer;

// Import WPILib Command Libraries
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Actors.Subsystems.Indexer;

public class Stop extends Command {
    // Real Variables
    private final Indexer indexer;

    /**
     * @param shooter The subsystem to be controlled by the command ({@link shooter})
     */
    public Stop(Indexer indexer) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void execute() {
        indexer.stop();
    }

    @Override
    public boolean isFinished() {
        // Do end the command
        return true;
    }
}