package frc.robot.Commands.Subsystems.Wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Actors.Subsystems.Wrist;

public class MoveToPos extends Command {

    // Real Variables
    private final Wrist wrist;
    private final double angle;

    /**
     * @param wrist The subsystem to be controlled by the command ({@link Wrist})
     */
    public MoveToPos(Wrist wrist, double angle) {
        // Assign the variables and add the subsystem as a requirement to the command
        this.wrist = wrist;
        this.angle = angle;
        addRequirements(this.wrist);
        }

    @Override
    public void execute() {
        wrist.goToPos(angle);
    }

    @Override
    public boolean isFinished() {
        // Do end the command
        return true;
    }
    
}
