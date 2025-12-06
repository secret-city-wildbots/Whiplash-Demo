package frc.robot.Commands.Subsystems.Indexer;

// Import WPILib Command Libraries
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Actors.Subsystems.Indexer;
import frc.robot.Actors.Subsystems.Shooter;

public class Shoot extends SequentialCommandGroup {
    public Shoot(Indexer indexer, Shooter shooter) {
        addCommands(
            new Start(indexer),
            new WaitCommand(0.5),
            new Stop(indexer),
            new frc.robot.Commands.Subsystems.Shooter.Stop(shooter)
        );
    }
}