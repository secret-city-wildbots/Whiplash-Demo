
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Actors.Subsystems.Drivetrain;
import frc.robot.Actors.Subsystems.Indexer;
import frc.robot.Actors.Subsystems.Intake;
// Import Subsystems
import frc.robot.Actors.Subsystems.Shooter;
import frc.robot.Commands.Subsystems.Drivetrain.TeleopDrive;
import frc.robot.Commands.Subsystems.Indexer.Shoot;
import frc.robot.Commands.Subsystems.Indexer.Start;
import frc.robot.Commands.Subsystems.Intake.IntakeCommand;
import frc.robot.Commands.Subsystems.Intake.StopIntakingCommand;
//Import Commands
import frc.robot.Commands.Subsystems.Shooter.SpinUp;
import frc.robot.Commands.Subsystems.Shooter.Stop;

// Import WPILib Command Libraries
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Shooter shooter = new Shooter();
  private final Indexer indexer = new Indexer();
  private final Intake intake = new Intake();
  private final Drivetrain drivetrain = new Drivetrain();

  // Instantiate drive and manipulator Xbox Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController kidController = new CommandXboxController(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**e this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}
   * /{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driverController.leftBumper().onTrue(new IntakeCommand(intake));
    driverController.leftBumper().onFalse(new StopIntakingCommand(intake));
    driverController.rightBumper().onTrue(new Shoot(indexer, shooter));

    driverController.a().onTrue(new Stop(shooter));
    driverController.b().onTrue(new SpinUp(shooter, -0.4));
    driverController.b().onFalse(new Stop(shooter));

    driverController.rightTrigger(0.7).onTrue(new SpinUp(shooter, 1.0));
    driverController.leftTrigger(0.3).onTrue(new SpinUp(shooter, 0.2));
    driverController.leftTrigger(0.6).onTrue(new SpinUp(shooter, 0.4));
    driverController.leftTrigger(0.9).onTrue(new SpinUp(shooter, 0.6));

    kidController.rightTrigger(0.3).onTrue(new ConditionalCommand(new SpinUp(shooter, 0.2), new InstantCommand(), () -> {return Robot.kidControllerEnabled;}));
    kidController.rightTrigger(0.6).onTrue(new ConditionalCommand(new SpinUp(shooter, 0.4), new InstantCommand(), () -> {return Robot.kidControllerEnabled;}));
    kidController.rightTrigger(0.9).onTrue(new ConditionalCommand(new SpinUp(shooter, 0.6), new InstantCommand(), () -> {return Robot.kidControllerEnabled;}));
    kidController.rightBumper().onTrue(new Shoot(indexer, shooter));

    driverController.button(7).onTrue(new InstantCommand(() -> {Robot.drivetrainEnabled = false;}));
    driverController.button(8).onTrue(new InstantCommand(() -> {Robot.drivetrainEnabled = true;}));

    drivetrain.setDefaultCommand(
      new TeleopDrive(drivetrain, driverController)
    );
  }

  /**
   * Gets the command needed for autonomous from dashboard
   * 
   * @return the command
   */
  public Command getAutonomousCommand() {
    return Commands.print("no auto selected");
  }
}