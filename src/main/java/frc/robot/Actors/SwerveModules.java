package frc.robot.Actors;

// Import WPI Libraries to help Swerve Drive Management
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

// Import SmartDashbaord Library for sending information to the Network Tables
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Import SubsystemBase for extension
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModules extends SubsystemBase {
    // Array to hold all of our swerve modules
    public SwerveModule[] swerveModules;

    /**
     * a helper class so that you don't have to be constantly annoyed by arrays
     * 
     * @param modules the swerve modules to incorporate
     */
    public SwerveModules(SwerveModule[] modules) {
        this.swerveModules = modules;
    }

    /**
     * get the positions of all the swerve modules
     * 
     * @return and array containing all the SwerveModulePositions
     */
    public SwerveModulePosition[] getPosition() {
        // Create array of swerve module positions
        SwerveModulePosition[] positions = new SwerveModulePosition[this.swerveModules.length];

        // Loop through each of the swerve modules and get their indivual positions and add them to the array
        for (int i = 0; i < this.swerveModules.length; i++) {
            positions[i] = this.swerveModules[i].getPosition();
        }

        // Return all of the positions
        return positions;
    }

    /**
     * sets the output states of all the swerve modules
     */
    public void pushModuleStates(SwerveModuleState[] moduleStates, double maxGroundSpeed_mPs) {
        for (int i = 0; i < this.swerveModules.length; i++) {
            this.swerveModules[i].pushModuleState(moduleStates[i], maxGroundSpeed_mPs);
        }

        // Logging values for use in Advantage Scope
        double[] loggingState = new double[] {
                moduleStates[0].angle.getRadians(),
                moduleStates[0].speedMetersPerSecond,
                moduleStates[1].angle.getRadians(),
                moduleStates[1].speedMetersPerSecond,
                moduleStates[2].angle.getRadians(),
                moduleStates[2].speedMetersPerSecond,
                moduleStates[3].angle.getRadians(),
                moduleStates[3].speedMetersPerSecond
        };

        SmartDashboard.putNumberArray("moduleStates", loggingState);
    }

    /**
     * get the module state from all the swerve modules
     * 
     * @return and array containing all the SwerveModuleStates
     */
    public SwerveModuleState[] getCurrentState() {
        // Create array of swerve module states
        SwerveModuleState[] states = new SwerveModuleState[this.swerveModules.length];

        // Loop through each of the swerve modules and get their indivual states and add them to the array
        for (int i = 0; i < this.swerveModules.length; i++) {
            states[i] = this.swerveModules[i].getCurrentState();
        }

        // Return the states
        return states;
    }

    @Override
    public void periodic() {
        // Get the current states of the modules
        var moduleStates = getCurrentState();

        // Logging values for use in Advantage Scope
        double[] loggingState = new double[] {
            moduleStates[1].angle.getRadians(),
            moduleStates[1].speedMetersPerSecond,
            moduleStates[0].angle.getRadians(),
            moduleStates[0].speedMetersPerSecond,
            moduleStates[3].angle.getRadians(),
            moduleStates[3].speedMetersPerSecond,
            moduleStates[2].angle.getRadians(),
            moduleStates[2].speedMetersPerSecond
        };

        SmartDashboard.putNumberArray("realModuleStates", loggingState);
    }
}