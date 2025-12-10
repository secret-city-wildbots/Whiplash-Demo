package frc.robot.Actors;

// Import CTRE Hardware Libraries
import com.ctre.phoenix6.configs.TalonFXConfiguration;

// Import WPI Libraries to help Swerve Drive Management
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
// Import Utils and Constants
import frc.robot.Utils.MotorType;
import frc.robot.Utils.RotationDir;

public class SwerveModule extends SubsystemBase {
    /*
     * Module number for the FRC Robot - Help determine the CAN ID of the swerve module
     * 
     * 4265 Naming/ID Convention:
     * Module 0 - Front Right (Drive Motor CAN ID - 10, Azimuth Motor CAN ID - 20)
     * Module 1 - Front Right (Drive Motor CAN ID - 11, Azimuth Motor CAN ID - 21)
     * Module 2 - Front Right (Drive Motor CAN ID - 12, Azimuth Motor CAN ID - 22)
     * Module 3 - Front Right (Drive Motor CAN ID - 13, Azimuth Motor CAN ID - 23)
     */
    private final int moduleNumber;

    // Define our drive and azimuth motors
    public Motor drive;
    public Motor azimuth;

    // Setup variables for tracking the speed and angle of the module
    private double currentDriveSpeed_mPs = 0;
    private double currentAzimuthAngle_rad = 0;

    public double azimuthGearRatio;
    public double driveGearRatio;
    public double wheelRadius_m;

    public SwerveModule(int moduleNumber, double wheelRadius_m, double driveGearRatio, double azimuthGearRatio, RotationDir driveDir) {
        // Set the module number of the swerve
        this.moduleNumber = moduleNumber;
        this.azimuthGearRatio = azimuthGearRatio;
        this.driveGearRatio = driveGearRatio;
        this.wheelRadius_m = wheelRadius_m;

        // Setup the drive motor configurations
        this.drive = new Motor(10 + this.moduleNumber, "canivore");
        this.drive.motorConfig.forwardLimitSwitchEnabled = false;
        this.drive.motorConfig.reverseLimitSwitchEnabled = false;
        this.drive.motorConfig.brake = false;
        this.drive.motorConfig.dutyCycleClosedLoopRampPeriod = 0.2;
        this.drive.motorConfig.dutyCycleOpenLoopRampPeriod = 0.4; //this one is used, as it's open loop duty cycle
        this.drive.motorConfig.direction = driveDir;
        this.drive.motorConfig.peakForwardDC = 0.3;
        this.drive.motorConfig.peakReverseDC = 0.3;
        this.drive.applyConfig();

        // Setup the azimuth motor configurations
        this.azimuth = new Motor(20 + moduleNumber, "canivore");
        this.drive.motorConfig.forwardLimitSwitchEnabled = false;
        this.drive.motorConfig.reverseLimitSwitchEnabled = false;
        this.drive.motorConfig.brake = true;
        this.drive.motorConfig.dutyCycleClosedLoopRampPeriod = 0.1; //this one is used, as it's a close loop pid
        this.drive.motorConfig.dutyCycleOpenLoopRampPeriod = 0.2;
        this.drive.motorConfig.direction = RotationDir.CounterClockwise;
        this.drive.applyConfig();

        // Setup the Azimuth PID
        this.azimuth.pid(0.15, 0.0, 0.0);
    }

    /**
     * updates the internal current state of the swerve module.
     * 
     * @return the current swerveModuleState of the module
     */
    public SwerveModuleState getCurrentState() {
        // Calculate the current module angle in radians
        currentAzimuthAngle_rad = Units.rotationsToRadians(azimuth.pos() / azimuthGearRatio);
        // Calculate the current module wheel speein in meters / second
        currentDriveSpeed_mPs = drive.vel()
            / driveGearRatio * 2
            * Math.PI
            * wheelRadius_m;

        // Return the swerve module state
        return new SwerveModuleState(currentDriveSpeed_mPs, new Rotation2d(currentAzimuthAngle_rad));
    }

    /**
     * Returns the position of the drive and azimuth motors
     * 
     * @return A SwerveModulePosition object
     */
    public SwerveModulePosition getPosition() {
        // Calculate the swerve module position
        return new SwerveModulePosition(
            (drive.pos() / driveGearRatio) * (2 * Math.PI * wheelRadius_m),
            new Rotation2d((azimuth.pos() / azimuthGearRatio) * 2 * Math.PI)
        );
    }

    /**
     * Gets any faults from the drive and azimuth motors
     * 
     * @return A boolean array with the structure:
     *         <ul>
     *         <li>drive fault,
     *         <li>azimuth fault
     */
    public boolean[] getSwerveFaults() {
        return new boolean[] { drive.isFault(), azimuth.isFault() };
    }

    public void pushModuleState(SwerveModuleState moduleState, double maxGroundSpeed_mPs) {
        /*
         * Determine the module states to create
         */
        // Get the encoderRotation and azimuth angle in radians
        var encoderRotation = new Rotation2d(Units.rotationsToRadians(azimuth.pos() / azimuthGearRatio));
        double azimuthAngle_rad = Units.rotationsToRadians(azimuth.pos() / azimuthGearRatio);

        // Optimize the reference state to avoid spinning further than 90 degrees
        moduleState.optimize(encoderRotation);

        // TODO: I think we can uncomment this when we are actually driving the robot. The signal output I thinks goes to 0 when testing
        // because the angle of the wheel is not being updated
        // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        // direction of travel that can occur when modules change directions. This results in smoother
        // driving.
        //moduleState.cosineScale(encoderRotation);

        // Wrapping the angle to allow for "continuous input"
        double minDistance = MathUtil.angleModulus(moduleState.angle.getRadians() - azimuthAngle_rad);

        /*
         * Calculate the azimuth output
         */
        double normalAzimuthOutput_rot = Units.radiansToRotations(azimuthAngle_rad + minDistance)
                * azimuthGearRatio;

        /*
         * Calculate the drive output
         */
        // Output drive
        double driveOutput = moduleState.speedMetersPerSecond / maxGroundSpeed_mPs;

        // TODO: Is this the same as: moduleState.cosineScale(encoderRotation) (LINE 113)
        driveOutput *= moduleState.angle.minus(new Rotation2d(currentAzimuthAngle_rad)).getCos();

        // Send the outputs to the drive and azimuth motors
        if (Robot.drivetrainEnabled) {
            azimuth.pos(normalAzimuthOutput_rot);
            drive.dc(driveOutput);
        } else {
            azimuth.dc(0);
            drive.dc(0);
        }
    }

    /**
     * TODO: describe the purpose of this function
     */
    public void pushLockState(boolean calibrateWheels, boolean unlockWheels) {
        if (calibrateWheels) {
            azimuth.resetPos(0.0);
        }

        azimuth.setBrake(!unlockWheels); // invert bc unlock != lock
    }

    /**
     * Temperature of drive motor
     * 
     * <ul>
     * <li><b>Minimum Value:</b> 0.0
     * <li><b>Maximum Value:</b> 255.0
     * <li><b>Default Value:</b> 0
     * <li><b>Units:</b> ℃
     * </ul>
     * 
     * @return Double temperature in degrees Celcius
     */
    public double getTemp() {
        return drive.getTemp();
    }
}