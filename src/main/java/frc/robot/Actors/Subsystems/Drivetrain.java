package frc.robot.Actors.Subsystems;

// Import CTRE Hardware Libraries
import com.ctre.phoenix6.hardware.Pigeon2;

// Import WPI Libraries to help Swerve Drive Management
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Import Constants and Utils
import frc.robot.Utils.*;
import frc.robot.Robot;
// Import Subsystems
import frc.robot.Actors.SwerveModule;
import frc.robot.Actors.SwerveModules;


public class Drivetrain extends SubsystemBase {

    // Inertial Measurement Unit
    private Pigeon2 pigeon;

    // Swerve Modules
    private SwerveModules swerveModules; // Helper class to help easily manage the individual swerve modules
    private Translation2d[] swerveModuleLocations_m; // Used to hold the position of the swerve modules with respect to the center of the robot
    private SwerveDriveOdometry odometry;
    private final SwerveDriveKinematics swerveKinematics;

    public SwerveModuleState[] moduleStates;

    public double azimuthGearRatio = 15.6;
    public double driveGearRatio = 6.42;
    public double wheelRadius_m = 0.057531;

    public double moduleToModuleLength_m = 0.311;
    public double moduleToModuleWidth_m = 0.254;

    public double maxGroundSpeed_mPs = 5.7;
    public double maxRotateSpeed_radPs = 7.8; //calculated using the max ground speed and simple geometry

    public Drivetrain() {
        /*
         * Setup auxillary sensors and components to help with the drivetrain
         */

        // pigeon
        this.pigeon = new Pigeon2(6);

        // Define the swerve modules
        // TODO: Test changing the numbers back to how we did last year. Found out how to display the swerves correctly on AdvantageScope
        this.swerveModules = new SwerveModules(
            new SwerveModule[] {
                new SwerveModule(1, wheelRadius_m, driveGearRatio, azimuthGearRatio, RotationDir.Clockwise),
                new SwerveModule(0, wheelRadius_m, driveGearRatio, azimuthGearRatio, RotationDir.CounterClockwise),
                new SwerveModule(2, wheelRadius_m, driveGearRatio, azimuthGearRatio, RotationDir.Clockwise),
                new SwerveModule(3, wheelRadius_m, driveGearRatio, azimuthGearRatio, RotationDir.CounterClockwise)
            }
        );

        /*
         * Setup the module locations with respect to the center of the robot. To calculate the center of the modules we
         * take have the module to module length and width. We also need to take into consideration the coordinate system
         * of WPILib. That can be found
         * here: https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#coordinate-system
         */
        // TODO: Test changing the numbers back to how we did last year. Found out how to display the swerves correctly on AdvantageScope
        this.swerveModuleLocations_m = new Translation2d[4];
        // Module 0 should be +X and -Y (Front Right - FR)
        this.swerveModuleLocations_m[1] = new Translation2d(
            moduleToModuleLength_m,
            -moduleToModuleWidth_m
        );
        // Module 1 should be +X and +Y (Front Left - FL)
        this.swerveModuleLocations_m[0] = new Translation2d(
            moduleToModuleLength_m,
            moduleToModuleWidth_m
        );
        // Module 2 should be -X and +Y (Back Left - BL)
        this.swerveModuleLocations_m[2] = new Translation2d(
            -moduleToModuleLength_m,
            moduleToModuleWidth_m
        );
        // Module 3 should be -X and -Y (Back Right - BR)
        this.swerveModuleLocations_m[3] = new Translation2d(
            -moduleToModuleLength_m,
            -moduleToModuleWidth_m
        );

        // Setup the swerve drive kinematics
        this.swerveKinematics = new SwerveDriveKinematics(this.swerveModuleLocations_m);

        // Setup the odometry tracking
        this.odometry = new SwerveDriveOdometry(
          this.swerveKinematics,
          this.pigeon.getRotation2d().unaryMinus(),
          swerveModules.getPosition()
        );
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        this.odometry.update(
            this.pigeon.getRotation2d().unaryMinus(),
            this.swerveModules.getPosition()
        );

        if (!Robot.drivetrainEnabled) {
            for (int i = 0; i < swerveModules.swerveModules.length; i++) {
                swerveModules.swerveModules[i].drive.dc(0);
                swerveModules.swerveModules[i].azimuth.dc(0);
            }
        }
    }

    /**
     * gets the current pose from the swerve odometry.
     * 
     * @return Pose2d
     */
    public Pose2d getPose() {
        return this.odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        this.odometry.resetPosition(
            this.pigeon.getRotation2d().unaryMinus(),
            this.swerveModules.getPosition(),    
            pose
        );
    }

    /**
     * drive the drivetrain at an x, y and h power field relative
     * 
     * @param xpow        power to drive at in field relative x
     * @param ypow        power to drive at in field relative y
     * @param hpow        power to change the heading (spin) at
     */
    public void drive(double xpow, double ypow, double hpow) {
        /**
         * This handles the issue where the xpow and ypow are values that end up in the shaded (unreal areas) of a circumscribed square
         * We caclulate the hypotenuse and if it ends up in the area between the square and circle (> 1), then we divide the values
         * by the hypotenuse to get them in a valid range. If we don't do this, then the swerveKinematics.toSwerveModuleStates will seem
         * to not be "reacting" to the controller changes
         */
        double hyp = Math.hypot(xpow, ypow);
        if (hyp > 1.0){
          ypow /= hyp;
          xpow /= hyp;
        }

        // Calculate the swerve module states (drive and azimuth motor commands) based on the controller inputs and the max ground and rotate speeds
        SwerveModuleState[] moduleStateOutputs = this.swerveKinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(
                xpow * maxGroundSpeed_mPs,
                ypow * maxGroundSpeed_mPs,
                hpow * maxRotateSpeed_radPs,
                // Converting pigeon from left hand rule (x+) to a right hand rule (x+)
                this.pigeon.getRotation2d().unaryMinus()
                ),
                0.020
            )
        );

        // Renormalizes the wheel speeds if any individual speed is above the specified maximum. 
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStateOutputs, maxGroundSpeed_mPs);

        // Send the commands to the swerve modules
        if (Robot.drivetrainEnabled) {
            swerveModules.pushModuleStates(moduleStateOutputs, maxGroundSpeed_mPs);
        }
    }

    /** Zeroes the heading of the robot. */
    public void resetIMU() {
        this.pigeon.reset();
    }
}