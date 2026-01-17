package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Configs;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Drive.Constants.MotorLocation;
import frc.robot.Constants.IDs;
import frc.robot.Constants.Operating;
import frc.robot.Constants.Vision;
import frc.robot.Constants.Vision.VisionIOInputs;

public class DriveSubsystem extends SubsystemBase{
    //Creates Swerve Modules
     private final SwerveModule frontLeft = new SwerveModule(
        IDs.DriveConstants.FL_DRIVE_ID,
        IDs.DriveConstants.FL_TURN_ID,
        Drive.Constants.FL_ANGULAR_OFFSET,
        Drive.Constants.FL_INVERTED,
        Configs.SwerveModule.FL_CONFIG,
        MotorLocation.FRONT_LEFT);

    private final SwerveModule backLeft = new SwerveModule(
        IDs.DriveConstants.BL_DRIVE_ID,
        IDs.DriveConstants.BL_TURN_ID,
        Drive.Constants.BL_ANGULAR_OFFSET,
        Drive.Constants.BL_INVERTED,
        Configs.SwerveModule.BL_CONFIG,
        MotorLocation.BACK_LEFT);

    private final SwerveModule frontRight = new SwerveModule(
        IDs.DriveConstants.FR_DRIVE_ID,
        IDs.DriveConstants.FR_TURN_ID,
        Drive.Constants.FR_ANGULAR_OFFSET,
        Drive.Constants.FR_INVERTED,
        Configs.SwerveModule.FR_CONFIG,
        MotorLocation.FRONT_RIGHT);

    private final SwerveModule backRight = new SwerveModule(
        IDs.DriveConstants.BR_DRIVE_ID,
        IDs.DriveConstants.BR_TURN_ID,
        Drive.Constants.BR_ANGULAR_OFFSET,
        Drive.Constants.BR_INVERTED,
        Configs.SwerveModule.BR_CONFIG,
        MotorLocation.BACK_RIGHT);

    private final Pigeon2 gyro = Operating.Constants.USING_GYRO ? new Pigeon2(IDs.DriveConstants.PIGEON_ID) : null;

    private SwerveModuleState desiredStates[] = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
    private StructArrayPublisher<SwerveModuleState> publisherDesieredStates = NetworkTableInstance.getDefault().getStructArrayTopic("MyDesiredStates", SwerveModuleState.struct).publish();
    private StructArrayPublisher<SwerveModuleState> publisherActualStates = NetworkTableInstance.getDefault().getStructArrayTopic("MyActualStates", SwerveModuleState.struct).publish();
    private StructPublisher<Pose2d> publisherPose = NetworkTableInstance.getDefault().getStructTopic("SwervePose", Pose2d.struct).publish();
 
    SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        Drive.Constants.DRIVE_KINEMATICS,
        getRotation2d(),
        getSwerveModulePosition());

    private final VisionSubsystem visionIO = (Operating.Constants.USING_VISION) ? new VisionSubsystem() : null;
    private final VisionIOInputs visionInputs = (Operating.Constants.USING_VISION) ? new VisionIOInputs() : null;
    private SwerveDrivePoseEstimator poseEstimator =
        new SwerveDrivePoseEstimator(Drive.Constants.DRIVE_KINEMATICS, getRotation2d(), getSwerveModulePosition(), new Pose2d(),
            Vision.Constants.SINGLE_STD_DEVS, Vision.Constants.SINGLE_STD_DEVS);
    
    //Constructs a new DriveSubsystem
    public DriveSubsystem() {
        //Usage reporting for MAXSwerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
        RobotConfig config;

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            config = null;
            e.printStackTrace();
        }

        configureAutoBuilder();
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, String statusName) {
        //Convert speeds into proper units for drivetrain
        double multiplier = 0.5; //update(?)
        double xSpeedDelivered = xSpeed * Drive.Constants.MAX_METERS_PER_SECOND * multiplier;
        double ySpeedDelivered = ySpeed * Drive.Constants.MAX_METERS_PER_SECOND * multiplier;
        double rotDelivered = rot * Drive.Constants.MAX_ANGULAR_SPEED * multiplier;

        //was type 'var' beforehand, fix?
        SwerveModuleState[] swerveModuleStates = Drive.Constants.DRIVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                getRotation2d())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, Drive.Constants.MAX_METERS_PER_SECOND);
        desiredStates = swerveModuleStates;
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);

        SmartDashboard.putString("Drive Mode", statusName);
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        // Stripped from Template Pathplanner Github
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
        SwerveModuleState[] targetStates = Drive.Constants.DRIVE_KINEMATICS.toSwerveModuleStates(targetSpeeds);
        frontLeft.setDesiredState(targetStates[0]);
        frontRight.setDesiredState(targetStates[1]);
        backLeft.setDesiredState(targetStates[2]);
        backRight.setDesiredState(targetStates[3]);
    }

    //returns a list of SwerveModulesStates
    public SwerveModuleState[] getSwerveModuleState() {
        return new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
    }

    //returns a list of SwerveModulePositions
    public SwerveModulePosition[] getSwerveModulePosition() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    //Returns the heading of the robot.
    public Rotation2d getRotation2d() {
        //return new Rotation2d(edu.wpi.first.math.util.Units.degreesToRadians(0)); //This way avoids issues
        if(Operating.Constants.USING_GYRO) 
            return new Rotation2d(gyro.getYaw().getValue()); //might have to invert (360 - yaw)
        else 
            return new Rotation2d(0);
    }

    //Returns the currently-estimated pose of the robot
    public Pose2d getOdometry() {
        return odometry.getPoseMeters();
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    //WIP
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Drive.Constants.DRIVE_KINEMATICS.toChassisSpeeds(
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        );
   }

    //Resets odometry to the specified pose.
    public void resetOdometry(Pose2d p_pose) {
        odometry.resetPosition(
            getRotation2d(),
            getSwerveModulePosition(),
            p_pose);
    }

    //Resests drive encoders to read a position of 0.
    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    //Zereos the heading of the robot.
    public void zeroHeading() {
        if(Operating.Constants.USING_GYRO) gyro.reset();
    }

    //Stops all motors on the DriveSubsystem.
    public void stopModules() {
        frontLeft.stopMotors();
        frontRight.stopMotors();
        backLeft.stopMotors();
        backRight.stopMotors();

        for(int i = 0; i < desiredStates.length; i++)
            desiredStates[i].speedMetersPerSecond = 0;
    }

    //WIP
    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                this::getOdometry,   // Supplier of current robot pose
                this::resetOdometry,         // Consumer for seeding pose against auto
                this::getRobotRelativeSpeeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                this::driveRobotRelative,
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(.04, 0, 0), //Change(?)
                    // PID constants for rotation
                    new PIDConstants(1, 0, 0) //Change(?)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    @Override
    public void periodic() {
        odometry.update(getRotation2d(), getSwerveModulePosition());

        if(Operating.Debugging.DRIVE_DEBUG) {
            updateSmartDashboard();
            //updateWheelPositions - add for extra debugging functionality
            publisherDesieredStates.set(desiredStates);
            publisherActualStates.set(getSwerveModuleState());

            frontLeft.updateSmartDashboard();
            frontRight.updateSmartDashboard();
            backLeft.updateSmartDashboard();
            backRight.updateSmartDashboard();
        }
        if(Operating.Constants.USING_VISION) {
            poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getRotation2d(), getSwerveModulePosition());

            visionIO.updateInputs(visionInputs, getOdometry());

            if(visionInputs.hasEstimate){
                for(int i = 0; i < visionInputs.estimate.length; i++) {
                    poseEstimator.addVisionMeasurement(visionInputs.estimate[i], Timer.getFPGATimestamp());
                }
            }  
            publisherPose.set(poseEstimator.getEstimatedPosition());
        }
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Robot Heading Yaw", getRotation2d().getDegrees());
        if(Operating.Constants.USING_GYRO) {
            SmartDashboard.putNumber("Roll", gyro.getRoll().getValue().in(Units.Degrees));
            SmartDashboard.putNumber("Pitch", gyro.getPitch().getValue().in(Units.Degrees));
        }
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    //Might want to add getWheelRotationSupplier() and any test methods if needed
}