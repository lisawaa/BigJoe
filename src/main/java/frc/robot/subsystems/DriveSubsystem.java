package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

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
import edu.wpi.first.math.geometry.Translation2d;
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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.Configs;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Drive.Constants.MotorLocation;
import frc.robot.Constants.IDs;
import frc.robot.Constants.Operating;
import frc.robot.Constants.Vision;
import frc.robot.Constants.Vision.VisionIOInputs;
import frc.robot.components.SwerveModule;
import frc.robot.components.SwerveModuleIO;
import frc.robot.components.SwerveModuleIOSim;
import frc.robot.components.SwerveModuleIOSparkMax;
import frc.robot.components.SwerveModuleIOReplay;

public class DriveSubsystem extends SubsystemBase{
    //Creates Swerve Modules
    private SwerveModule frontLeft = null;
    private SwerveModule frontRight = null;
    private SwerveModule backLeft = null;  
    private SwerveModule backRight = null;

    private final Pigeon2 gyro = Operating.Constants.USING_GYRO ? new Pigeon2(IDs.DriveConstants.PIGEON_ID) : null;

    private SwerveModuleState desiredStates[] = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
    private StructArrayPublisher<SwerveModuleState> publisherDesieredStates = NetworkTableInstance.getDefault().getStructArrayTopic("MyDesiredStates", SwerveModuleState.struct).publish();
    private StructArrayPublisher<SwerveModuleState> publisherActualStates = NetworkTableInstance.getDefault().getStructArrayTopic("MyActualStates", SwerveModuleState.struct).publish();
    private StructArrayPublisher<SwerveModulePosition> publisherPositions = NetworkTableInstance.getDefault().getStructArrayTopic("MyPositions", SwerveModulePosition.struct).publish();

    private StructPublisher<Pose2d> publisherPose = NetworkTableInstance.getDefault().getStructTopic("SwervePose", Pose2d.struct).publish();
 
    SwerveDriveOdometry odometry = null;
    private VisionSubsystem visionIO = null;
    private SwerveDrivePoseEstimator poseEstimator = null;
    private double lastMatchLog = 0.0;
    private boolean lastTeleopEnabled = false;
    private boolean lastAutonomousEnabled = false;

    //Constructs a new DriveSubsystem
    public DriveSubsystem(Optional<VisionSubsystem> photonVision) {
        if(photonVision.isEmpty()) {
            visionIO = new VisionSubsystem();
        } else {
            visionIO = photonVision.get();
        }
        
        //Usage reporting for MAXSwerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
        RobotConfig config = null;

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

    public void driveAligned(double xSpeed, double ySpeed, boolean fieldRelative, String statusName) { //fieldRelative may be redundant
        double multiplier = 0.5; //update(?)
        double xSpeedDelivered = xSpeed * Drive.Constants.MAX_METERS_PER_SECOND * multiplier;
        double ySpeedDelivered = ySpeed * Drive.Constants.MAX_METERS_PER_SECOND * multiplier;

        Pose2d drivePose = getPose();
        Translation2d shooterOffsetRobot = new Translation2d(); //UPDATE
        Translation2d shooterOffsetField = shooterOffsetRobot.rotateBy(drivePose.getRotation());
        Translation2d shooterPos = drivePose.getTranslation().plus(shooterOffsetField);
        Rotation2d desiredAngle = Vision.Constants.getHubPose().toPose2d().getTranslation().minus(shooterPos).getAngle();
       
        //rot * Drive.Constants.MAX_ANGULAR_SPEED * multiplier; Verify angles and PID with testing
        double rotDelivered = Drive.Constants.ROTATION_CONTROLLER.calculate(drivePose.getRotation().getRadians(), desiredAngle.getRadians()); 
    
        //See if it needs to be field relative or not
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
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
            getRotation2d(),
            getSwerveModulePosition(),
            pose);
    }

     public void resetPose(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }


    //Resests drive encoders to read a position of 0.
    public void resetEncoders() {
        frontLeft.resetDriveEncoder();
        frontRight.resetDriveEncoder();
        backLeft.resetDriveEncoder();
        backRight.resetDriveEncoder();
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
        //Logging Drive Outputs
        Logger.recordOutput("Drive/Pose", poseEstimator.getEstimatedPosition());
        Logger.recordOutput("Drive/Pose/X", poseEstimator.getEstimatedPosition().getX());
        Logger.recordOutput("Drive/Pose/Y", poseEstimator.getEstimatedPosition().getY());
        Logger.recordOutput("Drive/Pose/Rotation", poseEstimator.getEstimatedPosition().getRotation().getDegrees());

        if (Operating.Constants.USING_GYRO) {
            Logger.recordOutput("Drive/Gyro/Yaw", gyro.getYaw().getValue());
            Logger.recordOutput("Drive/Gyro/Pitch", gyro.getPitch().getValue());
            Logger.recordOutput("Drive/Gyro/Roll", gyro.getRoll().getValue());
        }

        Logger.recordOutput("Drive/ModuleStates/Desired", desiredStates);
        Logger.recordOutput("Drive/ModuleStates/Actual", getSwerveModuleState());

        //Logging Battery Voltage
        Logger.recordOutput("Power/BatteryVoltage", RobotController.getBatteryVoltage());

        //Logging Match Time (throttled)
        double now = Timer.getFPGATimestamp();
        if (now - lastMatchLog > 0.2) {
            lastMatchLog = now;
            Logger.recordOutput("Match/TimeRemaining", DriverStation.getMatchTime());
        }

        // Log teleop/autonomous enable state on changes
        boolean teleop = DriverStation.isTeleopEnabled();
        boolean auton = DriverStation.isAutonomousEnabled();
        if (teleop != lastTeleopEnabled || auton != lastAutonomousEnabled) {
            lastTeleopEnabled = teleop;
            lastAutonomousEnabled = auton;
            Logger.recordOutput("Match/TeleopEnabled", teleop);
            Logger.recordOutput("Match/AutonomousEnabled", auton);
            String mode = auton ? "Autonomous" : teleop ? "Teleop" : "Disabled";
            Logger.recordOutput("Match/Mode", mode);
        }

        odometry.update(getRotation2d(), getSwerveModulePosition());
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getRotation2d(), getSwerveModulePosition());

        if(Operating.Debugging.DRIVE_DEBUG) {
            updateSmartDashboard();
            //updateWheelPositions - add for extra debugging functionality
            publisherDesieredStates.set(desiredStates);
            publisherActualStates.set(getSwerveModuleState());
            publisherPositions.set(getSwerveModulePosition());
        }
        if(Operating.Constants.USING_VISION) {
            VisionIOInputs inputs = visionIO.getInputs();
            for(int i = 0; i < inputs.cameraPoses.length; i++) {
                if(inputs.cameraTargets[i] != null) {
                    poseEstimator.addVisionMeasurement(inputs.cameraPoses[i].toPose2d(), inputs.timestamps[i]);
                }
            }
        }
        
        publisherPose.set(poseEstimator.getEstimatedPosition());
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Robot Heading Yaw", getRotation2d().getDegrees());
        if(Operating.Constants.USING_GYRO) {
            SmartDashboard.putNumber("Roll", gyro.getRoll().getValue().in(Units.Degrees));
            SmartDashboard.putNumber("Pitch", gyro.getPitch().getValue().in(Units.Degrees));
        }
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putNumber("FGPA Time", Timer.getFPGATimestamp());
    }

    //Might want to add getWheelRotationSupplier() and any test methods if needed

    @Override
    public void simulationPeriodic() {
        var moduleStates = new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };

        var chassisSpeeds = Drive.Constants.DRIVE_KINEMATICS.toChassisSpeeds(moduleStates);
        
        if (gyro != null) {
            double loopTime = 0.02; // 20ms standard loop
            double currentOmegaRadPerSec = chassisSpeeds.omegaRadiansPerSecond;
            double degreesChange = Math.toDegrees(currentOmegaRadPerSec * loopTime);
            
            gyro.getSimState().addYaw(degreesChange);
        }

        // record to akit?
        Logger.recordOutput("Odometry/RobotPose", getOdometry());
        Logger.recordOutput("Odometry/ModuleStates", moduleStates);
    }
}
