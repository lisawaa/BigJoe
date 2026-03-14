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
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
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
import frc.robot.components.SwerveModuleIOSim;
import frc.robot.components.SwerveModuleIOSparkMax;

public class DriveSubsystem extends SubsystemBase{
    private SwerveModule frontLeft = null;
    private SwerveModule frontRight = null;
    private SwerveModule backLeft = null;  
    private SwerveModule backRight = null;
    
    private SwerveModuleState desiredStates[] = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};

    private final Pigeon2 gyro = Operating.Constants.USING_GYRO ? new Pigeon2(IDs.DriveConstants.PIGEON_ID) : null;
    SwerveDriveOdometry odometry = null;
    
    private VisionSubsystem visionIO = null;
    private SwerveDrivePoseEstimator poseEstimator = null;
    
    private double lastMatchLog = 0.0;
    private boolean lastTeleopEnabled = false;
    private boolean lastAutonomousEnabled = false;

    public DriveSubsystem(Optional<VisionSubsystem> photonVision) {
        if(Robot.isSimulation()) {
            frontLeft = new SwerveModule(new SwerveModuleIOSim(Drive.Constants.FL_ANGULAR_OFFSET), MotorLocation.FRONT_LEFT);
            frontRight = new SwerveModule(new SwerveModuleIOSim(Drive.Constants.FR_ANGULAR_OFFSET), MotorLocation.FRONT_RIGHT);
            backLeft = new SwerveModule(new SwerveModuleIOSim(Drive.Constants.BL_ANGULAR_OFFSET), MotorLocation.BACK_LEFT);
            backRight = new SwerveModule(new SwerveModuleIOSim(Drive.Constants.BR_ANGULAR_OFFSET), MotorLocation.BACK_RIGHT);
        }
        else {
            frontLeft = new SwerveModule(
                new SwerveModuleIOSparkMax(
                    IDs.DriveConstants.FL_DRIVE_ID,
                    IDs.DriveConstants.FL_TURN_ID,
                    Drive.Constants.FL_ANGULAR_OFFSET,
                    Configs.SwerveModule.FL_CONFIG,
                    Configs.SwerveModule.TURNING_CONFIG),
                MotorLocation.FRONT_LEFT);
            frontRight = new SwerveModule(new SwerveModuleIOSparkMax(
                    IDs.DriveConstants.FR_DRIVE_ID,
                    IDs.DriveConstants.FR_TURN_ID,
                    Drive.Constants.FR_ANGULAR_OFFSET,
                    Configs.SwerveModule.FR_CONFIG,
                    Configs.SwerveModule.TURNING_CONFIG),
                MotorLocation.FRONT_RIGHT);
            backLeft = new SwerveModule(new SwerveModuleIOSparkMax(
                    IDs.DriveConstants.BL_DRIVE_ID,
                    IDs.DriveConstants.BL_TURN_ID,
                    Drive.Constants.BL_ANGULAR_OFFSET,
                    Configs.SwerveModule.BL_CONFIG,
                    Configs.SwerveModule.TURNING_CONFIG),
                MotorLocation.BACK_LEFT);
            backRight = new SwerveModule(new SwerveModuleIOSparkMax(
                    IDs.DriveConstants.BR_DRIVE_ID,
                    IDs.DriveConstants.BR_TURN_ID,
                    Drive.Constants.BR_ANGULAR_OFFSET,
                    Configs.SwerveModule.BR_CONFIG,
                    Configs.SwerveModule.TURNING_CONFIG),
                 MotorLocation.BACK_RIGHT);
        }

        odometry = new SwerveDriveOdometry(
            Drive.Constants.DRIVE_KINEMATICS,
            getRotation2d(),
            getSwerveModulePosition());

        poseEstimator = new SwerveDrivePoseEstimator(Drive.Constants.DRIVE_KINEMATICS, getRotation2d(), getSwerveModulePosition(), new Pose2d(),
            Vision.Constants.SINGLE_STD_DEVS, Vision.Constants.SINGLE_STD_DEVS);

        if(photonVision.isEmpty()) {
            visionIO = new VisionSubsystem();
        } else {
            visionIO = photonVision.get();
        }
        
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
        double multiplier = 0.5; 
        double xSpeedDelivered = xSpeed * Drive.Constants.MAX_METERS_PER_SECOND * multiplier;
        double ySpeedDelivered = ySpeed * Drive.Constants.MAX_METERS_PER_SECOND * multiplier;
        double rotDelivered = rot * Drive.Constants.MAX_ANGULAR_SPEED * multiplier;

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

    public void driveAligned(double xSpeed, double ySpeed, boolean fieldRelative, String statusName) {
        double multiplier = 0.5;
        double xSpeedDelivered = xSpeed * Drive.Constants.MAX_METERS_PER_SECOND * multiplier;
        double ySpeedDelivered = ySpeed * Drive.Constants.MAX_METERS_PER_SECOND * multiplier;

        Pose2d drivePose = getPose();
        Translation2d shooterOffsetRobot = new Translation2d(); //Update fpr shooter location;
        Translation2d shooterOffsetField = shooterOffsetRobot.rotateBy(drivePose.getRotation());
        Translation2d shooterPos = drivePose.getTranslation().plus(shooterOffsetField);
        Rotation2d desiredAngle = Vision.Constants.getHubPose().toPose2d().getTranslation().minus(shooterPos).getAngle();
       
        double rotDelivered = Drive.Constants.ROTATION_CONTROLLER.calculate(drivePose.getRotation().getRadians(), desiredAngle.getRadians() + Math.PI); //undo PI for actual 
    
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
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
        SwerveModuleState[] targetStates = Drive.Constants.DRIVE_KINEMATICS.toSwerveModuleStates(targetSpeeds);
        frontLeft.setDesiredState(targetStates[0]);
        frontRight.setDesiredState(targetStates[1]);
        backLeft.setDesiredState(targetStates[2]);
        backRight.setDesiredState(targetStates[3]);
    }

    public SwerveModuleState[] getSwerveModuleState() {
        return new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
    }

    public SwerveModulePosition[] getSwerveModulePosition() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    public Rotation2d getRotation2d() {
        if(Operating.Constants.USING_GYRO) { 
            return new Rotation2d(gyro.getYaw().getValue()); //Verify
        }
        else {
            return new Rotation2d(0);
        }
    }

    public Pose2d getOdometry() {
        return odometry.getPoseMeters();
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    //Verify
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Drive.Constants.DRIVE_KINEMATICS.toChassisSpeeds(
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        );
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
            getRotation2d(),
            getSwerveModulePosition(),
            pose);
    }

     public void resetPose(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }

    public void resetEncoders() {
        frontLeft.resetDriveEncoder();
        frontRight.resetDriveEncoder();
        backLeft.resetDriveEncoder();
        backRight.resetDriveEncoder();
    }

    public void zeroHeading() {
        if(Operating.Constants.USING_GYRO) gyro.setYaw(0);
    }

    public void stopModules() {
        frontLeft.stopMotors();
        frontRight.stopMotors();
        backLeft.stopMotors();
        backRight.stopMotors();

        for(int i = 0; i < desiredStates.length; i++)
            desiredStates[i].speedMetersPerSecond = 0;
    }

    //Verify
    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                this::getPose,   // Supplier of current robot pose *getPose
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

    public double calculateDistance()
    {
       return getPose().getTranslation().getDistance(Vision.Constants.getHubPose().toPose2d().getTranslation()); 
    }

    

    @Override
    public void periodic() {
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

        Logger.recordOutput("Power/BatteryVoltage", RobotController.getBatteryVoltage());

        double now = Timer.getFPGATimestamp();
        if (now - lastMatchLog > 0.2) {
            lastMatchLog = now;
            Logger.recordOutput("Match/TimeRemaining", DriverStation.getMatchTime());
        }

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

        if(Operating.Constants.USING_VISION) {
            VisionIOInputs inputs = visionIO.getInputs();
            for(int i = 0; i < inputs.cameraPoses.length; i++) {
                if(inputs.cameraTargets[i] != null &&
                   inputs.ambiguity[i] < 0.2 &&
                   inputs.cameraPoses[i].toPose2d().getX() > 0.0 &&
                   inputs.cameraPoses[i].toPose2d().getX() < 16.48 &&
                   inputs.cameraPoses[i].toPose2d().getY() > 0.0 &&
                   inputs.cameraPoses[i].toPose2d().getY() < 8.1 &&
                   Math.abs(inputs.cameraPoses[i].getZ()) < 0.5) {
                    poseEstimator.addVisionMeasurement(inputs.cameraPoses[i].toPose2d(), inputs.timestamps[i]);
                }
            }
        }
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
