package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Configs;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Drive.Constants.MotorLocation;
import frc.robot.Constants.IDs;
import frc.robot.Constants.Operating;

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
    private StructArrayPublisher<SwerveModulePosition> publisherPositions = NetworkTableInstance.getDefault().getStructArrayTopic("MyPositions", SwerveModulePosition.struct).publish();
 
    SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        Drive.Constants.DRIVE_KINEMATICS,
        getRotation2d(),
        getSwerveModulePosition());

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

    @Override
    public void periodic() {
        odometry.update(getRotation2d(), getSwerveModulePosition());

        if(Operating.Debugging.DRIVE_DEBUG) {
            updateSmartDashboard();
            //updateWheelPositions - add for extra debugging functionality
            publisherDesieredStates.set(desiredStates);
            publisherActualStates.set(getSwerveModuleState());
            publisherPositions.set(getSwerveModulePosition());

            frontLeft.updateSmartDashboard();
            frontRight.updateSmartDashboard();
            backLeft.updateSmartDashboard();
            backRight.updateSmartDashboard();
        }
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Robot Heading Yaw", getRotation2d().getDegrees());
        if(Operating.Constants.USING_GYRO) {
            SmartDashboard.putNumber("Roll", gyro.getRoll().getValue().in(Units.Degrees));
            SmartDashboard.putNumber("Pitch", gyro.getPitch().getValue().in(Units.Degrees));
        }
    }

    //Might want to add getWheelRotationSupplier() and any test methods if needed
}