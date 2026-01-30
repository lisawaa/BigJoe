package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive.Constants.MotorLocation;
import frc.robot.Constants.Configs;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  private final SparkMax driveMotor;
  private final SparkMax turnMotor;

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turnEncoder;

  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;

  private final MotorLocation motorLocation;
  private final double driveEncoderInverted;

  private double chassisAngularOffset = 0; //update(?)
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());
 
  /*Constructs a REV MAXSwerve Module designed with NEOs, SPARKS MAX, 
    and a Through Bore Encoder.*/
  public SwerveModule(int driveID, int turnID, double offset, boolean inverted, SparkMaxConfig config, MotorLocation location) {
    driveMotor = new SparkMax(driveID, MotorType.kBrushless);
    turnMotor = new SparkMax(turnID, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getAbsoluteEncoder();

    driveController = driveMotor.getClosedLoopController();
    turnController = turnMotor.getClosedLoopController();

    motorLocation = location;

    /*Reset to default configs before applying our own, persisting 
      them to last between power cycles.*/
    driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnMotor.configure(Configs.SwerveModule.TURNING_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); //Add turn config constant

    chassisAngularOffset = offset;
    desiredState.angle = new Rotation2d(turnEncoder.getPosition());
    if(inverted)
      driveEncoderInverted = -1.0;
    else 
      driveEncoderInverted = 1.0;
    driveEncoder.setPosition(0);
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    //Apply chassis offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

    //Optimize the reference state as to not turn more than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(turnEncoder.getPosition()));

    //Command driving and turning SPARKS toward their respective setpoints.
    driveController.setSetpoint(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    turnController.setSetpoint(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    this.desiredState = desiredState;
  }


  //Returns the current module state.
  public SwerveModuleState getState() {
    //Apply chassis offset to the encoder position to get the position relative to the chassis.
    return new SwerveModuleState(
      getDriveVelocity(),
      new Rotation2d(turnEncoder.getPosition() - chassisAngularOffset));
  }

  //Returns the current position of the module.
  public SwerveModulePosition getPosition() {
    //Apply chassis offset to the encoder position to get the position relative to the chassis.
    return new SwerveModulePosition(
        getDrivePosition(),
        new Rotation2d(turnEncoder.getPosition() - chassisAngularOffset));
  }

  //Returns the encoder position of the drive motor in radians.
  public double getDrivePosition() {
    return driveEncoderInverted * driveEncoder.getPosition();
  }

  //Returns the module's drive velocity in m/s.
  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  //Returns the position of the turn motor in radians.
  public double getTurnPosition() {
    return turnEncoder.getPosition();
  }

  //Returns the module's turn velocity in m/s.
  public double getTurnVelocity() {
    return turnEncoder.getVelocity();
  }

  //Zeroes the module's encoders.
  public void resetEncoders() {
    driveEncoder.setPosition(0);
  }

  //Stops the module's motors;
  public void stopMotors() {
    driveMotor.stopMotor();
    turnMotor.stopMotor();
  }

  //Tests motor speed or turns it to a set angle (radians)
  public void testDriveMotors(double speed) {
    driveMotor.set(speed);
  }
  public void testTurnMotors(double position) {
    turnController.setSetpoint(position, ControlType.kPosition);
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber(motorLocation + " driver enoder", getDrivePosition());
    SmartDashboard.putNumber(motorLocation + " driver velocity", getDriveVelocity());
    SmartDashboard.putNumber(motorLocation + " turn encoder", turnEncoder.getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}