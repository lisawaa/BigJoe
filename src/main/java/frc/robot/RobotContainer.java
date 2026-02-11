// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OI;
import frc.robot.Constants.Operating;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private DriveSubsystem driveSub;
  private VisionSubsystem visionSub;
  private ShooterSubsystem shooterSub;

  private final CommandXboxController controller =
      new CommandXboxController(OI.Constants.DRIVE_CONTROLLER_PORT);
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    initSubystems();

    if(Operating.Constants.USING_DRIVE){
      autoChooser = AutoBuilder.buildAutoChooser("1");
      //Add paths here
      autoChooser.addOption("1", new PathPlannerAuto("1"));      
      autoChooser.addOption("2", new PathPlannerAuto("2")); 
      SmartDashboard.putData("Auto Mode", autoChooser);
    } else {
      autoChooser = null;
    }

    configureBindings();

    FollowPathCommand.warmupCommand().schedule();
  }

  public void initSubystems() {
    if(Operating.Constants.USING_VISION) {
      visionSub = new VisionSubsystem();
    }
    if(Operating.Constants.USING_DRIVE) {
      driveSub = new DriveSubsystem(Optional.ofNullable(visionSub));
      driveSub.setDefaultCommand(new RunCommand(
        () -> {
          double y = OI.Constants.DRIVER_AXIS_Y_INVERTED * MathUtil.applyDeadband(controller.getRawAxis(OI.Constants.DRIVER_AXIS_Y), OI.Constants.DRIVE_DEADBAND);
          double x = OI.Constants.DRIVER_AXIS_X_INVERTED * MathUtil.applyDeadband(controller.getRawAxis(OI.Constants.DRIVER_AXIS_X), OI.Constants.DRIVE_DEADBAND);
          double rot = OI.Constants.DRIVER_AXIS_ROT_INVERTED * MathUtil.applyDeadband(controller.getRawAxis(OI.Constants.DRIVER_AXIS_ROT), OI.Constants.DRIVE_DEADBAND);

          // Record operator inputs with the project logger
          Logger.recordOutput("Operator/Drive/Y", y);
          Logger.recordOutput("Operator/Drive/X", x);

          //Add logging for buttons

          Logger.recordOutput("Operator/Drive/Rot", rot);
          Logger.recordOutput("Operator/Drive/LeftTrigger", controller.leftTrigger().getAsBoolean());
          Logger.recordOutput("Operator/Drive/RightTrigger", controller.rightTrigger().getAsBoolean());

          driveSub.drive(y, x, rot, true, "Default / Field Oriented"); // CHECK LATER
        },
        driveSub));
    }
    if(Operating.Constants.USING_SHOOTER) {
      shooterSub = new ShooterSubsystem();
      shooterSub.setDefaultCommand(new RunCommand(
        () -> {
          // Record shooter-related operator inputs
          Logger.recordOutput("Operator/Shooter/RightTrigger", controller.rightTrigger().getAsBoolean());
          // Default behavior: hold RPM at 0 when not commanded
          shooterSub.setRPM(0);
        }, shooterSub));
    } 
    // extend if-else chain for other subsystems
  }

  private void configureBindings() {
  int preset = 0;
    switch (preset) {
      default:
        if(Operating.Constants.USING_VISION) {
            controller.leftTrigger().whileTrue(new RunCommand(() -> driveSub.driveAligned(
                  OI.Constants.DRIVER_AXIS_Y_INVERTED * MathUtil.applyDeadband(controller.getRawAxis(OI.Constants.DRIVER_AXIS_Y), OI.Constants.DRIVE_DEADBAND),
                  OI.Constants.DRIVER_AXIS_X_INVERTED * MathUtil.applyDeadband(controller.getRawAxis(OI.Constants.DRIVER_AXIS_X), OI.Constants.DRIVE_DEADBAND),
                  true,
                  "Aiming / Field Oriented"
            ), 
            driveSub));
        }
        if(Operating.Constants.USING_SHOOTER) {
          controller.rightTrigger().whileTrue(new RunCommand(() -> shooterSub.setRPM(1000), shooterSub));
        }
        if(false && Operating.Constants.USING_DRIVE) {
            // 1. Define the target and constraints
            Pose2d targetPose = new Pose2d(10, 5, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(180));
            
            PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), 
                Units.degreesToRadians(720));

            // 2. Bind to a button (e.g., the B button)
            controller.b().whileTrue(
                AutoBuilder.pathfindToPose(
                    targetPose,
                    constraints,
                    0.0 // Goal end velocity
                )
            );
        }
        break;
    }    
}

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    if(Operating.Constants.USING_DRIVE){
      return autoChooser.getSelected();
    }
    else 
      return null;
  }
}
