// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OI;
import frc.robot.Constants.Operating;
import frc.robot.subsystems.DriveSubsystem;
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

  private final CommandXboxController controller =
      new CommandXboxController(OI.Constants.DRIVE_CONTROLLER_PORT);
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    initSubystems();

    if(Operating.Constants.USING_DRIVE){
      autoChooser = AutoBuilder.buildAutoChooser("Tests");
      //Add paths here
      //autoChooser.addOption("Forward Right", new PathPlannerAuto("Forward Right")); <- example
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
        () -> driveSub.drive(
                  OI.Constants.DRIVER_AXIS_Y_INVERTED * MathUtil.applyDeadband(controller.getRawAxis(OI.Constants.DRIVER_AXIS_Y), OI.Constants.DRIVE_DEADBAND),
                  OI.Constants.DRIVER_AXIS_X_INVERTED * MathUtil.applyDeadband(controller.getRawAxis(OI.Constants.DRIVER_AXIS_X), OI.Constants.DRIVE_DEADBAND),
                  OI.Constants.DRIVER_AXIS_ROT_INVERTED * MathUtil.applyDeadband(controller.getRawAxis(OI.Constants.DRIVER_AXIS_ROT), OI.Constants.DRIVE_DEADBAND), 
                  true,
                  "Default / Field Oriented"
        ),
        driveSub));
    } 
    // extend if-else chain for other subsystems
  }

  private void configureBindings() {
    int preset = 0;
    switch (preset){ //Add other controller schemes later
      default: //Main controller scheme
        if(Operating.Constants.USING_VISION) {
            controller.leftTrigger().whileTrue(new RunCommand(() -> driveSub.driveAligned(
                  OI.Constants.DRIVER_AXIS_Y_INVERTED * MathUtil.applyDeadband(controller.getRawAxis(OI.Constants.DRIVER_AXIS_Y), OI.Constants.DRIVE_DEADBAND),
                  OI.Constants.DRIVER_AXIS_X_INVERTED * MathUtil.applyDeadband(controller.getRawAxis(OI.Constants.DRIVER_AXIS_X), OI.Constants.DRIVE_DEADBAND),
                  true,
                  "Aiming / Field Oriented"
            ), 
            driveSub));
        }
        break;
    }    
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    if(Operating.Constants.USING_DRIVE)
      return autoChooser.getSelected();
    else 
      return null;
  }
}
