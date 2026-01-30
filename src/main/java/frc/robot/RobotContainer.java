// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final CommandXboxController controller =
      new CommandXboxController(Shared.OperatorConstants.DRIVER_CONTROLLER_PORT);
  
  private DriveSubsystem driveSub;
  private KitbotIntakeSubsystem intakeSub;
  private KitbotShootingSubsystem shootSub;

  public RobotContainer() {
    driveSub = new DriveSubsystem();
    intakeSub = new KitbotIntakeSubsystem();
    shootSub = new KitbotShootingSubsystem();
     
    configureBindings();
  }

  private void configureBindings() {
      driveSub.setDefaultCommand(new RunCommand(
        () -> driveSub.drive(
                  OI.Constants.DRIVER_AXIS_Y_INVERTED * MathUtil.applyDeadband(controller.getRawAxis(OI.Constants.DRIVER_AXIS_Y), OI.Constants.DRIVE_DEADBAND),
                  OI.Constants.DRIVER_AXIS_X_INVERTED * MathUtil.applyDeadband(controller.getRawAxis(OI.Constants.DRIVER_AXIS_X), OI.Constants.DRIVE_DEADBAND),
                  OI.Constants.DRIVER_AXIS_ROT_INVERTED * MathUtil.applyDeadband(controller.getRawAxis(OI.Constants.DRIVER_AXIS_ROT), OI.Constants.DRIVE_DEADBAND), 
                  true,
                  "Default / Field Oriented"
        ),
        driveSub));  
      controller.a().whileTrue(new RunCommand(
        () -> intakeSub.intake(), intakeSub));
      controller.b().whileTrue(new RunCommand(
        () -> intakeSub.reverse(), intakeSub));
      controller.y().whileTrue(new RunCommand(
        () -> shootSub.shooting(), intakeSub));
  }

  public void configurePathPlanner() {
    return;
  }

  
}
