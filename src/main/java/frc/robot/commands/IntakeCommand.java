// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.KitbotIntakeSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.Intake.IntakeConstants.*;


public final class IntakeCommand extends Command {

  private final KitbotIntakeSubsystem m_intakeSubsystem;
  /** Example static factory for an autonomous command. */
  public IntakeCommand(KitbotIntakeSubsystem intakeSub) {
    this.m_intakeSubsystem = intakeSub;
    addRequirements(m_intakeSubsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.setFeederRoller(SmartDashboard.getNumber("Intaking Intake Roller Value", INTAKE_FEED_VOLT));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
