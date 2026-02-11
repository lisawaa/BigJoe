package frc.robot.commands;

import frc.robot.subsystems.KitbotShootingSubsystem;

public class KitbotShootingCommand {
    private final KitbotShootingSubsystem shootingSubsystem;

    public KitbotShootingCommand(KitbotShootingSubsystem subsystem) {
        shootingSubsystem = subsystem;
    }

    public void execute() {
        shootingSubsystem.shooting();
    }

    public void stop() {
        shootingSubsystem.stopMotors();
    }
}
