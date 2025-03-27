
// EmergencyStopArmCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmEmergencyStopCommand extends Command {
    private final ArmSubsystem arm;

    public ArmEmergencyStopCommand(ArmSubsystem arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

