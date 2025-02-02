// HomeArmCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmHomeCommand extends Command {
    private final ArmSubsystem arm;

    public ArmHomeCommand(ArmSubsystem arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setPosition(ArmSubsystem.HOME_POSITION);
    }

    @Override
    public boolean isFinished() {
        return arm.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            arm.stop();
        }
    }
}