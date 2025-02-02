// SetArmPositionCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmSetPositionCommand extends Command {
    private final ArmSubsystem arm;
    private final double targetPosition;

    public ArmSetPositionCommand(ArmSubsystem arm, double targetPosition) {
        this.arm = arm;
        this.targetPosition = targetPosition;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setPosition(targetPosition);
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