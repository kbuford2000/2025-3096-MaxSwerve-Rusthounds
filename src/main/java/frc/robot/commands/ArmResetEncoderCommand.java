// ResetArmEncoderCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmResetEncoderCommand extends Command {
    private final ArmSubsystem arm;

    public ArmResetEncoderCommand(ArmSubsystem arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.resetEncoder();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
