// ManualArmCommand.java
package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmManualCommand extends Command {
    private final ArmSubsystem arm;
    private final DoubleSupplier speedSupplier;
    private static final double DEADBAND = 0.1;

    public ArmManualCommand(ArmSubsystem arm, DoubleSupplier speedSupplier) {
        this.arm = arm;
        this.speedSupplier = speedSupplier;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        double speed = speedSupplier.getAsDouble();
        if (Math.abs(speed) < DEADBAND) {
            speed = 0;
        }
        arm.setManualSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}
