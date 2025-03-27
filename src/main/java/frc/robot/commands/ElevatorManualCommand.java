// ManualElevatorCommand.java
package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorManualCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final DoubleSupplier speedSupplier;
    private static final double DEADBAND = 0.1;

    public ElevatorManualCommand(ElevatorSubsystem elevator, DoubleSupplier speedSupplier) {
        this.elevator = elevator;
        this.speedSupplier = speedSupplier;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        double speed = speedSupplier.getAsDouble();
        if (Math.abs(speed) < DEADBAND) {
            speed = 0;
        }
        elevator.setManualSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}
