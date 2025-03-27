package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.commands.DriveToPositionCommand;
import frc.robot.commands.ElevatorSetPositionCommand;
import frc.robot.commands.ArmSetPositionCommand;

public class AutonomousRoutines {
    private final DriveSubsystem driveSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final ArmSubsystem armSubsystem;
    private final SendableChooser<Command> autoChooser;

    // Starting position
    private final Pose2d startingPose = new Pose2d(1.0, 1.0, new Rotation2d(0));

    // Hexagon points (adjust coordinates as needed for your field)
    //private final Pose2d[] hexagonPoints = {
      //  new Pose2d(3.0, 1.0, new Rotation2d(0)),          // Point 1 (0 degrees)
        //new Pose2d(2.5, 2.0, new Rotation2d(Math.PI/3)),  // Point 2 (60 degrees)
        //new Pose2d(1.5, 2.0, new Rotation2d(2*Math.PI/3)), // Point 3 (120 degrees)
        //new Pose2d(1.0, 1.0, new Rotation2d(Math.PI)),     // Point 4 (180 degrees)
        //new Pose2d(1.5, 0.0, new Rotation2d(-2*Math.PI/3)), // Point 5 (240 degrees)
        //new Pose2d(2.5, 0.0, new Rotation2d(-Math.PI/3))   // Point 6 (300 degrees)
    //};

    public AutonomousRoutines(
            DriveSubsystem driveSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            ArmSubsystem armSubsystem) {
        
        this.driveSubsystem = driveSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;
        
        autoChooser = new SendableChooser<>();
        setupAutoRoutines();
        
        // Add to Shuffleboard
        ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
        autoTab.add("Auto Routine", autoChooser)
            .withSize(2, 1)
            .withPosition(0, 0);
    }

    private void setupAutoRoutines() {
        // Simple drive forward routine
        autoChooser.setDefaultOption("Drive Forward",
            new SequentialCommandGroup(
                new DriveToPositionCommand(driveSubsystem, new Pose2d(3.0, 1.0, new Rotation2d(0)))
            )
        );

        // Deliver gamepiece routine
        autoChooser.addOption("Deliver Gamepiece",
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new ElevatorSetPositionCommand(elevatorSubsystem, ElevatorSubsystem.POSITION_4),
                    new ArmSetPositionCommand(armSubsystem, ArmSubsystem.POSITION_1),
                    new WaitCommand(2),
                    new DriveToPositionCommand(driveSubsystem, new Pose2d(3.0, 1.0, new Rotation2d(0)))
                )
                
            )
        );

        // Add hexagon point routines
       // for (int i = 0; i < hexagonPoints.length; i++) {
         //   final String routineName = String.format("Hexagon Point %d", i + 1);
           // autoChooser.addOption(routineName,
             //   new SequentialCommandGroup(
               //     new ParallelCommandGroup(
                 //       new DriveToPositionCommand(driveSubsystem, hexagonPoints[i]),
                   //     new ElevatorSetPositionCommand(elevatorSubsystem, 4),
                     //   new ArmSetPositionCommand(armSubsystem, 3)
                    //)
               // )
            //);
    //    }
    }

    public Command getSelectedAutonomousCommand() {
        return autoChooser.getSelected();
    }

    // Reset robot to starting position
    public void resetRobotPose() {
        driveSubsystem.resetOdometry(startingPose);
    }
}

