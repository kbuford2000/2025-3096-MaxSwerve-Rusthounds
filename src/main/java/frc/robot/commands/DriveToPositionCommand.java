package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

import java.util.List;

public class DriveToPositionCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final Pose2d targetPose;
    private Trajectory trajectory;
    private final HolonomicDriveController holonomicController;
    private double startTime;

    public DriveToPositionCommand(DriveSubsystem driveSubsystem, Pose2d targetPose) {
        this.driveSubsystem = driveSubsystem;
        this.targetPose = targetPose;
        
        // Create PID controllers for holonomic drive
        PIDController xController = new PIDController(
            AutoConstants.kPXController,
            0,
            AutoConstants.kDXController
        );
        
        PIDController yController = new PIDController(
            AutoConstants.kPYController,
            0,
            AutoConstants.kDYController
        );
        
        // Profiled PID controller for rotation
        ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController,
            0,
            AutoConstants.kDThetaController,
            new TrapezoidProfile.Constraints(
                AutoConstants.kMaxAngularSpeedRadiansPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared
            )
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        // Initialize holonomic drive controller
        this.holonomicController = new HolonomicDriveController(
            xController,
            yController,
            thetaController
        );
        
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // Create trajectory configuration for swerve drive
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared
        )
        .setKinematics(DriveConstants.kDriveKinematics);

        // Generate trajectory from current position to target
        trajectory = TrajectoryGenerator.generateTrajectory(
            driveSubsystem.getPose(),
            List.of(), // No intermediate waypoints
            targetPose,
            config
        );

        startTime = System.currentTimeMillis() / 1000.0;
        
        // Reset the theta controller to the current robot heading
        holonomicController.getThetaController().reset(
            driveSubsystem.getPose().getRotation().getRadians()
        );
    }

    @Override
    public void execute() {
        double currentTime = System.currentTimeMillis() / 1000.0;
        double elapsedTime = currentTime - startTime;

        // Get the desired state from the trajectory
        Trajectory.State desiredState = trajectory.sample(elapsedTime);

        // Calculate chassis speeds using holonomic drive controller
        ChassisSpeeds targetChassisSpeeds = holonomicController.calculate(
            driveSubsystem.getPose(),
            desiredState,
            targetPose.getRotation()
        );

        // Command the drive subsystem with calculated speeds
        driveSubsystem.drive(
            targetChassisSpeeds.vxMetersPerSecond,
            targetChassisSpeeds.vyMetersPerSecond,
            targetChassisSpeeds.omegaRadiansPerSecond,
            false  // Robot relative
            //true    // Closed loop
        );
    }

    @Override
    public boolean isFinished() {
        // Check if we've reached the target position within tolerance
        Pose2d currentPose = driveSubsystem.getPose();
        Translation2d positionError = targetPose.getTranslation()
            .minus(currentPose.getTranslation());

        double rotationError = Math.abs(
            targetPose.getRotation().minus(currentPose.getRotation()).getRadians()
        );

        return positionError.getNorm() < AutoConstants.kPositionToleranceMeters
            && rotationError < AutoConstants.kRotationToleranceRadians;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        driveSubsystem.drive(0, 0, 0, true);
    }
}