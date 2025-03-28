package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax armMotor;
    private final SparkMaxConfig armMotorConfig;
    private final RelativeEncoder encoder;
    private final PIDController pidController;

    // Define arm positions in encoder counts (degrees)
    public static final double HOME_POSITION = -2.0;
    public static final double POSITION_1 = 2.0;
    public static final double POSITION_2 = 12.0;
    public static final double POSITION_3 = 17.0;

    // PID constants
    private static final double kP = 0.05;
    private static final double kI = 0.0;
    private static final double kD = 0.001;
    private static final double kG = 0.2; // Gravity compensation constant

    // Motor constants
    private static final int MOTOR_ID = 23;
    private static final double MAX_OUTPUT = 0.25; //was 0.5
    private static final double MIN_OUTPUT = -0.25;//Was 0.5
    private static final double MAX_ANGLE = 20.0;
    private static final double MIN_ANGLE = -5.0;
    private static final double RAMP_RATE = 0.07; // Was 0.25

    private double setPoint = HOME_POSITION;

    public ArmSubsystem() {
        // Initialize motor
        armMotor = new SparkMax(MOTOR_ID, MotorType.kBrushless);

   // Initialize master motor

        armMotorConfig = new SparkMaxConfig();
        armMotorConfig.idleMode(IdleMode.kBrake);
        // Set motor output limits and current limits
        armMotorConfig.smartCurrentLimit(40);
        // set ramp rate in seconds from neutral to full output
        armMotorConfig.closedLoopRampRate(RAMP_RATE);
        armMotor.configure(armMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);


       
        
        // Initialize encoder
        encoder = armMotor.getEncoder();
        encoder.setPosition(0);

        // Initialize PID controller
        pidController = new PIDController(kP, kI, kD);
        pidController.setTolerance(1.0); // 1 degree tolerance

        
    }

    @Override
    public void periodic() {
        // Calculate gravity compensation based on arm angle
        double gravityCompensation = kG * Math.cos(Math.toRadians(encoder.getPosition()));
        
        // Calculate PID output
        double output = pidController.calculate(encoder.getPosition(), setPoint);
        output = Math.max(MIN_OUTPUT, Math.min(output + gravityCompensation, MAX_OUTPUT));
        
        armMotor.set(output);

        // Update dashboard
        SmartDashboard.putNumber("Arm Position", encoder.getPosition());
        SmartDashboard.putNumber("Arm Target", setPoint);
        SmartDashboard.putNumber("Arm Output", output);
        SmartDashboard.putNumber("Arm Current", armMotor.getOutputCurrent());
    }

    public void setPosition(double position) {
        setPoint = Math.max(MIN_ANGLE, Math.min(position, MAX_ANGLE));
    }

    public void setManualSpeed(double speed) {
        double currentPosition = encoder.getPosition();
        
        // Check soft limits
        if ((currentPosition >= MAX_ANGLE && speed > 0) ||
            (currentPosition <= MIN_ANGLE && speed < 0)) {
            speed = 0;
        }
        
        // Apply gravity compensation
        double gravityCompensation = kG * Math.cos(Math.toRadians(currentPosition));
        double finalOutput = speed + gravityCompensation;
        
        // Apply output limits
        finalOutput = Math.max(MIN_OUTPUT, Math.min(finalOutput, MAX_OUTPUT));
        
        armMotor.set(finalOutput);
    }

    public void stop() {
        armMotor.set(0);
        setPoint = getCurrentPosition(); // Maintain current position after stop
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

    public void resetEncoder() {
        encoder.setPosition(0);
    }

    public double getCurrentPosition() {
        return encoder.getPosition();
    }

    public boolean isAtUpperLimit() {
        return getCurrentPosition() >= MAX_ANGLE;
    }

    public boolean isAtLowerLimit() {
        return getCurrentPosition() <= MIN_ANGLE;
    }
}
