package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANids;


public class SwerveModule {
    private final SparkMax m_turningMotor;
    private final SparkFlex m_driveMotor;

    private final AbsoluteEncoder m_turningEncoder;
    private final RelativeEncoder m_driveEncoder;

    private double m_chassisAngularOffset;

    private final PIDController m_drivePIDController = new PIDController(0, 0, 0);
    private final ProfiledPIDController m_turningProfiledPIDController = new ProfiledPIDController(1, 0, 0, 
                                                                                new TrapezoidProfile.Constraints(5, 5));
    // private final PIDController m_turningPIDController;  // ??

    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.18, 2.28, 0.37);     //0.37
    private final SimpleMotorFeedforward m_turningFeedforward = new SimpleMotorFeedforward(0.4, 0.2, 0);     //

    // SmartDashboard prefixes
    private final String SWERVE;
    private final String SMART_DASHBOARD_PREFIX;

    public double m_driveVoltage = 0;
    public double m_turningVoltage = 0;

    public SwerveModule(int driveMotorId, int turningMotorId, double chassisAngularOffset) {
        m_chassisAngularOffset = chassisAngularOffset;

        m_driveMotor = new SparkFlex(driveMotorId, MotorType.kBrushless);
        m_turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
        
        m_turningEncoder = m_turningMotor.getAbsoluteEncoder();
        m_driveEncoder = m_driveMotor.getEncoder();

        m_turningProfiledPIDController.enableContinuousInput(-Math.PI, Math.PI);


        switch(driveMotorId) {
            case CANids.kFrontLeftDrivingCanId:
                SWERVE = "FrontLeft";
                break;
            case CANids.kFrontRightDrivingCanId:
                SWERVE = "FrontRight";
                break;
            case CANids.kRearLeftDrivingCanId:
                SWERVE = "BackLeft";
                break;
            case CANids.kRearRightDrivingCanId:
                SWERVE = "BackRight";
                break;
            default:
                SWERVE = "Unknown WTF";
                break;
        }

        SMART_DASHBOARD_PREFIX = "SwerveModule/" + SWERVE + "/";
    }   

    public double getDrivePosition() {
        return m_driveEncoder.getPosition();
    }
        
    public double getDriveVelocity() {
        return m_driveEncoder.getVelocity();
    }

    public double getTurningPosition() {
        return m_turningEncoder.getPosition();
    }

    public double getTurningVelocity() {
        return m_turningEncoder.getVelocity();
    }

    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position relative to the chassis.
        return new SwerveModulePosition(
            m_driveEncoder.getPosition(),
            new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition() - m_chassisAngularOffset));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // apply chassis angular offset to the desired state
        desiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));
        // optimize the reference to avoid spinning further than 90 degrees
        desiredState.optimize(new Rotation2d(getTurningPosition()));
        
        double drivePIDOutput = m_drivePIDController.calculate(getDriveVelocity(), desiredState.speedMetersPerSecond);
        double driveFeedforwardOutput = m_driveFeedforward.calculate(desiredState.speedMetersPerSecond);
        
        double turningPIDOutput = m_turningProfiledPIDController.calculate(getTurningPosition(), desiredState.angle.getRadians());
        double turningFeedforwardOutput = m_turningFeedforward.calculate(m_turningProfiledPIDController.getSetpoint().velocity);

        // double driveOutput = /*driveFeedforwardOutput + drivePIDOutput */ m_driveVoltage;
        double driveOutput = driveFeedforwardOutput + drivePIDOutput;
        double turningOutput = turningFeedforwardOutput + turningPIDOutput;

        m_driveMotor.setVoltage(driveOutput);
        m_turningMotor.setVoltage(turningOutput);

        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "drive voltage", driveOutput);
        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "turning voltage", turningOutput);
        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "drive encoder velocity", m_driveEncoder.getVelocity());
        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "drive encoder potition", m_driveEncoder.getPosition());
        
        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "turning encoder velocity", m_turningEncoder.getVelocity());
        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "turning encoder potition", m_turningEncoder.getPosition());
    
        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "turning encoder +pi", m_turningEncoder.getPosition() + (Math.PI/2) % (2*Math.PI));
        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "turning encoder-pi", m_turningEncoder.getPosition() - (Math.PI / 2) % (2*Math.PI));
        
        
    }



    public void stop() {
        m_driveMotor.stopMotor();
        m_turningMotor.stopMotor();
    }

}