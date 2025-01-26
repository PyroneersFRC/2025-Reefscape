package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.robot;


public class SwerveModule {
    private final SparkMax m_turningMotor;
    private final SparkFlex m_driveMotor;

    private final AbsoluteEncoder m_turningEncoder;
    private final RelativeEncoder m_driveEncoder;

    private double m_chassisAngularOffset;

    private final PIDController m_turningPIDcontroller;

    public SwerveModule(int driveMotorId, int turningMotorId, double chassisAngularOffset) {
        m_chassisAngularOffset = chassisAngularOffset;

        m_driveMotor = new SparkFlex(driveMotorId, MotorType.kBrushless);
        m_turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
        
        m_turningEncoder = m_turningMotor.getAbsoluteEncoder();
        m_driveEncoder = m_driveMotor.getEncoder();

        m_turningPIDcontroller = new PIDController(robot.kPTurning,robot.kITurning,robot.kDTurning);
        m_turningPIDcontroller.enableContinuousInput(Math.PI, Math.PI);
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
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModulePosition(
            m_driveEncoder.getPosition(),
            new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    }

    

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition() - m_chassisAngularOffset));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // apply chassis angular offset to the desired state
        SwerveModuleState correctedState = new SwerveModuleState();
        correctedState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        // coreectedState.speedMetersPerSecond = 0 
        correctedState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));
        // optimize the reference to avoid spinning further than 90 degrees
        correctedState.optimize(new Rotation2d(getTurningPosition()));        
        
        // Command driving and turnig SPAKRS towards their perspective setpoints
        // // TODO Check if correct, leftover from yt
        m_driveMotor.set(correctedState.speedMetersPerSecond / robot.kPhysicalMaxSpeedMetersPerSecond);
        m_turningMotor.set(m_turningPIDcontroller.calculate(getTurningPosition(), correctedState.angle.getRadians()));
    }

    public void stop() {
        m_driveMotor.stopMotor();
        m_turningMotor.stopMotor();
    }

}