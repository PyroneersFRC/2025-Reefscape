package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;

public class ElevatorModule {
    private final SparkMax m_leftMotor;
    private final SparkMax m_rightMotor;
    private final RelativeEncoder m_encoder;
    private double m_zeroPotition;

    private String SMART_DASHBOARD_PREFIX = "Elevator Module/";

    public ElevatorModule(int leftMotorID, int rightMotorID){
        m_leftMotor = new SparkMax(leftMotorID, MotorType.kBrushed);
        m_rightMotor = new SparkMax(rightMotorID, MotorType.kBrushed);

        m_encoder = m_leftMotor.getEncoder();         
        m_zeroPotition = m_encoder.getPosition();
    }

    public double getPotition(){
        double encoderPotition = - m_encoder.getPosition() + m_zeroPotition;
        return encoderPotition;
    }

    public double getVelocity(){
        return m_encoder.getVelocity();
    }

    public void setMotorSpeed(double outputSpeed){
        outputSpeed *= -1;
        // SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "last out speed", outputSpeed);
        m_leftMotor.setVoltage(outputSpeed);
        m_rightMotor.setVoltage(outputSpeed);
    }
}
