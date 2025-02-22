package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;

public class ElevatorModule {
    private final SparkMax m_leftMotor;
    private final SparkMax m_rightMotor;
    private final AbsoluteEncoder m_encoder;
    private double m_zeroPotition;


    public ElevatorModule(int leftMotorID, int rightMotorID){
        m_leftMotor = new SparkMax(leftMotorID, MotorType.kBrushed);
        m_rightMotor = new SparkMax(rightMotorID, MotorType.kBrushed);

        m_encoder = m_leftMotor.getAbsoluteEncoder();
        m_zeroPotition = m_encoder.getPosition();
    }

    public double getPotition(){
        double encoderPotition = - m_encoder.getPosition() + m_zeroPotition;
        return encoderPotition;
    }

    public void setMotorSpeed(double outputSpeed){
        SmartDashboard.putNumber("elevator module/last out speed", -outputSpeed);
        m_leftMotor.setVoltage(-outputSpeed);
        m_rightMotor.setVoltage(-outputSpeed);
    }
}
