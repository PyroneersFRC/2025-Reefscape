package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANids;
import frc.robot.Constants.robot;


public class ElevatorSubsystem  extends SubsystemBase{
    private final SparkMax m_rightMotor;
    private final SparkMax m_leftMotor;
    private final AbsoluteEncoder m_encoder;
    private final PIDController m_elevatorPIDController;
    private double m_currentSpeed;

    public ElevatorSubsystem(int rightMotorID, int leftMotorID){
        m_leftMotor = new SparkMax(leftMotorID, MotorType.kBrushed);
        m_rightMotor = new SparkMax(rightMotorID, MotorType.kBrushed);

        m_encoder = m_rightMotor.getAbsoluteEncoder();
            

        m_elevatorPIDController = robot.kPIDElevator;
        m_currentSpeed = 0;
    }

    public void setVoltage() {
        m_leftMotor.set(m_currentSpeed);
        m_rightMotor.set(-m_currentSpeed);
    }

    public Command runElevator(){
        return this.run(this::setVoltage);
    }
    public Command stopElevator(){
        return this.runOnce(() ->m_currentSpeed = 0);
    }
    public Command accelerate(){
        return this.runOnce(() ->m_currentSpeed += 0.1);
    }
    public Command decrease(){
        return this.runOnce(() ->m_currentSpeed -= 0.1);
    }
}
