package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANids;
import frc.robot.Constants.robot;


public class ElevatorSubsystem  extends SubsystemBase{
    private final  SparkMax m_rightSparkMax;
    private final  SparkMax m_leftSparkMax;
    private final PIDController m_elevatorPIDController;

    public ElevatorSubsystem(int rightMotorID, int leftMotorID){
        m_leftSparkMax = new SparkMax(leftMotorID, MotorType.kBrushed);
        m_rightSparkMax = new SparkMax(leftMotorID, MotorType.kBrushed);

        m_elevatorPIDController = robot.kPIDElevator;
        
    }
}
