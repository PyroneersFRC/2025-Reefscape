package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OutakeConstants;
import frc.robot.Constants.robot;

public class OutakeSubsystem extends SubsystemBase{
    private final SparkMax m_motor;
    private final RelativeEncoder m_encoder;
    private final SimpleMotorFeedforward m_Feedforward;

    enum SetPoints{
        
        
    }

    public OutakeSubsystem(int CanID){
        m_motor = new SparkMax(CanID, MotorType.kBrushless);
        m_encoder = m_motor.getEncoder();
        m_Feedforward = new SimpleMotorFeedforward(OutakeConstants.ks, OutakeConstants.kv);
    }

    

    private void runOutake(double setPoint){
        double outputVoltage = m_Feedforward.calculate(setPoint);


    }
}
