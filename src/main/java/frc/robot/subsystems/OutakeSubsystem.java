package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OutakeConstants;
import frc.robot.Constants.robot;

public class OutakeSubsystem extends SubsystemBase{
    private final SparkMax m_motor;
    private final SimpleMotorFeedforward m_Feedforward;

    public
     OutakeSubsystem(int CanID){
        m_motor = new SparkMax(CanID, MotorType.kBrushed);
        m_Feedforward = new SimpleMotorFeedforward(OutakeConstants.ks, OutakeConstants.kv);
    }

    private void runOutake(){

    }
}
