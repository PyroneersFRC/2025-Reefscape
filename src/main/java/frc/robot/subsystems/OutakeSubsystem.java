package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OutakeSubsystem extends SubsystemBase{
    private final SparkMax m_motor;
    // private final RelativeEncoder m_encoder;
    // private final SimpleMotorFeedforward m_Feedforward;
    private final double m_zero = 0;
    
        
        


    public OutakeSubsystem(int CanID){
        m_motor = new SparkMax(CanID, MotorType.kBrushless);
        //m_encoder = m_motor.getEncoder();
        //m_Feedforward = new SimpleMotorFeedforward(OutakeConstants.ks, OutakeConstants.kv);
    
    }

    private void zero(){
        m_motor.setVoltage(m_zero);
    }

    public Command zeroCmd(){
        return this.run(this::zero);
    }

    private void intake(){
        m_motor.setVoltage(2);
    }
    private void outake(){
        m_motor.setVoltage(-2);
    }

    public Command intakeCmd(){
        return this.run(this::intake);
    }

    public Command outakeCmd(){
        return this.run(this::outake);
    }
}
