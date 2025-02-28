package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Pound;

import javax.lang.model.util.ElementScanner14;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OutakeConstants;

public class OutakeSubsystem extends SubsystemBase{
    private final SparkMax m_motor;
    private final double m_zero = 0;
    private final RelativeEncoder m_encoder;
    private double position;
    private double setPoint;
    
        
        


    public OutakeSubsystem(int CanID){
        m_motor = new SparkMax(CanID, MotorType.kBrushless);
        m_encoder = m_motor.getAlternateEncoder();
    }

    @Override
    public void periodic(){
        position = m_encoder.getPosition();
        setPoint = position +1;
        SmartDashboard.putNumber("encoder potition", position);



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
        if (position<setPoint){
            m_motor.setVoltage(-2);
        }
        else if (position>=setPoint){
            m_motor.setVoltage(0);
        }
        else{
            System.out.println("\n\n\n\n\n\n nigggggga \n\n\n\n\n");
        }
    }

    public Command intakeCmd(){
        return this.run(this::intake);
    }

    public Command outakeCmd(){
        return this.run(this::outake);
    }

}
