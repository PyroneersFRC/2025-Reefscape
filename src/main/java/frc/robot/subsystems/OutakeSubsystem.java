package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Pound;

import javax.lang.model.util.ElementScanner14;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OutakeSubsystem extends SubsystemBase{
    private final SparkMax m_motor;
    private final double m_zero = 0;
    private final RelativeEncoder m_encoder;

    public OutakeSubsystem(int CanID){
        m_motor = new SparkMax(CanID, MotorType.kBrushless);
        m_encoder = m_motor.getEncoder();
    }

    @Override
    public void periodic(){

    }

    public Command zeroCmd(){
        return this.run(() -> m_motor.setVoltage(m_zero));
    }

    private void outake(){
        double setPoint = m_encoder.getPosition()+5;
        while(m_encoder.getPosition() < setPoint){
            m_motor.setVoltage(2);
        }
        m_motor.setVoltage(0);
    }

    public Command outakeSlowCmd(){
        return this.run(() -> m_motor.setVoltage(1));
    }

    public Command outakeCmd(){
        return this.runOnce(this::outake);
    }
    public Command emergencyCmd(){
        return this.run(() -> m_motor.setVoltage(3));
    }
    public Command reverseCmd(){
        return this.run(() -> m_motor.setVoltage(-1.3));
    }
}
