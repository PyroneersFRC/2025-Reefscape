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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.OutakeConstants;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;

public class OutakeSubsystem extends SubsystemBase{
    private final SparkMax m_motor;
    private final double m_zero = 0;
    private final RelativeEncoder m_encoder;
    private ElevatorSubsystem m_elevatorSubsystemRef;
    



    public OutakeSubsystem(int CanID, ElevatorSubsystem elevatorSubsystem){
        m_motor = new SparkMax(CanID, MotorType.kBrushless);
        m_encoder = m_motor.getEncoder();
        m_elevatorSubsystemRef = elevatorSubsystem;


    }

    @Override
    public void periodic(){

    }


    private void zero(){
        m_motor.setVoltage(m_zero);
    }

    public Command zeroCmd(){
        return this.run(this::zero);
    }

    private void outakeSlow(){
        m_motor.setVoltage(1);
    }
    private void outake(){
        double setPoint = m_encoder.getPosition()+5;
        while(m_encoder.getPosition() < setPoint){
            m_motor.setVoltage(1.5);
        }
        m_motor.setVoltage(0);
        System.out.println("lift 0");
        // m_elevatorSubsystemRef.setLevel(0);
        
    }

    public Command outakeSlowCmd(){
        return this.run(this::outakeSlow);
    }

    public Command outakeCmd(){
        return this.runOnce(this::outake);
    }

}
