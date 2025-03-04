package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.OutakeConstants;

public class OutakeSubsystem extends SubsystemBase{
    private final SparkMax m_motor;
    private final RelativeEncoder m_encoder;
    private double m_setPoint = 0;

    public OutakeSubsystem(int CanID){
        m_motor = new SparkMax(CanID, MotorType.kBrushless);
        m_encoder = m_motor.getEncoder();
    }

    @Override
    public void periodic(){

    }

    public Command zeroCmd(){
        return this.runOnce(() -> m_motor.setVoltage(0));
    }

    // private void outake(){
    //     double setPoint = m_encoder.getPosition() + 5;
    //     while(m_encoder.getPosition() < setPoint){
    //         m_motor.setVoltage(2);
    //     }
    //     m_motor.setVoltage(0);
    // }

    public Command outakeSlowCmd(){
        return this.run(() -> m_motor.setVoltage(OutakeConstants.kSlowVoltage));
    }

    public Command outakeCmd(){
        // return Commands.sequence(
        //     new InstantCommand(() -> m_motor.setVoltage(2)),
        //     // new InstantCommand(() -> {System.out.println("NIGGA1");}),
        //     new WaitCommand(1),
        //     // new InstantCommand(() -> m_setPoint = m_encoder.getPosition() + 5),
        //     // new InstantCommand(() -> {System.out.println(m_setPoint);}),
        //     // new InstantCommand(() -> {System.out.println(m_encoder.getPosition());}),
            // new RunCommand(() -> {}).until(() -> {System.out.println(m_setPoint+" "+m_encoder.getPosition()); return m_encoder.getPosition() >= m_setPoint;}),
        //     new InstantCommand(() -> m_motor.setVoltage(0))
        // );

        // return this.run(() -> m_motor.setVoltage(2)).until()
        return this.runOnce(() -> m_motor.setVoltage(OutakeConstants.kOutakeVoltage));
    }
    
    public Command emergencyCmd(){
        return this.run(() -> m_motor.setVoltage(OutakeConstants.kEmergencyVoltage));
    }
    public Command reverseCmd(){
        return this.run(() -> m_motor.setVoltage(OutakeConstants.kReverseVoltage));
    }
    
}