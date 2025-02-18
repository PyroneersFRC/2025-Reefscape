package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANids;
import frc.robot.Constants.elevatorConstants;
import frc.robot.Constants.robot;


public class ElevatorSubsystem  extends SubsystemBase{
    private final ProfiledPIDController m_PIDController;

    private final ElevatorModule m_elevator;

    private final String SMART_DASHBOARD_PREFIX = "Elevator Subsystem/";

    private double m_motorSpeed = 0;

    public ElevatorSubsystem(){
        m_PIDController = new ProfiledPIDController(
            elevatorConstants.kP
        ,elevatorConstants.kI
        ,elevatorConstants.kD
        , elevatorConstants.kelevatorConstraints);

        m_elevator = new ElevatorModule(CANids.kLeftElevatorCanId, CANids.kRightElevatorCanId);

    }


    @Override
    public void periodic(){;
        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "encoder/potition", m_elevator.getPotition()); 
        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "motor speed", m_motorSpeed); 

        
        if(m_elevator.getPotition() > 1 || m_elevator.getPotition() < -0.1){
            this.stop();
        }
    }

 
    public void setDesiredState(){
        double outputVoltage =  m_PIDController.calculate(m_elevator.getPotition(), 0.5);
        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX+ "0.5", 0.5);
        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "output voltage", outputVoltage);
        m_elevator.setMotorSpeed(outputVoltage);

    }

    public Command runElevator(){
        return this.run(this::setDesiredState);
    }

    public void stop(){
        m_elevator.setMotorSpeed(0);
    }
    public Command stopCmd(){
        return this.runOnce(this::stop);
    }
 
 

    public Command accelerateCmd(){
        return this.runOnce(() -> {m_motorSpeed += 0.5; 
                                    m_elevator.setMotorSpeed(m_motorSpeed);});

    }

    public Command decellerateCmd(){
        return this.runOnce(() -> { m_motorSpeed -= 0.5;
                                    m_elevator.setMotorSpeed(m_motorSpeed);});
        }
    // public Command stopCmd(){
    //     return this.runOnce(() -> m_motorSpeed = 0);
    // }
}

