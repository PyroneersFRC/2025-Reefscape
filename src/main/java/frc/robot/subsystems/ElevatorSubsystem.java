package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANids;
import frc.robot.Constants.elevatorConstants;
import frc.robot.Constants.robot;


public class ElevatorSubsystem  extends SubsystemBase{
    private final ProfiledPIDController m_PIDController = new ProfiledPIDController(
            elevatorConstants.kP
        ,elevatorConstants.kI
        ,elevatorConstants.kD
        , elevatorConstants.kelevatorConstraints);

    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(0.425, 1, 0);

    private final ElevatorModule m_elevator;

    private final String SMART_DASHBOARD_PREFIX = "Elevator Subsystem/";

    private double m_motorSpeed = 1.2;

    public ElevatorSubsystem(){
        m_elevator = new ElevatorModule(CANids.kLeftElevatorCanId, CANids.kRightElevatorCanId);
    }
    

    private void potitionSafety(){
        double pos = m_elevator.getPotition();
        if(pos > 1.5 /* || pos < -0.1*/){
            System.out.println("\n\n\n\n\n\n pos " + pos + "\n\n\n\n\n\n\n");
            this.stop();
            throw new RuntimeException("paixtike malakia");
        }
    }

    @Override
    public void periodic(){;
        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "encoder/potition", m_elevator.getPotition()); 
        // SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "encoder/real position", m_elevator.m_encoder.getPosition()); 
        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "motor speed", m_motorSpeed); 


        potitionSafety(); 


        // m_elevator.setMotorSpeed(m_motorSpeed);
    }
    
    private void printSetpoint(TrapezoidProfile.State setpoint){
        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "profiled pid Setpoint/position", setpoint.position);
        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "profiled pid Setpoint/velocity", setpoint.velocity);
    }
 
    public void setDesiredState(double desiredState){
        double PIDOutput = m_PIDController.calculate(m_elevator.getPotition(), desiredState);
        double feedforwardOutput = m_feedforward.calculate(m_PIDController.getSetpoint().velocity);
        double outputVoltage = PIDOutput + feedforwardOutput;

        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "output voltage/total output", outputVoltage);
        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "output voltage/pid output", PIDOutput);
        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "output voltage/feedforward output", feedforwardOutput);
        printSetpoint(m_PIDController.getSetpoint());
        
        if(Math.abs(outputVoltage) > 3){
            this.stop();
            throw new RuntimeException("paixtike malakia 2");
        }

        m_elevator.setMotorSpeed(outputVoltage);
    }

    public Command runElevator(){
        return this.run(() -> setDesiredState(0.8));    // TODO fix
    }

    public void stop(){
        m_elevator.setMotorSpeed(0);
    }
    public Command stopCmd(){
        return this.runOnce(this::stop);
    }
 
 

    public Command accelerateCmd(){
        return this.runOnce(() -> { m_motorSpeed += 0.01; 
                                    m_elevator.setMotorSpeed(m_motorSpeed);});

    }

    public Command decellerateCmd(){
        return this.runOnce(() -> { m_motorSpeed -= 0.01;
                                    m_elevator.setMotorSpeed(m_motorSpeed);});
        }
}

