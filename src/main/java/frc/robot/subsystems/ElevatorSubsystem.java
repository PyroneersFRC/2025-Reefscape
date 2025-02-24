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

    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(0.7, 1.4, 0.3);   // kg htan 1.4 kai htan kalo!!

    private final ElevatorModule m_elevator;

    private final String SMART_DASHBOARD_PREFIX = "Elevator Subsystem/";

    private double m_motorSpeed = 1.2;

    public ElevatorSubsystem(){
        m_elevator = new ElevatorModule(CANids.kLeftElevatorCanId, CANids.kRightElevatorCanId);
    }
    

    private void potitionSafety(){
        double pos = m_elevator.getPotition();
        if(pos > 5  || pos < -0.1){
            System.out.println("\n\n\n\n\n\n pos " + pos + "\n\n\n\n\n\n\n");
            this.setDesiredState(0);
        }
    }

    @Override
    public void periodic(){;
        // SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "encoder/potition", m_elevator.getPotition()); 
        // SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "encoder/velocity", m_elevator.getVelocity()); 
        // SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "motor speed", m_motorSpeed); 

        potitionSafety(); 

        m_elevator.setMotorSpeed(m_motorSpeed);
    }
    
    private void printSetpoint(TrapezoidProfile.State setpoint){
        // SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "profiled pid Setpoint/position", setpoint.position);
        // SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "profiled pid Setpoint/velocity", setpoint.velocity);
    }
 
    public void setDesiredState(double desiredState){
        double PIDOutput = m_PIDController.calculate(m_elevator.getPotition(), desiredState);
        double feedforwardOutput = m_feedforward.calculate(m_PIDController.getSetpoint().velocity);
        double outputVoltage = PIDOutput + feedforwardOutput;

        if(m_PIDController.getSetpoint().position == 0){
            outputVoltage = 0;
        }

        // SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "output voltage/total output", outputVoltage);
        // SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "output voltage/pid output", PIDOutput);
        // SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "output voltage/feedforward output", feedforwardOutput);
        printSetpoint(m_PIDController.getSetpoint());
        
        if(Math.abs(outputVoltage) > 7){
            throw new RuntimeException("megalo vlotage elevator: " + outputVoltage);
            // System.out.println("\n\n\n\n\n\n eeeeeeeeeeeeeeeeeeeeeeee \n\n\n\n\n");
            // outputVoltage=1;
        }

        // SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "/output voltage/actual total output", outputVoltage);
        m_elevator.setMotorSpeed(outputVoltage);
    }

    public Command runElevator(){
        return this.run(() -> setDesiredState(1.6));    // TODO fix
    }

    public Command setLevel(int level){
        double setPoint;
        if(level == 0){
            setPoint = 0;
        } else if(level == 1){
            setPoint = 2.6;
        } else if(level == 2){
            setPoint = 3.4;
        } else if(level == 3){
            setPoint = 4.8;
        } else {
            throw new RuntimeException("Invalid level (" + level + ")");
        }

        return this.run(() -> setDesiredState(setPoint));
    }


    public void stop(){
        m_elevator.setMotorSpeed(0);
    }
    public Command stopCmd(){
        return this.runOnce(this::stop);
    }
 

    public Command accelerateCmd(){
        return this.runOnce(() -> { m_motorSpeed += 0.1; 
                                    m_elevator.setMotorSpeed(m_motorSpeed);});
    }

    public Command decellerateCmd(){
        return this.runOnce(() -> { m_motorSpeed -= 0.1;
                                    m_elevator.setMotorSpeed(m_motorSpeed);});
        }
}

