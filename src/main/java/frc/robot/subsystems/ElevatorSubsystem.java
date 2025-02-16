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
    private final SparkMax m_rightMotor;
    private final SparkMax m_leftMotor;
    private final AbsoluteEncoder m_encoder;
    private final ProfiledPIDController m_PIDController;
    private final ElevatorFeedforward m_Feedforward;
    private double m_motorSpeed = 0;

    public ElevatorSubsystem(int rightMotorID, int leftMotorID){
        m_leftMotor = new SparkMax(leftMotorID, MotorType.kBrushed);
        m_rightMotor = new SparkMax(rightMotorID, MotorType.kBrushed);

        m_encoder = m_leftMotor.getAbsoluteEncoder();
        m_PIDController = new ProfiledPIDController(
            elevatorConstants.kP
        ,elevatorConstants.kI
        ,elevatorConstants.kD
        , elevatorConstants.kelevatorConstraints);
        m_Feedforward = new ElevatorFeedforward(elevatorConstants.kS, elevatorConstants.kG, elevatorConstants.kV);
    }


    @Override
    public void periodic(){
        double outputPID = m_PIDController.calculate(m_encoder.getPosition());
        double outputFeedForward = m_Feedforward.calculate(m_encoder.getVelocity());

        double finalOutput =outputPID+outputFeedForward;

        // m_rightMotor.setVoltage(finalOutput);
        // m_leftMotor.setVoltage(finalOutput);
        SmartDashboard.putNumber("elevator/Speed", m_motorSpeed);
        SmartDashboard.putNumber("elevator/potition", m_encoder.getPosition());
        SmartDashboard.putNumber("elevator/PID+FeedForward", finalOutput);
        setMotorSpeed(m_motorSpeed);    
    }
    public void test(){
        this.setMotorSpeed(m_motorSpeed);
    }

    private void setMotorSpeed(double speed){
        m_rightMotor.setVoltage(-speed);
        m_leftMotor.setVoltage(-speed);
        }
 
    public void setGoal(int stage){
        if (stage == 0) {
            m_PIDController.setGoal(0);
        }
        else if(stage == 1){
            m_PIDController.setGoal(0.1);
        }
        else if (stage == 2) {
            m_PIDController.setGoal(0.2);
        }
        else if (stage == 3) {
            m_PIDController.setGoal(0.15);
        }
        else{
            throw new RuntimeException("invalid stage");
        }

    }
 
 
    public Command runCmd(){
        //return this.runOnce(() -> setGoal(stage));
        return this.run(this::test);
    }

    public Command accelerateCmd(){
        return this.runOnce(() -> m_motorSpeed += 1).andThen(this::runCmd);
    }

    public Command decellerateCmd(){
        return this.runOnce(() -> m_motorSpeed -= 1).andThen(this::runCmd);
    }
    public Command stopCmd(){
        return this.runOnce(() -> m_motorSpeed = 0);
    }
}

