package frc.robot.subsystems;


import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANids;
import frc.robot.Constants.elevatorConstants;


public class ElevatorSubsystem  extends SubsystemBase{
    public enum ElevatorState {
        LEVEL_0(elevatorConstants.levelValues[0]), LEVEL_1(elevatorConstants.levelValues[1]), LEVEL_2(elevatorConstants.levelValues[2]), LEVEL_3(elevatorConstants.levelValues[3]);

        private final double encoderValue;

        private ElevatorState(double encoderValue) {
            this.encoderValue = encoderValue;
        }

        public double getLevelValue(){
            return this.encoderValue;
        }
    }

    private ElevatorState elevatorState = ElevatorState.LEVEL_0; // this is forcasted position, not actual

    private final ProfiledPIDController m_PIDController = new ProfiledPIDController(
            elevatorConstants.kP
        ,elevatorConstants.kI
        ,elevatorConstants.kD
        , elevatorConstants.kelevatorConstraints);

    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(0.7, 1.4, 0.3);   // kg htan 1.4 kai htan kalo!!

    private final ElevatorModule m_elevator;

    private final String SMART_DASHBOARD_PREFIX = "Elevator Subsystem/";

    private double m_motorSpeed = 0;

    public ElevatorSubsystem(){
        m_elevator = new ElevatorModule(CANids.kLeftElevatorCanId, CANids.kRightElevatorCanId);
    }
    

    private void potitionSafety(){
        double pos = m_elevator.getPotition();
        if(pos > 6.6  || pos < -0.1){
            System.out.println("\n\n\n\n\n\n pos " + pos + "\n\n\n\n\n\n\n");
            this.setDesiredState(0);
        }
    }

    @Override
    public void periodic(){;
        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "encoder/potition", m_elevator.getPotition()); 
        // SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "encoder/velocity", m_elevator.getVelocity()); 
        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "motor speed", m_motorSpeed); 

        potitionSafety(); 

        //m_elevator.setMotorSpeed(m_motorSpeed);
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

        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "output voltage/total output", outputVoltage);
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

    public Command setLevel(ElevatorState state){
        if(state != ElevatorState.LEVEL_0 && state != ElevatorState.LEVEL_1 && state != ElevatorState.LEVEL_2 && state != ElevatorState.LEVEL_3){
            throw new RuntimeException("Invalid level (" + state + ")");
        }

        elevatorState = state;
        return this.run(() -> setDesiredState(elevatorState.getLevelValue()));
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

    public ElevatorState getElevatorState(){
        return elevatorState;
    }
}