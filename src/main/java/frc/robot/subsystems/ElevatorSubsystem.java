package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANids;
import frc.robot.Constants.elevatorConstants;


public class ElevatorSubsystem  extends SubsystemBase{
    private final ProfiledPIDController m_PIDController = new ProfiledPIDController(
        elevatorConstants.PID.kP,
        elevatorConstants.PID.kI,
        elevatorConstants.PID.kD,
        elevatorConstants.PID.kelevatorConstraints);

    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
        elevatorConstants.FeedForward.kS,
        elevatorConstants.FeedForward.kG,
        elevatorConstants.FeedForward.kV);

    private final ElevatorModule m_elevator;

    private Level m_lastLevel = Level.Level0;

    private final String SMART_DASHBOARD_PREFIX = "Elevator Subsystem/";

    private double m_motorSpeed = 0;

    public enum Level {
        Level0(elevatorConstants.setPoints.level0),
        Level1(elevatorConstants.setPoints.level1),
        Level2(elevatorConstants.setPoints.level2),
        Level3(elevatorConstants.setPoints.level3);

        private double setPoint;

        private Level(double setPoint) {
            this.setPoint = setPoint;
        }
    }

    public ElevatorSubsystem(){
        m_elevator = new ElevatorModule(CANids.kLeftElevatorCanId, CANids.kRightElevatorCanId);
        SmartDashboard.putString(SMART_DASHBOARD_PREFIX + "tuning/pid values", "P: " + elevatorConstants.PID.kP + " I: " + elevatorConstants.PID.kI + " D: " + elevatorConstants.PID.kD);
        SmartDashboard.putString(SMART_DASHBOARD_PREFIX + "tuning/feeddorward values", "Ks: " + elevatorConstants.FeedForward.kS + " kG: " + elevatorConstants.FeedForward.kG + " kV: " + elevatorConstants.FeedForward.kV);
    }
    
    public boolean outsideLimits(){
        double pos = m_elevator.getPotition();
        return pos > elevatorConstants.setPoints.maxSafety || pos < -0.1;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "encoder/potition", m_elevator.getPotition()); 
        // SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "encoder/zori", 3.5);
        // SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "encoder/velocity", m_elevator.getVelocity()); 
        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "motor speed", m_motorSpeed); 

        // m_elevator.setMotorSpeed(m_motorSpeed);
    }
    
    private void printSetpoint(TrapezoidProfile.State setpoint){
        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "profiled pid Setpoint/position", setpoint.position);
        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "profiled pid Setpoint/velocity", setpoint.velocity);
    }
 
    private void setDesiredState(double desiredState){
        double PIDOutput = m_PIDController.calculate(m_elevator.getPotition(), desiredState);
        double feedforwardOutput = m_feedforward.calculate(m_PIDController.getSetpoint().velocity);
        double outputVoltage = PIDOutput + feedforwardOutput;
        outputVoltage = MathUtil.clamp(outputVoltage, -8, 8);
        

        // patenta
        if(m_PIDController.getSetpoint().position == 0){
            outputVoltage = 0;
        }

        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "output voltage/total output", outputVoltage);
        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "output voltage/pid output", PIDOutput);
        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "output voltage/feedforward output", feedforwardOutput);
        printSetpoint(m_PIDController.getSetpoint());

        SmartDashboard.putNumber(SMART_DASHBOARD_PREFIX + "output voltage/actual total output", outputVoltage);
        m_elevator.setMotorSpeed(outputVoltage);
    }

    public Command setLevel(Level level){
        m_lastLevel = level;
        return this.run(() -> setDesiredState(level.setPoint));
    }
    
    public Level getLastLevel(){
        return m_lastLevel;
    }

    public void stop(){
        m_elevator.setMotorSpeed(0);
    }
    
    public Command stopCmd(){
        return this.runOnce(this::stop);
    }

    // public Command accelerateCmd(){
    //     return this.runOnce(() -> { m_motorSpeed += 0.1; 
    //                                 m_elevator.setMotorSpeed(m_motorSpeed);});
    // }

    // public Command decellerateCmd(){
    //     return this.runOnce(() -> { m_motorSpeed -= 0.1;
    //                                 m_elevator.setMotorSpeed(m_motorSpeed);});
    //     }

    public boolean enforcedPrecision(){
        return m_elevator.getPotition() > elevatorConstants.kForcedPrecision;
    }

}