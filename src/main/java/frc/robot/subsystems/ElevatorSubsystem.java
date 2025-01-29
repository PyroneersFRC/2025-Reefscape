package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANids;
import frc.robot.Constants.robot;

public class ElevatorSubsystem  extends SubsystemBase{
private final SparkMax m_rightElevatorMotor = new SparkMax(CANids.kRightElevatorCanId, MotorType.kBrushed);
private final SparkMax m_LeftElevatorMotor = new SparkMax(CANids.kLeftElevatorCanId, MotorType.kBrushed);
private final PIDController m_elevatorPidController = new PIDController(robot.kPElevator,robot.kIElevator, robot.kDElevator);

}
