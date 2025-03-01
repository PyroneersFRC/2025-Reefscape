package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.OutakeSubsystem;
import frc.robot.Constants.CANids;
import frc.robot.Constants.xboxConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(xboxConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(xboxConstants.kOperatorControllerPort);


  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final OutakeSubsystem m_outakeSubsystem = new OutakeSubsystem(CANids.KOutakeCanId);

    public RobotContainer() {
      configureButtonBindings();
            m_driveSubsystem.setDefaultCommand(m_driveSubsystem.driveWithJoystickCmd(m_driverController));
     }


  private void configureButtonBindings() {
      m_driverController.b().onTrue(m_driveSubsystem.resetGyro());
      m_driverController.a().onTrue(m_driveSubsystem.goToPose());



      m_operatorController.povUp().onTrue(m_elevatorSubsystem.accelerateCmd());
      m_operatorController.povDown().onTrue(m_elevatorSubsystem.decellerateCmd());
      m_operatorController.leftBumper().onTrue(m_elevatorSubsystem.setLevel(0));
      // m_operatorController.rightBumper().onTrue(m_elevatorSubsystem.setLevel(1));
      // m_operatorController.leftTrigger().onTrue(m_elevatorSubsystem.setLevel(2));
      // m_operatorController.rightTrigger().onTrue(m_elevatorSubsystem.setLevel(3));
      m_operatorController.x().onTrue(m_elevatorSubsystem.stopCmd());
      m_operatorController.b().onTrue(m_outakeSubsystem.outakeCmd().andThen(m_elevatorSubsystem.setLevel(0)));
      m_operatorController.a().onTrue(m_outakeSubsystem.outakeSlowCmd()).onFalse(m_outakeSubsystem.zeroCmd());
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("auto1").andThen(m_outakeSubsystem.outakeCmd());
    
  }
  public Command getAutoElevator(){
    return m_elevatorSubsystem.setLevel(1);
  }

}