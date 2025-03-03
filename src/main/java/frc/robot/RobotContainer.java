package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.OutakeSubsystem;
import frc.robot.subsystems.DriveSubsystem.Mode;
import frc.robot.Constants.CANids;
import frc.robot.Constants.xboxConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
	private final CommandXboxController m_driverController = new CommandXboxController(xboxConstants.kDriverControllerPort);
	private final CommandXboxController m_operatorController = new CommandXboxController(xboxConstants.kOperatorControllerPort);

	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
	private final OutakeSubsystem m_outakeSubsystem = new OutakeSubsystem(CANids.KOutakeCanId);

    public RobotContainer() {
		m_driveSubsystem.setDefaultCommand(m_driveSubsystem.driveWithJoystickCmd(m_driverController));

		new Trigger(m_elevatorSubsystem::outsideLimits).onTrue(umm());
		configureButtonBindings();
	}

	private Command umm(){
		System.out.println("\n\nWTF Hit Limits\n");
		return m_elevatorSubsystem.setLevel(0);
	}

	private void configureButtonBindings() {
		configureOperatorBindings();
		configureDriverBindings();
	}
  
	private void configureDriverBindings(){
		m_driverController.b().onTrue(m_driveSubsystem.resetGyro());
		m_driverController.leftTrigger().onTrue(m_driveSubsystem.setMode(Mode.Precision)).onFalse(m_driveSubsystem.setMode(Mode.Normal));
		m_driverController.rightTrigger().onTrue(m_driveSubsystem.setMode(Mode.Turbo)).onFalse(m_driveSubsystem.setMode(Mode.Normal));
	}
  
    private void configureOperatorBindings(){
		m_operatorController.leftBumper().onTrue(m_elevatorSubsystem.setLevel(0));
		m_operatorController.rightBumper().onTrue(m_elevatorSubsystem.setLevel(1));
		m_operatorController.leftTrigger().onTrue(m_elevatorSubsystem.setLevel(2));
		m_operatorController.rightTrigger().onTrue(m_elevatorSubsystem.setLevel(3));
		m_operatorController.x().onTrue(m_elevatorSubsystem.stopCmd());
		m_operatorController.b().onTrue(m_outakeSubsystem.outakeCmd().andThen(m_elevatorSubsystem.setLevel(0)));
		m_operatorController.a().onTrue(m_outakeSubsystem.outakeSlowCmd()).onFalse(m_outakeSubsystem.zeroCmd());
		// m_operatorController.povUp().onTrue(m_elevatorSubsystem.accelerateCmd());
		// m_operatorController.povDown().onTrue(m_elevatorSubsystem.decellerateCmd());
	}


	public Command getAutonomousCommand() {
		return Commands.sequence(
			Commands.parallel(
				m_driveSubsystem.goToPose(),
				new WaitCommand(2).andThen(m_elevatorSubsystem.setLevel(1).withDeadline(new WaitCommand(4)))
			),
			m_outakeSubsystem.outakeCmd(),
			m_elevatorSubsystem.setLevel(0));
	}

}