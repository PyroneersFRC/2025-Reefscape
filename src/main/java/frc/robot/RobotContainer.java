package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.OutakeSubsystem;
import frc.robot.subsystems.DriveSubsystem.Mode;
import frc.robot.subsystems.ElevatorSubsystem.Level;
import frc.robot.Constants.CANids;
import frc.robot.Constants.xboxConstants;

import javax.sound.sampled.SourceDataLine;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
		new Trigger(m_elevatorSubsystem::enforcedPrecision).onTrue(	
			m_driveSubsystem.setMode(Mode.SuperPrecision)
			.alongWith(Commands.sequence(
				new InstantCommand(() -> m_driverController.setRumble(RumbleType.kBothRumble, 1)),
				new WaitCommand(0.5),
				new InstantCommand(() -> m_driverController.setRumble(RumbleType.kBothRumble, 0)))))
			.onFalse(m_driveSubsystem.setMode(Mode.Normal));
		configureButtonBindings();
	}

	private Command umm(){
		System.out.println("\n\nWTF Hit Limits\n");
		return m_elevatorSubsystem.setLevel(Level.Level2);
	}

	private void configureButtonBindings() {
		configureOperatorBindings();
		configureDriverBindings();
	}
  
	private void configureDriverBindings(){
		m_driverController.x().onTrue(m_driveSubsystem.resetGyro());
		m_driverController.leftTrigger().or(m_driverController.button(xboxConstants.buttons.upLeft))
				.onTrue(m_driveSubsystem.setMode(Mode.Precision))
				.onFalse(m_driveSubsystem.setMode(Mode.Normal));
		m_driverController.rightTrigger().or(m_driverController.button(xboxConstants.buttons.upRight)).and(() -> !m_elevatorSubsystem.outsideLimits())
				.onTrue(m_driveSubsystem.setMode(Mode.Turbo))
				.onFalse(m_driveSubsystem.setMode(Mode.Normal));

		// new Trigger(() -> {return true;}).whileTrue(new InstantCommand(() -> m_operatorController.setRumble(RumbleType.kBothRumble, 1)));

		// m_driverController.button(xboxConstants.buttons.r3)
		// 		.onTrue( new InstantCommand(() -> m_operatorController.setRumble(RumbleType.kBothRumble, 1)))
		// 		.onFalse( new InstantCommand(() -> m_operatorController.setRumble(RumbleType.kBothRumble, 0)));
	}
  
    private void configureOperatorBindings(){
		m_operatorController.leftBumper().onTrue(m_elevatorSubsystem.setLevel(Level.Level0));
		m_operatorController.rightBumper().onTrue(m_elevatorSubsystem.setLevel(Level.Level1));
		m_operatorController.leftTrigger().onTrue(m_elevatorSubsystem.setLevel(Level.Level2));
		m_operatorController.rightTrigger().onTrue(m_elevatorSubsystem.setLevel(Level.Level3));
		m_operatorController.x().onTrue(m_elevatorSubsystem.stopCmd());
		m_operatorController.b().onTrue(m_outakeSubsystem.outakeCmd().andThen(new WaitCommand(0.7)).andThen(m_outakeSubsystem.zeroCmd().andThen(m_elevatorSubsystem.setLevel(Level.Level0))));
		m_operatorController.a().onTrue(m_outakeSubsystem.outakeSlowCmd()).onFalse(m_outakeSubsystem.zeroCmd());
		m_operatorController.y().onTrue(Commands.sequence(
			m_elevatorSubsystem.setLevel(m_elevatorSubsystem.getLastLevel()),
			m_outakeSubsystem.emergencyCmd()
		)).onFalse(m_outakeSubsystem.zeroCmd());
		m_operatorController.x().onTrue(m_outakeSubsystem.reverseCmd()).onFalse(m_outakeSubsystem.zeroCmd());
		// m_operatorController.povUp().onTrue(m_elevatorSubsystem.accelerateCmd());
		// m_operatorController.povDown().onTrue(m_elevatorSubsystem.decellerateCmd());

		// m_operatorController.setRumble(RumbleType.kBothRumble, 1);
	}

	public Command getAutonomousCommand() {
		return Commands.sequence(
			// align wheels
			Commands.run(() -> m_driveSubsystem.drive(0.000001, 0, 0, false), m_driveSubsystem).withDeadline(new WaitCommand(0.2)),
			Commands.runOnce(() -> m_driveSubsystem.zeroHeading()),
			Commands.parallel(
				m_driveSubsystem.goToPose(),
				new WaitCommand(1.3).andThen(m_elevatorSubsystem.setLevel(Level.Level1).withDeadline(new WaitCommand(2)))
			),
			m_outakeSubsystem.outakeCmd().andThen(new WaitCommand(1)).andThen(m_outakeSubsystem.zeroCmd()),
			m_elevatorSubsystem.setLevel(Level.Level0));
	}

}