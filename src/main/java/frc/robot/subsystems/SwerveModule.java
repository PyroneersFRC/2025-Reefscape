package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;


public class SwerveModule {
    private final SparkMax m_turningMotor;
    private final SparkFlex m_driveMotor;

    private final AnalogEncoder m_turningEncoder;
    private final RelativeEncoder m_driveEncoder;

    private final PIDController turningPIDcontroller;

    private final SparkAbsoluteEncoder m_AbsoluteEncoder;
    private final double m_AbsoluteEncoderReversed;

    public SwerveModule(int driveMotorid, int m_turningMotorid,)



    
}
