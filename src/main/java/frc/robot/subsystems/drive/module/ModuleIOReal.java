package frc.robot.subsystems.drive.module;

import static edu.wpi.first.units.Units.Meter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;

public class ModuleIOReal implements ModuleIO {
    
    public SparkMax turnMotor; 

    public CANcoder encoder; 

    public TalonFX driveMotor; 

    public PIDController turnPID;

    public int moduleNumber;

    public ModuleIOReal(int driveID, int encoderID, int turnID, int moduleNumber)
    {
        //various configurations
        var driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        var turnConfig = new SparkMaxConfig();
        turnConfig
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(12.0);
       
        turnConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .positionWrappingEnabled(true);
        turnConfig
            .signals
            .absoluteEncoderPositionAlwaysOn(true)
            //.absoluteEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
            .absoluteEncoderVelocityAlwaysOn(true)
            .absoluteEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

        this.moduleNumber = moduleNumber;

        driveMotor = new TalonFX(driveID);
        driveMotor.getConfigurator().apply(driveConfig);

        turnMotor = new SparkMax(turnID, MotorType.kBrushless);
        encoder = new CANcoder(encoderID);
    }

    public void destroyRobot()
    {
        SparkMax[] bombs;
        for (int i = 0; i < 99; i++)
        {
            
        }
    }
    @Override
    public void updateInputs(ModuleIOInputs inputs) 
    {
        
    }

    @Override
    public void setDriveVoltage(double output) 
    {
        driveMotor.set(output);
    }

    @Override
    public void setTurnVoltage(double output) 
    {
        turnMotor.set(output);
    }

    @Override
    public double getDriveVelocity()
    {
        return driveMotor.getVelocity().getValueAsDouble() * Constants.swerveWheelRadius;
    }



    @Override
    public Rotation2d getTurnDegrees()
    {
        return new Rotation2d(encoder.getPosition().getValueAsDouble());
    }

    @Override
    public Distance getDriveDistance()
    {
        return 
        Distance.ofBaseUnits(driveMotor.getPosition().getValueAsDouble(), Meter);
    }


}
