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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;

public class ModuleIOReal implements ModuleIO {
    
    public final SparkMax turnMotor; 

    public final CANcoder encoder; 

    public final TalonFX driveMotor; 

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
            .idleMode(IdleMode.kBrake);
            //.voltageCompensation(12.0);
       
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

        //rnMotor.set(0.05);
        encoder = new CANcoder(encoderID);
    }


    @Override
    public void updateInputs(ModuleIOInputs inputs) 
    {
        inputs.turnPosition = getTurnDegrees();
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
        return driveMotor.getRotorVelocity().getValueAsDouble() * Constants.swerveWheelRadius;

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


    @Override
    public SwerveModuleState getModuleState()
    {
        return new SwerveModuleState(getDriveVelocity(), getTurnDegrees());
    }

}
