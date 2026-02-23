package frc.robot.subsystems.drive.module;

import static edu.wpi.first.units.Units.Meter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;

public class ModuleIOFullKraken implements ModuleIO {
    
    public TalonFX turnMotor; 

    public CANcoder turnEncoder;

    public TalonFX driveMotor;
    
    public CANcoder driveEncoder;

    public int moduleNumber;

    public ModuleIOFullKraken(int driveID, int turnID, int driveEncoderID, int turnEncoderID, int moduleNumber)
    {

        var driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        driveMotor = new TalonFX(driveID);
        driveMotor.getConfigurator().apply(driveConfig);

        var turnConfig = new TalonFXConfiguration();
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turnConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        turnMotor = new TalonFX(driveID);
        turnMotor.getConfigurator().apply(turnConfig);

        turnEncoder = new CANcoder(turnEncoderID);
        driveEncoder = new CANcoder(driveEncoderID);

        this.moduleNumber = moduleNumber;
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
        return driveMotor.getVelocity().getValueAsDouble() * Constants.swerveWheelRadius;
    }



    @Override
    public Rotation2d getTurnDegrees()
    {
        return new Rotation2d(turnMotor.getPosition().getValueAsDouble());
    }

    @Override
    public Distance getDriveDistance()
    {
        return 
        Distance.ofBaseUnits(driveMotor.getPosition().getValueAsDouble(), Meter);
    }


}
