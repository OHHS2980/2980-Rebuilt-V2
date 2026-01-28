package frc.robot.subsystems.drive.module;

import static edu.wpi.first.units.Units.Meter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;

public class ModuleIOFullKraken implements ModuleIO {
    
    public TalonFX turnMotor; 

    public TalonFX driveMotor; 

    public PIDController turnPID;

    public int moduleNumber;

    public ModuleIOFullKraken(double canID, int moduleNumber)
    {

        this.moduleNumber = moduleNumber;
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
        return new Rotation2d(turnMotor.getPosition().getValueAsDouble());
    }

    @Override
    public Distance getDriveDistance()
    {
        return 
        Distance.ofBaseUnits(driveMotor.getPosition().getValueAsDouble(), Meter);
    }


}
