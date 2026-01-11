package frc.robot.subsystems.drive.module;

import static edu.wpi.first.units.Units.Meter;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.ModuleState;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.subsystems.drive.module.ModuleIO.ModuleIOInputs;

public class Module {

    public PIDController turnPID;

    public PIDController drivePD;

    public SimpleMotorFeedforward driveFF;

    public ModuleIO moduleIO;

    public int moduleNumber;

    public SwerveModulePosition modulePosition;

    public SwerveModuleState desiredModuleState;

    public ModuleIOInputs inputs;

    public enum position  
    {
        frontLeft,
        frontRight,
        backLeft,
        backRight
    }

    public Module(
        
        ModuleIO moduleIO,

        double kP, double kI, double kD,

        double drive_kP, double drive_kD

    )

    {
        this.moduleIO = moduleIO; 

        turnPID = new PIDController(kP, kI, kD);

        drivePD = new PIDController(kP, 0, kD);

        driveFF = new SimpleMotorFeedforward(5, 5);

        inputs = new ModuleIOInputs();
    }

    public void update()
    {
        moduleIO.setTurnVoltage(
            turnPID.calculate(inputs.turnPosition.getDegrees())
        );

        moduleIO.setDriveVoltage(
            drivePD.calculate(moduleIO.getDriveVelocity()) //+ driveFF.calculate(desiredModuleState.speedMetersPerSecond)
        );

        moduleIO.updateInputs(inputs);

        configurePID();
    }

    public void runToState(SwerveModuleState state)
    {
        desiredModuleState = state;

        //moduleIO.setDriveVelocity(state.speedMetersPerSecond);
        System.out.println(state.speedMetersPerSecond);
        System.out.println(state.angle);
        turnPID.setSetpoint(state.angle.getDegrees());
        drivePD.setSetpoint(state.speedMetersPerSecond);
    }

    public SwerveModulePosition getPosition()
    {
        System.out.println("pos" + moduleIO.getDriveDistance().baseUnitMagnitude());
        System.out.println("pos" + moduleIO.getTurnDegrees().getRadians());
        return new SwerveModulePosition(

            moduleIO.getDriveDistance(),

            moduleIO.getTurnDegrees()

        );
    }

    public SwerveModuleState getModuleState()
    {
        return moduleIO.getModuleState();
    }

    public void configurePID
    (

    ) 
        
    {
        turnPID.setPID
        (
            Constants.SimConstants.turnP.get(), 
            Constants.SimConstants.turnI.get(), 
            Constants.SimConstants.turnD.get()
        );

        drivePD.setPID
        (
            Constants.SimConstants.driveP.get(), 
            0,
            Constants.SimConstants.driveD.get()
        );
    }

}
