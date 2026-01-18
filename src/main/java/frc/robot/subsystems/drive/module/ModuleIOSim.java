package frc.robot.subsystems.drive.module;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ModuleIOSim implements ModuleIO {
    
    public SwerveModuleSimulation moduleSim;

    private final SimulatedMotorController.GenericMotorController driveMotor;

    private final SimulatedMotorController.GenericMotorController turnMotor;

    public PIDController turnPID;

    public Rotation2d turnSetpoint;

    public int moduleNumber;

    public ModuleIOSim(SwerveModuleSimulation driveModuleSim, int moduleNumber)
    {
        this.moduleSim = driveModuleSim;

        driveMotor = moduleSim.useGenericMotorControllerForDrive();
        turnMotor = moduleSim.useGenericControllerForSteer();

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
        driveMotor.requestVoltage(Volts.of(output));
    }

    @Override
    public void setTurnVoltage(double output) 
    {
        turnMotor.requestVoltage(Volts.of(output));
    }

    @Override
    public double getDriveVelocity()
    {
        return moduleSim.getDriveWheelFinalSpeed().in(RadiansPerSecond) * Constants.swerveWheelRadius;
    }



    @Override
    public Rotation2d getTurnDegrees()
    {
        return moduleSim.getSteerAbsoluteFacing();
    }

    @Override
    public Distance getDriveDistance()
    {

        return Distance.ofBaseUnits
        (
            moduleSim.getDriveWheelFinalPosition().in(Radian)
        
            * Constants.swerveWheelRadius,
        
            Meter
        );
    }

    @Override
    public SwerveModuleState getModuleState()
    {
        return moduleSim.getCurrentState();
    }


}
