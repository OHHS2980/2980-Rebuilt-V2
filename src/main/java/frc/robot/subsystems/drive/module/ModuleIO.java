package frc.robot.subsystems.drive.module;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;

public interface ModuleIO {

    @AutoLog
    public static class ModuleIOInputs {

        public int moduleNumber = 0;

        public boolean driveConnected = false;
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public boolean turnConnected = false;
        public Rotation2d turnPosition = Rotation2d.fromDegrees(0);
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;

        public double[] tempCelcius = new double[] {};
        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    public default void updateInputs(ModuleIOInputs inputs) {}

    public default void update() { }

    public default void setDriveVoltage(double output) {}

    public default void setTurnVoltage(double output) {}

    public default void setTurnPosition(Rotation2d rotation) {}

    public default Rotation2d getTurnDegrees() 
    {
        return null;
    }

    public default Distance getDriveDistance() 
    {
        return null;
    }

    public default double getDriveVelocity()
    {
        return 0;
    }

    public default SwerveModuleState getModuleState()
    {
        return null;
    }

}
