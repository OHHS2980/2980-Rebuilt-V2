package frc.robot.subsystems.drive;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOReal implements GyroIO {
    
    AHRS driveGyro;

    public GyroIOReal()
    {
        driveGyro = new AHRS(NavXComType.kUSB1);
    }

    public Rotation2d getHeading()
    {
        return driveGyro.getRotation3d().toRotation2d();
    }
}
