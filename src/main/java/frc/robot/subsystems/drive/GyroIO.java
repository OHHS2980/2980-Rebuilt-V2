package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

public interface GyroIO {
    
    public default Rotation2d getHeading() {
        return null;
    }
}
