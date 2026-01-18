package frc.robot.subsystems.Turret;

import edu.wpi.first.math.geometry.Rotation2d;

public interface TurretIO {

    public class TurretIOInputs
    {
        public Rotation2d currentRotation = new Rotation2d(90d);
    }

    public default Rotation2d getRotation()
    {
        return null;
    }

    public void setPower(double power);

    public default void updateInputs(TurretIOInputs inputs)
    {
    }
    
}
