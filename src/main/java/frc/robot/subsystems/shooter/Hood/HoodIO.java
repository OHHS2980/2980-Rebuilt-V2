package frc.robot.subsystems.shooter.Hood;

import edu.wpi.first.math.geometry.Rotation2d;

public interface HoodIO {
    
    public static class HoodIOInputs
    {
        public Rotation2d currentRotation = new Rotation2d(1);
    }

    public default Rotation2d getRotation()
    {
        return null;
    }

    public void setPower(double power);

    public default void updateInputs(HoodIOInputs inputs)
    {

    }

}
