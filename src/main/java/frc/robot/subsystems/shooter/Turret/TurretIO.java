package frc.robot.subsystems.shooter.Turret;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

public interface TurretIO {

    public static class TurretIOInputs
    {
        public Rotation2d currentRotation = new Rotation2d(0);
    }

    public default Rotation2d getRotation()
    {
        return null;
    }

    public void setPower(double power);

    public default void updateInputs(TurretIOInputs inputs, Timer timer)
    {
        
    }
    
}
