package frc.robot.subsystems.shooter.Hood;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import frc.robot.Constants;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood {
    
    public HoodIO hoodIO;
    public PIDController hoodPID;

    public Hood
    (
        HoodIO hoodIO, 
        double kP, 
        double kI, 
        double kD
    )
    {
        this.hoodIO = hoodIO;
        hoodPID = new PIDController(kP,kI,kD);
    }

    public Rotation2d findAngle()
    {
        Translation2d difference = 
            Constants.FieldConstants.getHub().getTranslation()
            .minus(RobotState.getInstance().getPosition());

        difference.getNorm();
        return null;
    }

}
