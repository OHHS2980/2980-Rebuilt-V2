package frc.robot.subsystems.Turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret.TurretIO.TurretIOInputs;

public class Turret extends SubsystemBase {
    
    public PIDController turretPID;

    public TurretIO turretIO;

    public Rotation2d desiredRotation = new Rotation2d();

    public TurretIOInputs inputs = new TurretIOInputs();

    public Turret
    (
        TurretIO turretIO,
        double kP,
        double kI,
        double kD
    )

    {
        this.turretIO = turretIO;

        turretPID = new PIDController(kP, kI, kD);
        
    }

    public void moveToRotation(Rotation2d rotation)
    {
        desiredRotation = rotation;
        turretPID.setSetpoint(rotation.getDegrees() % Constants.turretLimit);
    }

    public Rotation2d odometryAutoalign()
    {
        double y = RobotState.getInstance().estimatedPose.getY()  - Constants.FieldConstants.getHub().getY();
        double x = RobotState.getInstance().estimatedPose.getX()  - Constants.FieldConstants.getHub().getX();
        double hypotenuse = Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2));

        double angle = Math.asin(
          x / hypotenuse
        );

        return new Rotation2d(angle);
    }

    public Rotation2d visionAutoalign()
    {
        return null;
    }

    public void update()
    {
        turretIO.setPower
        (
            turretPID.calculate(turretIO.getRotation().getDegrees())
        );

        turretPID.setPID
        (
            Constants.SimConstants.turretP.get(), 
            Constants.SimConstants.turretP.get(), 
            Constants.SimConstants.turretP.get()
        );
    }

    @Override
    public void periodic()
    {
        turretIO.updateInputs(inputs);

        update();

        moveToRotation(odometryAutoalign());
    }
}
