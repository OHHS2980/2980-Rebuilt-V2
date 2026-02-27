package frc.robot.subsystems.shooter.Turret;

//import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class TurretIOMotor implements TurretIO {

    public SparkMax motor;

    public RelativeEncoder encoder;

    public double lastTime = 0;

    public TurretIOMotor()
    {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
    
        motor = new SparkMax(Constants.turretID, MotorType.kBrushless);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = motor.getEncoder();
    }

    @Override
    public Rotation2d getRotation()
    {
        return new Rotation2d(encoder.getPosition() * Math.PI * 2);
    }

    @Override
    public void setPower(double power) {
        motor.set(power);
    }
    
    public void updateInputs(TurretIOInputs inputs, Timer timer)
    {
        inputs.currentRotation = getRotation();
    }

}
