package frc.robot.subsystems.shooter.Hood;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.shooter.Turret.TurretIO.TurretIOInputs;

public class HoodIOReal implements HoodIO {
    
    public SparkMax flywheel;

    public Servo servo1;
    public Servo servo2;

    public HoodIOReal()
    {
        flywheel = new SparkMax(0, MotorType.kBrushless);
    }

    @Override
    public Rotation2d getRotation()
    {
        return new Rotation2d(servo1.get() * 2 * Math.PI);
    }

    @Override
    public void setPower(double power) {
        flywheel.set(power);
    }
    
    public void updateInputs(TurretIOInputs inputs)
    {
        
        

        inputs.currentRotation = getRotation();
    }
}
