package frc.robot.subsystems.shooter.Turret;

import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class TurretIOMotor implements TurretIO {

    public SparkMax motor;

    public AbsoluteEncoder encoder;

    public SparkMax motor2;

    public AbsoluteEncoder encoder2;   

    public double lastTime = 0;

    public TurretIOMotor()
    {

        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
    
        motor = new SparkMax(Constants.turretID, MotorType.kBrushless);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor2 = new SparkMax(Constants.turretID2, MotorType.kBrushless);

        encoder = motor.getAbsoluteEncoder();

        //motor.setInputVoltage(1);
            
    }

    @Override
    public Rotation2d getRotation()
    {

        return new Rotation2d(encoder.getPosition() / Constants.TURRET_GEAR_RATIO);

    }

    @Override
    public void setPower(double power) {
        motor.set(power);
        motor2.set(power);
    }
    
    public void updateInputs(TurretIOInputs inputs, Timer timer)
    {
        inputs.currentRotation = getRotation();
    }
}
