package frc.robot.subsystems.indexer.kicker;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class KickerIONeo implements KickerIO {
  SparkMax kickerMotor;
  SparkMaxConfig kickerConfig;
  RelativeEncoder kickerEncoder;

  public KickerIONeo() {
    kickerMotor = new SparkMax(999999, MotorType.kBrushless);
    kickerMotor.configure(
        new SparkMaxConfig(),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    kickerEncoder = kickerMotor.getEncoder();
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {

    inputs.kickerVoltage = kickerMotor.getBusVoltage();
    inputs.kickerCurrent = kickerMotor.getOutputCurrent();
    inputs.kickerPosition = kickerEncoder.getPosition();
    inputs.kickerVelocity = kickerEncoder.getVelocity();
    inputs.kickerTempC = kickerMotor.getMotorTemperature();
  }

  @Override
  public void setVoltage(double voltage) {
    kickerMotor.set(voltage);
  }

}
