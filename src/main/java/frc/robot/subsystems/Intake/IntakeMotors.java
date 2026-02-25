package frc.robot.subsystems.Intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeMotors implements IntakeIO{
    SparkMax intake1;
    SparkMax intake2;
    RelativeEncoder intakeEncoder;

    public IntakeMotors() {
        intake1 = new SparkMax(0, null);
        intake2 = new SparkMax(1, null);
        intakeEncoder = intake2.getEncoder();
    }

    public void intake1UpdateInputs(intake1IOInputs inputs) {
        inputs.intake1Voltage = intake1.getBusVoltage();
        inputs.intake1Current = intake1.getOutputCurrent();
        inputs.intake1TempC = intake1.getMotorTemperature();
        inputs.intake1Connected = false;
    }

    public void intake2UpdateInputs(intake2IOInputs inputs) {
        inputs.intake2Voltage = intake2.getBusVoltage();
        inputs.intake2Current = intake2.getOutputCurrent();
        inputs.intake2TempC = intake2.getMotorTemperature();
        inputs.intake2Position = intakeEncoder.getPosition();
        inputs.intake2Velocity = intakeEncoder.getVelocity();
        inputs.intake2Connected = false;

    }

    @Override
    public void intake1SetPower(double power) {
        intake1SetPower(power);
    }

    public void intake2SetPower(double power) {
        intake2SetPower(power);
    }

     public Rotation2d getRotation() {
        return new Rotation2d(intakeEncoder.getPosition());
    }

}
