package frc.robot.subsystems.Intake;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakeIO {
    //intake1: feeds intake
    //intake2: moves intake
    public static class intake1IOInputs {
        public boolean intake1Connected = false;
        public double intake1Voltage = 0.0;
        public double intake1Current = 0.0;
        public double intake1Position = 0.0;
        public double intake1Velocity = 0.0;
        public double intake1TempC = 0.0;
    }

    public static class intake2IOInputs {
        public boolean intake2Connected = false;
        public double intake2Voltage = 0.0;
        public double intake2Current = 0.0;
        public double intake2Position = 0.0;
        public double intake2Velocity = 0.0;
        public double intake2TempC = 0.0;
        public Rotation2d intake2Rotation = new Rotation2d(0);
    }

    public default Rotation2d getRotation() {
        return null;
    }

    public void intake1SetPower (double power);
    public void intake2SetPower (double power);
}
