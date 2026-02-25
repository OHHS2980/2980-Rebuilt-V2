package frc.robot.subsystems.indexer.agitator;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.agitator.AgitatorIO.AgitatorIOInputs;

import org.littletonrobotics.junction.Logger;

public class Agitator extends SubsystemBase {
  private final AgitatorIO io;
  private final AgitatorIOInputs inputs = new AgitatorIOInputs();

  public Agitator(AgitatorIO io) {
    this.io = io;
  }

  SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(1, 1, 1);

  public void agitatorFeedforward(double agitator1Velocity, double agitator2Velocity) {
    io.set1Voltage(feedForward.calculate(agitator1Velocity));
    io.set2Voltage(feedForward.calculate(agitator2Velocity));
  }

    @Override
    public void periodic()
    {
        io.updateInputs(inputs);
    }

    public Command stop()
    {
        return Commands.run(() -> { 
          io.set1Voltage(0); 
          io.set2Voltage(0); 
          }, 
          this);
    }

    public Command agitate()
    {
      return Commands.run(() -> agitatorFeedforward(1, 1), this);
    }

}
// agitator: im soo freakin mad yo
// like and subscribe if you get this joke.