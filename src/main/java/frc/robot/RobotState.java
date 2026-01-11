package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class RobotState
{

    private static RobotState instance;

    public Pose2d estimatedPose = new Pose2d();

    public Rotation2d getRotation()
    {
        return estimatedPose.getRotation();
    }

    public static RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
    }

    public Translation2d getPosition()
    {
        return estimatedPose.getTranslation();
    }

    public Pose2d getPose()
    {
        return estimatedPose;
    }

    public void setPose(Pose2d odomPose) {
        estimatedPose = odomPose;
    }
}
