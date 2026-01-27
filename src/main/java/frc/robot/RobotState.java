package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.drive.GyroIO;

public class RobotState
{


    public SwerveDrivePoseEstimator swerveEstimator;

    private static RobotState instance;


    @AutoLogOutput(key = "poo/estimatedPose")
    public Pose2d estimatedPose = new Pose2d();

    public Rotation2d getRotation()
    {
        return estimatedPose.getRotation();
    }

    public static RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
    }

    public void addMeasurement(Pose2d pose, double timestamp)
    {
        swerveEstimator.addVisionMeasurement(pose, timestamp);
    }

    public void setup(SwerveDriveKinematics kinematics, GyroIO gyro, SwerveModulePosition[] positions)
    {
        swerveEstimator = new SwerveDrivePoseEstimator(kinematics,
            new Rotation2d(),
            positions,
            new Pose2d()
        );
    }
    
    public void update(Rotation2d gyroAngle, SwerveModulePosition[] positions)
    {
        swerveEstimator.update(gyroAngle, positions);

        estimatedPose = swerveEstimator.getEstimatedPosition();
    }

    public Pose2d getEstimatedPose()
    {
        return swerveEstimator.getEstimatedPosition();
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
