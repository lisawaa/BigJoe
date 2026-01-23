package frc.robot.Constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.List;
import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;


public class Vision {
    public static final class Constants {
        //CHANGE
        public static final Transform3d[] CAMERA_TO_ROBOT = 
        {
         new Transform3d(new Translation3d(Units.inchesToMeters(16), Units.inchesToMeters(-8), Units.inchesToMeters(6)), 
            new Rotation3d(0, Units.degreesToRadians(3), 0)),
         new Transform3d(new Translation3d(Units.inchesToMeters(16), Units.inchesToMeters(8), Units.inchesToMeters(6)), 
            new Rotation3d(0, Units.degreesToRadians(3), 0))
        };
        public static final AprilTagFieldLayout TARGET_POSES = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        //Adjust
        public static final Matrix<N3, N1> SINGLE_STD_DEVS = VecBuilder.fill(0.1, 0.1, Units.radiansToDegrees(5));
        public static final Matrix<N3, N1> MULTI_STD_DEVS = VecBuilder.fill(0.1, 0.1, Units.radiansToDegrees(5));

        public static final Pose3d redHubPose = new Pose3d(Units.inchesToMeters(468.56), Units.inchesToMeters(158.32), Units.inchesToMeters(72.0), new Rotation3d());
        public static final Pose3d blueHubPose = new Pose3d(Units.inchesToMeters(152.56), Units.inchesToMeters(158.32),  Units.inchesToMeters(72.0), new Rotation3d());
       
        public static final Pose3d getHubPose() {
            Pose3d pose = DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? redHubPose : blueHubPose;
            return pose;
        }
    }
    public static class VisionIOInputs {
        public Pose3d cameraPoses[];
        public List<PhotonTrackedTarget>[] cameraTargets;
        public double timestamp = 0.0;
    }
}
