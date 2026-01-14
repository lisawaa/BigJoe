package frc.robot.Constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class Vision {
    public static final class Constants {
        //flip signs?
        public static final Transform3d[] CAMERA_TO_ROBOT = //Adjust
        {
         new Transform3d(new Translation3d(Units.inchesToMeters(16), Units.inchesToMeters(-8), Units.inchesToMeters(6)), 
            new Rotation3d(0, Units.degreesToRadians(60), 0)),
         new Transform3d(new Translation3d(Units.inchesToMeters(16), Units.inchesToMeters(8), Units.inchesToMeters(6)), 
            new Rotation3d(0, Units.degreesToRadians(60), 0))
        };
        public static final AprilTagFieldLayout m_targetPoses = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        //Adjust
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.1, 0.1, Units.radiansToDegrees(5));
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.1, 0.1, Units.radiansToDegrees(5));
    }
    public static class VisionIOInputs {
        public Pose2d[] estimate = new Pose2d[0];
        public double timestamp = 0;
        public double[] timestampArray = new double[0];

        public int[] camera1Targets = new int[0];
        public int[] camera2Targets = new int[0];
        public int[] camera3Targets = new int[0];

        public boolean hasEstimate = false;

        public byte[] results;
    }
}
