package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;
import frc.robot.Constants.Vision.VisionIOInputs;

public class VisionSubsystem extends SubsystemBase {
    
    public class TagPlacements
    {
        List<AprilTag> tags;
    };
        
    private final ArrayList<PhotonCamera> cameras = new ArrayList<PhotonCamera>();
    private final ArrayList<PhotonPoseEstimator> cameraEstimators = new ArrayList<PhotonPoseEstimator>();

    //add more cameras if needed
    private StructPublisher<Pose3d> publisherCamera1Pose = NetworkTableInstance.getDefault().getStructTopic("Camera1Pose", Pose3d.struct).publish();
    private StructPublisher<Pose3d> publisherCamera2Pose = NetworkTableInstance.getDefault().getStructTopic("Camera2Pose", Pose3d.struct).publish();
    private StructArrayPublisher<Pose3d> publisherTagPoses = NetworkTableInstance.getDefault().getStructArrayTopic("TagPlacements", Pose3d.struct).publish();

    private VisionIOInputs visionInputs = new VisionIOInputs();

    public VisionSubsystem() {
        //change
        PortForwarder.add(5810, "10.44.70.11", 5800);

        // camera 1
        cameras.add(new PhotonCamera("camera1"));
        cameraEstimators.add(new PhotonPoseEstimator(Vision.Constants.TARGET_POSES, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Vision.Constants.CAMERA_TO_ROBOT[0]));

        // camera 2
        cameras.add(new PhotonCamera("camera2"));
        cameraEstimators.add(new PhotonPoseEstimator(Vision.Constants.TARGET_POSES,
             PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Vision.Constants.CAMERA_TO_ROBOT[1]));
    }

    @Override
    public void periodic()
    {
        updateInputs();

        if(null != visionInputs.cameraTargets[0]) {
            publisherCamera1Pose.set(visionInputs.cameraPoses[0]);
        }
        if(null != visionInputs.cameraTargets[1]) {
            publisherCamera2Pose.set(visionInputs.cameraPoses[1]);
        }
        List<AprilTag> Tags = Vision.Constants.TARGET_POSES.getTags();
        Pose3d[] tagPoses = new Pose3d[Tags.size()];
        for(int i = 0; i < Tags.size(); ++i) {
            tagPoses[i] = Tags.get(i).pose;
        }
        publisherTagPoses.set(tagPoses);
    }

    public void updateInputs() {
        int numberOfCameras = cameras.size();
        visionInputs.cameraPoses = new Pose3d[numberOfCameras];
        visionInputs.cameraTargets = new List[numberOfCameras];
        visionInputs.timestamps = new double[numberOfCameras];
        for(int i = 0; i < cameras.size(); ++i) {
            visionInputs.timestamps[i] = 0.0;
            PhotonCamera camera = cameras.get(i);
            PhotonPoseEstimator estimator = cameraEstimators.get(i);           
            List<PhotonPipelineResult> results = camera.getAllUnreadResults();
            PhotonPipelineResult result = !results.isEmpty() ? results.get(results.size() - 1) : new PhotonPipelineResult();
            visionInputs.timestamps[i] = result.getTimestampSeconds();
            Optional<EstimatedRobotPose> estimatedPose = estimator.update(result);
            if(!estimatedPose.isPresent()) {
                visionInputs.cameraPoses[i] = Pose3d.kZero;
                visionInputs.cameraTargets[i] = null;
            } else {
                visionInputs.cameraPoses[i] = estimatedPose.get().estimatedPose;
                visionInputs.cameraTargets[i] = estimatedPose.get().targetsUsed;
            }
            Logger.recordOutput("Vision/Camera" + (i+1) + "/HasTarget", visionInputs.cameraTargets[i] != null);
            Logger.recordOutput("Vision/Camera" + (i+1) + "/TagCount", visionInputs.cameraTargets[i] != null ? visionInputs.cameraTargets[i].size() : 0);
            Logger.recordOutput("Vision/Camera" + (i+1) + "/Pose", visionInputs.cameraPoses[i]);
            Logger.recordOutput("Vision/Camera" + (i+1) + "/Timestamps", visionInputs.timestamps[i]);
            Logger.recordOutput("Vision/Camera" + (i+1) + "/DistanceToHub", visionInputs.cameraTargets[i] != null && !visionInputs.cameraTargets[i].isEmpty() ? 
                visionInputs.cameraTargets[i].get(0).getBestCameraToTarget().getTranslation().getNorm() : 0.0);

            //Log each tracked April Tag ID
            if (visionInputs.cameraTargets[i] != null) {
                for (int t = 0; t < visionInputs.cameraTargets[i].size(); t++) {
                    int id = visionInputs.cameraTargets[i].get(t).getFiducialId();
                    Logger.recordOutput("Vision/Camera" + (i+1) + "/Tag" + t + "/ID", id);
                }
            } else {
                // Optionally clear indices if you expect a fixed number of slots
                Logger.recordOutput("Vision/Camera" + (i+1) + "/Tag0/ID", -1);
            }
        }
    }

    public VisionIOInputs getInputs() {
        return visionInputs;
    }
}