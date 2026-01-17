package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;
import frc.robot.Constants.Vision.VisionIOInputs;

public class VisionSubsystem extends SubsystemBase{
    
    private final PhotonCamera camera1;
    private final PhotonPoseEstimator camera1Estimator;

    private final PhotonCamera camera2;
    private final PhotonPoseEstimator camera2Estimator;
    
    private final AprilTagFieldLayout layout = Vision.Constants.TARGET_POSES;

    public VisionSubsystem() {
        //CHANGE
        PortForwarder.add(5810, "10.44.70.203", 5810);
        layout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);

        camera1 = new PhotonCamera("camera1");
        camera1Estimator = new PhotonPoseEstimator(layout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Vision.Constants.CAMERA_TO_ROBOT[0]);
        camera1Estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        camera2 = new PhotonCamera("camera2");
        camera2Estimator = new PhotonPoseEstimator(layout,
             PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Vision.Constants.CAMERA_TO_ROBOT[1]);
        camera2Estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public void updateInputs(VisionIOInputs inputs, Pose2d currentEstimate) {

        PhotonPipelineResult[] results = getAprilTagResults();
        PhotonPoseEstimator[] photonPoseEstimators = new PhotonPoseEstimator[]{camera1Estimator, camera2Estimator};

        inputs.estimate = new Pose2d[]{ new Pose2d() };

        inputs.timestamp = estimateLatestTimestamp(results);

        if(hasEstimate(results)) {
            inputs.estimate = getEstimatesArray(results, photonPoseEstimators);
            inputs.hasEstimate = true;

            int[][] cameraTargets = getCameraTargets(results);
            inputs.camera1Targets = cameraTargets[0];
            inputs.camera2Targets = cameraTargets[1];
        } else {
            inputs.timestamp = inputs.timestamp;
            inputs.hasEstimate = false;
        }
    }

    private PhotonPipelineResult[] getAprilTagResults() {
        List<PhotonPipelineResult> list1 = camera1.getAllUnreadResults();
        List<PhotonPipelineResult> list2 = camera2.getAllUnreadResults();
        PhotonPipelineResult result1 = (list1.size() > 0) ? list1.get(list1.size() - 1) : new PhotonPipelineResult();
        PhotonPipelineResult result2 = (list2.size() > 0) ? list2.get(list2.size() - 1) : new PhotonPipelineResult();
        return new PhotonPipelineResult[] {result1, result2};
    }

    public boolean hasEstimate(PhotonPipelineResult[] results) {
        for(PhotonPipelineResult result : results) {
            if(result.hasTargets()) return true;
        }
        return false;
    }

    private double estimateLatestTimestamp(PhotonPipelineResult[] results) {
        double latestTimestamp = 0;
        int count = 0;
        for(PhotonPipelineResult result : results) {
            latestTimestamp = result.getTimestampSeconds();
            count++;
        }
        return latestTimestamp / count;
    }

    public  Pose2d[] getEstimatesArray(PhotonPipelineResult[] results, PhotonPoseEstimator[] photonEstimator) {
        Optional<Pose2d>[] estimates = getEstimates(results, photonEstimator);
        Pose2d[] estimatesArray = new Pose2d[estimates.length];
        for (int i = 0; i < estimates.length; i++) {
            if (estimates[i].isPresent() && estimates[i].get() != null) {
                estimatesArray[i] = estimates[i].get();
            }
        }

        int count = 0;
        for (int i = 0; i < estimatesArray.length; i++) {
            if (estimatesArray[i] != null) {
            count++;
            }
        }

        Pose2d[] finalEstimates = new Pose2d[count];
        int index = 0;
        for (int i = 0; i < estimatesArray.length; i++) {
        if (estimatesArray[i] != null) {
            finalEstimates[index] = estimatesArray[i];
            index++;
        }
        }

        return finalEstimates;
    } 

    public Optional<Pose2d>[] getEstimates(PhotonPipelineResult[] results, PhotonPoseEstimator[] photonEstimator) {
        ArrayList<Optional<Pose2d>> estimates = new ArrayList<>();
        for (int i = 0; i < results.length; i++) {
            PhotonPipelineResult result = results[i];
            if (result.hasTargets()) {
                var est = photonEstimator[i].update(result);
                if (est.isPresent() && result.hasTargets()) {
                    estimates.add(Optional.of(est.get().estimatedPose.toPose2d()));
                } else {
                    estimates.add(Optional.empty());
                }
            } else {
                estimates.add(Optional.empty());
            }
        }
        Optional<Pose2d>[] estimatesArray = estimates.toArray(new Optional[0]);
        return estimatesArray;
    } 

    public int[][] getCameraTargets(PhotonPipelineResult[] results) {
        int[][] targets = new int[results.length][];
    
        for (int i = 0; i < results.length; i++) {
          targets[i] = new int[results[i].targets.size()];
          for (int j = 0; j < results[i].targets.size(); j++) {
            targets[i][j] = results[i].targets.get(j).getFiducialId();
          }
        }
    
        return targets;
    }
   
   //Unused? Multi/SIngle standard deviations constans rn, see if it's better for variability
   public  List<Matrix<N3, N1>> getStdArray(VisionIOInputs inputs, Pose2d currentPose) {
        List<Matrix<N3, N1>> stdsArray = new ArrayList<Matrix<N3, N1>>();

        for (int i = 0; i < getCameraTargets(inputs).length; i++) {
            if (getCameraTargets(inputs)[i].length != 0) {
                stdsArray.add(getEstimationStdDevs(inputs, currentPose, i));
            }
        }

        return stdsArray;
    }

    public  int[][] getCameraTargets(VisionIOInputs inputs) {
        return new int[][] { inputs.camera1Targets, inputs.camera2Targets, inputs.camera3Targets };
    }

    public  Matrix<N3, N1> getEstimationStdDevs(VisionIOInputs inputs, Pose2d pose, int camera) {
        var estStdDevs = Vision.Constants.SINGLE_STD_DEVS;
        int numTags = 0;
        double avgDist = 0;
        int[] targets = getCameraTargets(inputs)[camera];
        for (var tgt : targets) {
            Optional<Pose3d> tagPose = layout.getTagPose(tgt);
            if (tagPose.isEmpty())
                continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(pose.getTranslation());
        }     
        if (numTags == 0)
            return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
            estStdDevs = Vision.Constants.MULTI_STD_DEVS;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
        estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    public Pose3d[] getTargetsPositions(PhotonPipelineResult[] results) {
        int total_targets = 0;
        for (int i = 0; i < results.length; i++) {
            if (results[i].hasTargets()) {
                total_targets += results[i].getTargets().size();
            }
        }
        Pose3d[] targets = new Pose3d[total_targets];
        int index = 0;
        for (int i = 0; i < results.length; i++) {
            if (results[i].hasTargets()) {
                for (PhotonTrackedTarget target : results[i].getTargets()) {
                    targets[index] = layout.getTagPose(target.getFiducialId()).get();
                    index++;
                }
            }
        }
        return targets;
    }
}
