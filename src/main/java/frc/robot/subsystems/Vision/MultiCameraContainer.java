package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.EntechGeometryUtils;

public class MultiCameraContainer implements CameraContainer {
  private final CameraContainer[] cameraContainers;

  //made by 281, magic to me.
  public MultiCameraContainer(CameraContainer... cameraContainers) {
    this.cameraContainers = cameraContainers;
  }

  @Override
  public Optional<Pose2d> getEstimatedPose() {
    List<Pose2d> estimatedPoses = new ArrayList<>();

    for (CameraContainer cameraContainer : cameraContainers) {
      Optional<Pose2d> estPose = cameraContainer.getEstimatedPose();
      if (estPose.isPresent()) {
        estimatedPoses.add(estPose.get());
      }
    }

    if (estimatedPoses.isEmpty())
      return Optional.empty();

    if (estimatedPoses.size() == 1)
      return Optional.of(estimatedPoses.get(0));

    Pose2d averagePose = estimatedPoses.get(0);

    for (int i = 1; i < estimatedPoses.size(); i++) {
      averagePose = EntechGeometryUtils.averagePose2d(averagePose, estimatedPoses.get(i));
    }

    return Optional.of(averagePose);
  }

  @Override
  public PhotonPipelineResult getFilteredResult() {
    List<PhotonTrackedTarget> targets = new ArrayList<>();
    double timeStamp = 0.0;
    double latency = 0.0;

    for (CameraContainer cameraContainer : cameraContainers) {
      PhotonPipelineResult result = cameraContainer.getFilteredResult();
      targets.addAll(cameraContainer.getFilteredResult().getTargets());
      timeStamp += result.getTimestampSeconds();
      latency += result.getLatencyMillis();
    }

    latency /= cameraContainers.length;
    timeStamp /= cameraContainers.length;

    PhotonPipelineResult filteredResult = new PhotonPipelineResult(latency, targets);
    filteredResult.setTimestampSeconds(timeStamp);

    return filteredResult;
  }

  @Override
  public double getLatency() {
    double latencySum = 0.0;
    for (CameraContainer cameraContainer : cameraContainers) {
      latencySum += cameraContainer.getLatency();
    }
    return latencySum / cameraContainers.length;
  }

  @Override
  public int getTargetCount() {
    int targetCount = 0;
    for (CameraContainer cameraContainer : cameraContainers) {
      targetCount += cameraContainer.getTargetCount();
    }
    return targetCount;
  }

  @Override
  public boolean hasTargets() {
    boolean hasTargets = false;
    for (CameraContainer cameraContainer : cameraContainers) {
      hasTargets = hasTargets || cameraContainer.hasTargets();
    }
    return hasTargets;
  }

  @Override
  public List<EntechTargetData> getTargetData() {
    List<EntechTargetData> data = new ArrayList<>();
    for (CameraContainer cameraContainer : cameraContainers) {
      data.addAll(cameraContainer.getTargetData());
    }
    return data;
  }
}