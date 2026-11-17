package frc.robot.utility;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/** Converts a Limelight center-origin .fmap into the WPILib blue-origin field-layout JSON format. */
public final class LimelightFmapConverter {
  private LimelightFmapConverter() {}

  public static AprilTagFieldLayout read(Path fmapPath) throws Exception {
    JsonNode root = new ObjectMapper().readTree(fmapPath.toFile());
    double length = root.path("fieldlength").asDouble(Double.NaN);
    double width = root.path("fieldwidth").asDouble(Double.NaN);
    JsonNode fiducials = root.path("fiducials");
    if (!Double.isFinite(length) || !Double.isFinite(width) || length <= 0 || width <= 0
        || !fiducials.isArray() || fiducials.isEmpty()) {
      throw new IllegalArgumentException("FMAP must contain field dimensions and fiducials");
    }
    List<AprilTag> tags = new ArrayList<>();
    for (JsonNode fiducial : fiducials) {
      int id = fiducial.path("id").asInt(-1);
      JsonNode matrix = fiducial.path("transform");
      if (id <= 0 || !matrix.isArray() || matrix.size() != 16)
        throw new IllegalArgumentException("Invalid fiducial ID or 4x4 transform");
      Matrix<N3, N3> rotationMatrix = new Matrix<>(Nat.N3(), Nat.N3());
      for (int row=0; row<3; row++) for (int col=0; col<3; col++) {
        double value=matrix.get(row*4+col).asDouble(Double.NaN);
        if (!Double.isFinite(value)) throw new IllegalArgumentException("Non-finite tag rotation");
        rotationMatrix.set(row,col,value);
      }
      double x=matrix.get(3).asDouble(Double.NaN)+length/2;
      double y=matrix.get(7).asDouble(Double.NaN)+width/2;
      double z=matrix.get(11).asDouble(Double.NaN);
      if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(z))
        throw new IllegalArgumentException("Non-finite tag translation");
      Pose3d pose=new Pose3d(new Translation3d(x,y,z),new Rotation3d(rotationMatrix));
      tags.add(new AprilTag(id,pose));
    }
    return new AprilTagFieldLayout(tags,length,width);
  }

  public static void convert(Path fmapPath, Path wpilibJsonPath) throws Exception {
    AprilTagFieldLayout layout=read(fmapPath);
    if (layout.getTagPose(28).isEmpty()) throw new IllegalArgumentException("Source FMAP does not contain Tag 28");
    layout.serialize(wpilibJsonPath);
  }

  public static void main(String[] args) throws Exception {
    if (args.length != 2) throw new IllegalArgumentException("Usage: <source.fmap> <output.json>");
    convert(Path.of(args[0]),Path.of(args[1]));
  }
}
