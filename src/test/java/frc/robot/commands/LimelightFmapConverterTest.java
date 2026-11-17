package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.utility.LimelightFmapConverter;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

class LimelightFmapConverterTest {
  @Test void convertedTag28MatchesFmapTranslationRotationAndFieldDimensions() throws Exception {
    Path source=Path.of("src/test/resources/fiducials.fmap");
    Path converted=Path.of("src/main/deploy/hallway-28.json");
    JsonNode root=new ObjectMapper().readTree(source.toFile());
    JsonNode source28=null;
    for(JsonNode tag:root.path("fiducials")) if(tag.path("id").asInt()==28) source28=tag;
    assertNotNull(source28,"The selected FMAP must contain ID 28");

    AprilTagFieldLayout layout=new AprilTagFieldLayout(converted);
    Pose3d result=layout.getTagPose(28).orElseThrow();
    JsonNode transform=source28.path("transform");
    assertEquals(transform.get(3).asDouble()+root.path("fieldlength").asDouble()/2,result.getX(),1e-9);
    assertEquals(transform.get(7).asDouble()+root.path("fieldwidth").asDouble()/2,result.getY(),1e-9);
    assertEquals(transform.get(11).asDouble(),result.getZ(),1e-9);
    Matrix<N3,N3> sourceRotation=new Matrix<>(Nat.N3(),Nat.N3());
    for(int row=0;row<3;row++) for(int col=0;col<3;col++)
      sourceRotation.set(row,col,transform.get(row*4+col).asDouble());
    assertEquals(new Rotation3d(sourceRotation),result.getRotation());
    assertEquals(root.path("fieldlength").asDouble(),layout.getFieldLength(),1e-9);
    assertEquals(root.path("fieldwidth").asDouble(),layout.getFieldWidth(),1e-9);
    assertEquals(root.path("fiducials").size(),layout.getTags().size());
  }

  @Test void converterOutputMatchesCheckedInHallwayLayout() throws Exception {
    Path regenerated=Path.of("build/tmp/fmap-converter-test.json");
    LimelightFmapConverter.convert(Path.of("src/test/resources/fiducials.fmap"),regenerated);
    AprilTagFieldLayout expected=new AprilTagFieldLayout(Path.of("src/main/deploy/hallway-28.json"));
    AprilTagFieldLayout actual=new AprilTagFieldLayout(regenerated);
    assertEquals(expected.getTagPose(28),actual.getTagPose(28));
    assertEquals(expected.getFieldLength(),actual.getFieldLength(),1e-9);
    assertEquals(expected.getFieldWidth(),actual.getFieldWidth(),1e-9);
  }
}
