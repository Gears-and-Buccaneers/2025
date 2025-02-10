package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Locations {
    private static AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public static Pose2d[][] station = genPoses(new int[] { 1, 2, 12, 13 },
            new Transform2d(0.0, 0.0, Rotation2d.k180deg),
            new Transform2d(Inches.zero(), Inches.of(8.0), Rotation2d.kZero), -4, 4);

    public static Pose2d[][] reef = genPoses(new int[] { 7, 8, 9, 10, 11, 6, 18, 17, 22, 21, 20, 19 },
        new Transform2d(0.0, 0.0, Rotation2d.k180deg),
        new Transform2d(Inches.zero(), Inches.of(12.937677), Rotation2d.kZero), 0, 1);

     public static Pose2d[][] cage = genPoses(new int[] { 5, 14 },
        new Transform2d(0.0, 0.0, Rotation2d.k180deg),
        new Transform2d(Inches.zero(), Inches.of(42.937416), Rotation2d.kZero), -1, 1);

    private static Pose2d[][] genPoses(int[] tags, Transform2d tagToBase, Transform2d step, int stepStart,
            int stepEnd) {
        Pose2d[][] poses = new Pose2d[tags.length][stepEnd - stepStart + 1];

        for (int tagI = 0; tagI < tags.length; tagI++) {
            var base = field.getTagPose(tags[tagI]).get().toPose2d().transformBy(tagToBase);
            for (int i = stepStart; i <= stepEnd; i++)
                poses[tagI][i - stepStart] = base.transformBy(step.times(i));
        }

        return poses;
    }

    public static void telemeterise() {
        NetworkTable locations = NetworkTableInstance.getDefault().getTable("Locations");

        telemeterisePoses(Locations.station, locations.getSubTable("Station"));
        telemeterisePoses(Locations.reef, locations.getSubTable("Reef"));
        telemeterisePoses(Locations.cage, locations.getSubTable("Cage"));
    }

    private static void telemeterisePoses(Pose2d[][] poses, NetworkTable target) {
        for (int i = 0; i < poses.length; i++) {
            NetworkTable sub = target.getSubTable(Integer.toString(i));

            for (int j = 0; j < poses[i].length; j++) {
                sub.getStructTopic(Integer.toString(j), Pose2d.struct).publish().set(poses[i][j]);
            }
        }
    }
}
