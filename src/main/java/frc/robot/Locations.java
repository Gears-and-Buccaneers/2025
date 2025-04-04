package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.function.IntPredicate;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Locations {
    private static AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    private static Pose2d[] genPoses(int[] tags, Translation2d tagToBase, double step, int stepStart,
            int stepEnd) {
        int nSteps = stepEnd - stepStart + 1;

        Pose2d[] poses = new Pose2d[tags.length * nSteps];

        int i = 0;

        Transform2d tagToBaseXform = new Transform2d(tagToBase, Rotation2d.k180deg);

        for (int tag : tags) {
            var base = field.getTagPose(tag).get().toPose2d().transformBy(tagToBaseXform);

            for (int j = stepStart; j <= stepEnd; j++) {
                poses[i++] = base.transformBy(new Transform2d(0.0, step * j, Rotation2d.kZero));
            }
        }

        return poses;
    }

    // Branch heights in inches for levels 2 through 4.
    public static final double L2 = 31.72;
    public static final double L3 = 47.59;
    public static final double L4 = 71.87;

    public static double coralOffset = 0.05;
    public static double branchOffset = 0.164;

    public static Pose2d[] station = genPoses(new int[] { 1, 2, 13, 12 },
            new Translation2d(0.5, -coralOffset), Inches.of(8.0).in(Meters), -4, 4);

    public static IntPredicate stationIsRed = i -> i < 18;
    public static IntPredicate stationIsLeft = i -> i % 18 < 9;

    private static int[] reefIds = new int[] { 7, 8, 9, 10, 11, 6, 18, 17, 22, 21, 20, 19 };

    public static Pose2d[] reef = genPoses(reefIds, Translation2d.kZero, 0, 0, 0);

    public static Pose2d[] branches = genPoses(reefIds,
            new Translation2d(0.64, branchOffset - coralOffset), branchOffset * 2, 0, 1);

    public static IntPredicate branchIsRed = i -> i < 12;
    public static IntPredicate branchIsLeft = i -> switch (i % 12) {
        // If the index is for one of the front three reef zones, then it is left if it
        // is the second.
        case 1, 3, 11 -> true;
        // If it's for the back, then it's on the left if it's first.
        case 4, 6, 8 -> true;
        // Otherwise, it must be on the right side.
        default -> false;
    };

    public static Pose2d[] cage = genPoses(new int[] { 5, 14 },
            new Translation2d(0.0, 0.0), 1.0906104, -1, 1);

    public static IntPredicate cageIsRed = i -> i < 3;

    public static Pose2d[] processor = genPoses(new int[] { 3, 16 },
            new Translation2d(0.0, 0.0), 0.0, 0, 0);

    public static IntPredicate processorIsRed = i -> i == 0;

    public static Pose2d[] withPredicate(Pose2d[] poses, IntPredicate pred) {
        ArrayList<Pose2d> out = new ArrayList<>();

        for (int i = 0; i < poses.length; i++)
            if (pred.test(i))
                out.add(poses[i]);

        return out.toArray(Pose2d[]::new);
    }

    public static void publish() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("Locations");

        table.getStructArrayTopic("Station", Pose2d.struct).publish().set(station);
        table.getStructArrayTopic("Branches", Pose2d.struct).publish().set(branches);
        table.getStructArrayTopic("Cage", Pose2d.struct).publish().set(cage);
    }
}
