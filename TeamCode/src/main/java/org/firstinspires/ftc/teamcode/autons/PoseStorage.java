package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.roadrunner.Pose2d;

public class PoseStorage {
    public static Pose2d currentPose = new Pose2d(0,0,Math.toRadians(90));
    public static Pose2d grabColorPose = new Pose2d(10.5, -43,Math.toRadians(90));
    public static Pose2d grabYellowPose1 = new Pose2d(64.5-2, 94,Math.toRadians(-90));
    public static Pose2d grabYellowPose2 = new Pose2d(64.5-2, 94,Math.toRadians(-90));
    public static double target1 = 0;
    public static double target2 = 0;
    public static ARM1_V3Robot arm1;
    public static ARM2_V3Robot arm2;
}
