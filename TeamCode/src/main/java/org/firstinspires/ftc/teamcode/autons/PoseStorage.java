package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.roadrunner.Pose2d;

public class PoseStorage {
    public static Pose2d currentPose = new Pose2d(0,0,Math.toRadians(90));
    public static Pose2d grabColorPose = new Pose2d(10.5, -45,Math.toRadians(90));
    public static Pose2d grabYellowPose = new Pose2d(66.5, 90,Math.toRadians(-90));
    public static double target1 = 0;
    public static double target2 = 0;
}
