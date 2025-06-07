package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.roadrunner.Pose2d;

public class PoseStorage {
    public static Pose2d currentPose = new Pose2d(0,0,Math.toRadians(90));
    public static double target1 = 0;
    public static double target2 = 0;
    public static ARM1_V3Robot arm1;
    public static ARM2_V3Robot arm2;
}
