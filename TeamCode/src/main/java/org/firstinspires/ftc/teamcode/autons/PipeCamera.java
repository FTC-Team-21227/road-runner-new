package org.firstinspires.ftc.teamcode.autons;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.ExcludePipeline;
import org.firstinspires.ftc.teamcode.vision.YellowPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;
import java.util.List;
import java.util.Locale;

public class PipeCamera {
    /*
     * EDIT THESE PARAMETERS AS NEEDED
     */
    boolean USING_WEBCAM = true;
    final BuiltinCameraDirection INTERNAL_CAM_DIR = BuiltinCameraDirection.BACK;
    final int RESOLUTION_WIDTH = 640;
    final int RESOLUTION_HEIGHT = 480;
    boolean chamberPos;
    boolean printStuff = ExcludePipeline.printStuff;

    OpenCvWebcam camera;
    ExcludePipeline exclude;
//    FtcDashboard dashboard;

    public PipeCamera(HardwareMap hardwareMap, Telemetry telemetry, boolean chamberPos, String color, boolean inclYellow) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
        this.chamberPos = chamberPos;
        exclude = new ExcludePipeline(telemetry,chamberPos);
        if (inclYellow && color.equals("yellow")) {
            swapInt(3);
        }
        else if (!inclYellow && color.equals("red")) {
            swapRed();
        }
        else if (!inclYellow && color.equals("blue")) {
            swapBlue();
        }
        else if (inclYellow && color.equals("red")) {
            swapTeleRed();
        }
        else if (inclYellow && color.equals("blue")) {
            swapTeleBlue();
        }

        camera.setPipeline(exclude);
        if (printStuff) RobotLog.dd("everything working so far","ue");
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened(){
                camera.startStreaming(RESOLUTION_WIDTH, RESOLUTION_HEIGHT, OpenCvCameraRotation.UPRIGHT); // 320, 240 res
//                camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.NATIVE_VIEW);
                if (printStuff) {
                    telemetry.addLine("Camera streaming started");
                    telemetry.update();
                    FtcDashboard.getInstance().startCameraStream(camera, 10);
                }
            }
            @Override
            public void onError(int errorCode)
            {
                if (printStuff) RobotLog.dd("error", errorCode + "");
            }
        });
    }
    public double[] getCenter(@NonNull TelemetryPacket packet){
        return exclude.getCenter(packet);
    }
    public int getCurrent(){
        return exclude.getColor();
    }
    public void swapInt(int swap){
        exclude.setColor(swap);
    }
    public void swapNext(){
        int newy = exclude.getColor()+1;
        if (newy > 2) newy = 0;
        exclude.setColor(newy);
    }
    public double getLatency(){
        return .001*camera.getTotalFrameTimeMs();
    }
    public void swapRed(){
        exclude.setColor(0);
    }
    public void swapBlue(){
        exclude.setColor(1);
    }
    public void swapYellow(){
        exclude.setColor(3);
    }
    public void resetCenter(){
        exclude.resetCenter();
    }
    public void swapTeleRed() {
        ExcludePipeline.isBlue = false;
        exclude.setColor(2);
    }
    public void swapTeleBlue() {
        ExcludePipeline.isBlue = true;
        exclude.setColor(2);
    }
    public void off(){
        camera.closeCameraDeviceAsync(() -> {
//                    camera.stopRecordingPipeline();
        });
        USING_WEBCAM = false;
    }

    public class computeTargetPose implements Action {
        ElapsedTime time = new ElapsedTime();
        boolean start;
        double runTime;
        public computeTargetPose(double waittim){
            start = false;
            runTime = waittim;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!start) {
                time.reset();
                start = true;
            }
            if (time.seconds() < runTime) {
                return true;
            }
            else if (USING_WEBCAM){
                double[] relCent = getCenter(packet).clone();

                if (!Arrays.equals(relCent, new double[]{0, 0, 0, 0})) {
                    resetCenter();
//
                    if (printStuff) {
                        packet.put("relCent0", relCent[0]+"");
                        packet.put("relCent1", relCent[1]+"");
                        packet.put("angle", relCent[3]+"");
                        RobotLog.dd("relCent0", relCent[0] + "");
                        RobotLog.dd("relCent1", relCent[1] + "");
                        RobotLog.dd("angle", relCent[3] + "");
                    }
//
                    if (chamberPos) {
                        double x = 10.5; //PoseStorage.grabColorPose.position.x;
                        if (PoseStorage.grabColorPose.position.x < 5.5) x = 0;
                        double y = -45; //PoseStorage.grabColorPose.position.y;
                        PoseStorage.grabColorPose = new Pose2d(x + relCent[0], y + relCent[1], Math.toRadians(90));
                    }
                    else{
                        double x = PoseStorage.grabYellowPose.position.x;
                        double y = /*90;*/ PoseStorage.grabYellowPose.position.y;
                        PoseStorage.grabYellowPose = new Pose2d(x + relCent[0], y + relCent[1], Math.toRadians(-90));
                    }
                }
                else if (printStuff){
//                    packet.addLine("no relCent detected");
                    RobotLog.d("no relCent detected");
                }
                if (printStuff) {
                    if (chamberPos) {
                    packet.put("Pose Storage x", PoseStorage.grabColorPose.position.x+"");
                    packet.put("Pose Storage y", PoseStorage.grabColorPose.position.y+"");
                        RobotLog.dd("Pose Storage x", PoseStorage.grabColorPose.position.x + "");
                        RobotLog.dd("Pose Storage y", PoseStorage.grabColorPose.position.y + "");
                    } else {
                    packet.put("Pose Storage x", PoseStorage.grabYellowPose.position.x+"");
                    packet.put("Pose Storage y", PoseStorage.grabYellowPose.position.y+"");
                        RobotLog.dd("Pose Storage x", PoseStorage.grabYellowPose.position.x + "");
                        RobotLog.dd("Pose Storage y", PoseStorage.grabYellowPose.position.y + "");
                    }
                }
                resetCenter();
//                camera.stopStreaming();
//                camera.closeCameraDevice();
//                RobotLog.dd("SUB GRAB POSE", PoseStorage.grabColorPose.position.x+","+PoseStorage.grabColorPose.position.y+","+PoseStorage.grabColorPose.heading.toDouble());
                return false;
            }
            else return false;
        }
    }
    public Action comp (double tim) {
        return new computeTargetPose(tim);
    }
    public class Close implements Action {
        ElapsedTime time = new ElapsedTime();
        boolean start;
        double runTime;
        public Close(double waittim){
            start = false;
            runTime = waittim;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!start) {
                time.reset();
                start = true;
            }
            if (time.seconds() < runTime) {
                return true;
            }
            else {
                camera.closeCameraDeviceAsync(() -> {
//                    camera.stopRecordingPipeline();
                }
                );
                return false;
            }
        }
    }
    public Action close (double tim) {
        return new Close(tim);
    }
}
