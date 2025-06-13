//package org.firstinspires.ftc.teamcode.autons;
//
//import android.util.Size;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.RobotLog;
//import com.qualcomm.robotcore.util.SortOrder;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.vision.ExcludePipeline;
//import org.firstinspires.ftc.teamcode.vision.YellowPipeline;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
//import org.firstinspires.ftc.vision.opencv.ColorRange;
//import org.firstinspires.ftc.vision.opencv.ImageRegion;
//import org.opencv.core.RotatedRect;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvWebcam;
//
//import java.util.Arrays;
//import java.util.List;
//import java.util.Locale;
//
//public class PipeCamera {
//    /*
//     * EDIT THESE PARAMETERS AS NEEDED
//     */
//    final boolean USING_WEBCAM = true;
//    final BuiltinCameraDirection INTERNAL_CAM_DIR = BuiltinCameraDirection.BACK;
//    final int RESOLUTION_WIDTH = 640;
//    final int RESOLUTION_HEIGHT = 480;
//
//    OpenCvWebcam camera;
//    ExcludePipeline exclude;
//    FtcDashboard dashboard;
//
//    public PipeCamera(HardwareMap hardwareMap, Telemetry telemetry, boolean chamberPos, String color1, String color2) {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
//        if (color1.equals("yellow")) {
//
//            exclude = new ExcludePipeline();
//        }
//
//        camera.setPipeline(exclude);
//        RobotLog.dd("everything working so far","ue");
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened(){
//                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT); // 320, 240 res
//                telemetry.addLine("Camera streaming started");
//                dashboard.startCameraStream(camera, 10);
//            }
//            @Override
//            public void onError(int errorCode)
//            {
//                telemetry.addData("error", errorCode);
//            }
//        });
//    }
//    public double[] getCenter(){
//        return exclude.getCenter();
//    }
//    public int getCurrent(){
//        return exclude.getColor();
//    }
//    public void swapInt(int swap){
//        exclude.setColor(swap);
//    }
//    public void swapNext(){
//        int newy = exclude.getColor()+1;
//        if (newy > 2) newy = 0;
//        exclude.setColor(newy);
//    }
//    public double getLatency(){
//        return .001*camera.getTotalFrameTimeMs();
//    }
//    public void swapRed(){
//
//        exclude.setColor(0);
//    }
//    public void swapBlue(){
//        exclude.setColor(1);
//    }
//    public void swapYellow(){
//        exclude.setColor(2);
//    }
//    public void resetCenter(){
//        exclude.resetCenter();
//    }
//    public void swapTeleRed() {exclude.isBlue = false;
//        exclude.setColor(0);}
//    public void swapTeleBlue() {exclude.isBlue = true; exclude.setColor(1);}
//
//    public class computeTargetPose implements Action {
//        ElapsedTime time = new ElapsedTime();
//        boolean start;
//        double runTime;
//        public computeTargetPose(double waittim){
//            start = false;
//            runTime = waittim;
//        }
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            if (!start) {
//                time.reset();
//                start = true;
//            }
//            if (time.seconds() < runTime) {
//                return true;
//            }
//            else {
//                double[] relCent = getCenter().clone();
//
//                if (!Arrays.equals(relCent, new double[]{0, 0, 0, 0})) {
//                    resetCenter();
////                    relCent[0] = (relCent[2] * Math.sin(arm.getRot() * PI / 180) + relCent[0] * Math.cos(arm.getRot() * PI / 180) - FOR_CONST) * FOR_MULT;
//                    packet.put("relCent0", relCent[0]);
//                    packet.put("relCent1", relCent[1]);
////                    packet.put("rotVel", follower.getVelocityPose().getHeading());
////                    if (relCent[0] * relCent[0] + relCent[1] * relCent[1] < 200) {
////                        if (time - lastMoveTime > MOVE_INTERVAL) {
////                            Vector2d relVect = new Vector2d(0, ((-relCent[1] + Math.signum(-relCent[1]) * SIDE_CONST)) * SIDE_MULT
////                                    - 1.2 * follower.getStableRotVelo().getY() * cv.getLatency()).rotated(follower.getPose().getHeading());
////                            Vector2d relVect2 = new Vector2d(0, (-relCent[1] * SIDE_MULT))
////                                    .rotated(follower.getPose().getHeading());
////                            Pose pos = follower.getPose();
////                            pos.add(new Pose(relVect.getX(), relVect.getY(), 0));
////                            Pose pos2 = follower.getPose();
////                            pos2.add(new Pose(relVect2.getX(), relVect2.getY(), 0));
////                            double newExt = Math.max(arm.getExt() + relCent[0] - (-arm.getVel() + follower.getStableRotVelo().getX()) * cv.getLatency() , MIN_EXT);
////
////                            if (newExt > arm.getExt() + 8 || newExt > MAX_EXT) {
////                                targeted = false;
////                            } else {
//////                                    if (newExt > MIN_EXT + .1) {
////                                flip.flipTo(Flip.FlipStates.SUBMERSIBLE);
//////                                    }
////                                double head = follower.getPose().getHeading()/* - 0.5*follower.getStableRotVelo().getHeading()*cv.getLatency()*/;
//////                                    if (follower.getCurrentPath() != null) {
//////                                        head = follower.getCurrentPath().getHeadingGoal(1);
//////                                    }
////                                Point curTarg = lastTarg;
////                                Point newTarg = new Point(pos2);
////                                if (curTarg == null) {
////                                    follower.holdPoint(new BezierPoint(new Point(pos)), head);
////                                } else if (curTarg.distanceFrom(newTarg) > 0.05) {
////                                    follower.holdPoint(new BezierPoint(new Point(pos)), head);
////                                }
//////                                    if(newExt>arm.getTargetExt()+.25){
//////                                        newExt+=.5;
//////                                    }
////                                if (newExt < arm.getTargetExt() - .25) {
////                                    newExt -= RETRACT_CONST;
////                                }
////                                arm.goToResetManual(newExt, Math.atan2(3, newExt + 12) * 180 / PI);
////                                twist.twistToAng(relCent[3]);
////                                packet.put("newExt", newExt);
////                                packet.put("relVect", relVect);
////                                packet.put("relAng", relCent[3]);
////                                packet.put("latency", cv.getLatency());
////                                lastTarg = new Point(pos2);
////                                lastMoveTime = time;
////                            }
////                        }
////                    }
//                    PoseStorage.grabColorPose = new Pose2d(relCent[0],relCent[1],Math.toRadians(90));
//                }
//                resetCenter();
//                RobotLog.dd("SUB GRAB POSE", PoseStorage.grabColorPose.position.x+","+PoseStorage.grabColorPose.position.y+","+PoseStorage.grabColorPose.heading.toDouble());
//                return false;
//            }
//        }
//    }
//    public Action comp (double tim) {
//        return new computeTargetPose(tim);
//    }
//}
