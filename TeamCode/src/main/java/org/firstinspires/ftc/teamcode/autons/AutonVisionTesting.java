package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.YellowPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

//@Autonomous(name = "AutonVisionTesting")
public class AutonVisionTesting extends LinearOpMode {
//  VisionPortal myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "/Users/gracesong/Downloads/IMG_4852.png"), );


    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
        YellowPipeline yellowPipeline = new YellowPipeline(telemetry);

        camera.setPipeline(yellowPipeline);
        telemetry.addLine("everything working so far... :(");
        RobotLog.dd("everything working so far","ue");
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened(){
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT); // 320, 240 res
                telemetry.addLine("Camera streaming started");
            }
            @Override
            public void onError(int errorCode)
            {
                camera.stopStreaming();
                telemetry.addData("error", errorCode);
            }
        });
        telemetry.update();

        waitForStart();

    }
}