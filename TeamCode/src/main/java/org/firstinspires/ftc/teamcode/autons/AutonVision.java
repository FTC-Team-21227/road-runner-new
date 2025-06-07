package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import org.firstinspires.ftc.teamcode.vision.YellowPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "AutonVisionTesting")
public class AutonVision extends LinearOpMode {
//  VisionPortal myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "/Users/gracesong/Downloads/IMG_4852.png"), );


    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
        YellowPipeline yellowPipeline = new YellowPipeline(telemetry);

        camera.setPipeline(yellowPipeline);
        telemetry.addLine("everything working so far... :(");
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
                telemetry.addData("error", errorCode);
            }
        });
        telemetry.update();

        waitForStart();

    }
}
