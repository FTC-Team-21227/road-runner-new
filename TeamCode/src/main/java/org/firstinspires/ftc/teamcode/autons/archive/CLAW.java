//package org.firstinspires.ftc.teamcode.autons.archive;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//public class CLAW {
//    private Servo Claw;
//    public CLAW(HardwareMap hardwareMap) {
//        Claw = hardwareMap.get(Servo.class, "Claw");
//        Claw.scaleRange(0.45,1);
//    }
//
//    public class CloseClaw implements Action {
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            Claw.setPosition(1);
//            return false;
//        }
//    }
//    public Action closeClaw() {
//        return new CloseClaw();
//    }
//
//    public class OpenClaw implements Action {
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            Claw.setPosition(0);
//            return false;
//        }
//    }
//    public Action openClaw() {
//        return new OpenClaw();
//    }
//
//    public class OpenSlightly implements Action {
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            Claw.setPosition(0.5);
//            return false;
//        }
//    }
//    public Action openSlightly() {
//        return new OpenSlightly();
//    }
//}
