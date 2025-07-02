//package org.firstinspires.ftc.teamcode.autons.archive;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//public class INTAKE_ANGLE {
//    private Servo Intake_Angle;
//
//    public INTAKE_ANGLE(HardwareMap hardwareMap) {
//        Intake_Angle = hardwareMap.get(Servo.class, "Intake_Angle");
//    }
//
//    public class RotatePosition0 implements Action {
//        ElapsedTime time = new ElapsedTime();
//        boolean start;
//        double runTime;
//        public RotatePosition0(){
//            start = false;
//            runTime = 0.5;
//        }
//        public RotatePosition0(double runt){
//            start = false;
//            runTime = runt;
//        }
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            if (!start) {
//                time.reset();
//                start = true;
//            }
//            if (time.seconds() < runTime) {
//                return true;
//            } else {
//                Intake_Angle.setPosition(0.26); //0.28 //0.65
//                return false;
//            }
//        }
//    }
//    public Action RotatePosition0() {
//        return new RotatePosition0();
//    }
//    public Action RotatePosition0(double runt) {
//        return new RotatePosition0(runt);
//    }
//
//    public class RotatePosition1 implements Action {
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            Intake_Angle.setPosition(0.8); //0.98
//            return false;
//        }
//    }
//    public Action RotatePosition1() {
//        return new RotatePosition1();
//    }
//
//    public class RotatePositionNegative1 implements Action {
//        ElapsedTime time = new ElapsedTime();
//        boolean start;
//        double runTime;
//        public RotatePositionNegative1(){
//            start = false;
//            runTime = 0.5;
//        }
//        public RotatePositionNegative1(double runt){
//            start = false;
//            runTime = runt;
//        }
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            if (!start) {
//                time.reset();
//                start = true;
//            }
//            if (time.seconds() < runTime) {
//                return true;
//            } else {
//                Intake_Angle.setPosition(-0.064); //0.98
//                return false;
//            }
//        }
//    }
//    public Action RotatePositionNegative1() {
//        return new RotatePositionNegative1();
//    }
//    public Action RotatePositionNegative1(double runt) {
//        return new RotatePositionNegative1(runt);
//    }
//
//}