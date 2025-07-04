//package org.firstinspires.ftc.teamcode.autons;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.TunePID;
//
//public class ARM2_NEW {
//    private DcMotor arm2;
//    //PID controllers for ARM1 and ARM2
//    private PIDController controller2;
//    //PIDF gains
//    double p2 = TunePID.p2, i2 = TunePID.i2, d2 = TunePID.d2;
//    double f2 = TunePID.f2;
//    //ticks to degrees conversion
//    private final double ticks_in_degree_2 = 11.2855555556;
//    //length, COM, mass values for feedforward calculation (not even performed in arm2)
//    private final double L1 = 43.2;
//    private final double L2 = 43.2;
//    private final double x1 = 36.96;
//    private final double x2 = 26.4;
//    private final double m1 = 810;
//    private final double m2 = 99.79;
//
//    public ARM2_NEW(HardwareMap hardwareMap) {
//        arm2 = hardwareMap.get(DcMotor.class, "ARM2");
//        arm2.setDirection(DcMotor.Direction.REVERSE); //CHANGE BACK TO DCMOTORSIMPLE IF SOMETHING DOESN'T WORK
//        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        controller2 = new PIDController(p2, i2, d2);
//    }
//    public class LiftTarget implements Action {
//        ElapsedTime time = new ElapsedTime();
//        boolean start;
//        double target2;
//        double runTime;
//        public LiftTarget(double pos) {
//            target2 = pos;
//            start = false;
//            runTime = 2;
//        }
//        public LiftTarget(double pos, double runtime) {
//            target2 = pos;
//            start = false;
//            runTime = runtime;
//        }
//        public double ARM_Control_PID() {
//            int arm2Pos = arm2.getCurrentPosition();
//            double pid2 = controller2.calculate(arm2Pos, (int) (target2 * ticks_in_degree_2)); //PID calculation
//            double ff2 = 0; //feedforward calculation, change when equation is derived
//            return ((pid2/* + ff2*/));
//        }
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            if (!start) {
//                PoseStorage.target2 = target2; //NEW
//                time.reset();
//                start = true;
//            }
//            packet.addLine("time.seconds():"+(time.seconds()));
//            packet.addLine("arm2pos:"+(arm2.getCurrentPosition()));
//            packet.addLine("target2pos:"+target2);
//            packet.addLine("target2pos:"+(int)(target2*ticks_in_degree_2));
//            if (time.seconds() < runTime) {
//                packet.addLine("still running 2");
//                double power = ARM_Control_PID();
//                packet.addLine("power2:"+power);
//                arm2.setPower(power);
//                return true;
//            } else {
//                arm2.setPower(0);
//                return false;
//            }
//        }
//    }
//    public Action liftHighBasket() {return new LiftTarget(169.4201);}
//    public Action liftRung() {return new LiftTarget(95.3431);}
//    public Action liftWall() {return new LiftTarget(154.1794);}
//    public Action liftLowBasket() {return new LiftTarget(50);} //not tested i think
//    public Action liftFloor() {return new LiftTarget(164.6352);}
//    public Action liftDown() {return new LiftTarget(5.0199819357);}
//    public Action liftHighBasket(double seconds) {return new LiftTarget(169.4201, seconds);}
//    public Action liftRung(double seconds) {return new LiftTarget(95.3431, seconds);}
//    public Action liftWall(double seconds) {return new LiftTarget(154.1794, seconds);}
//    public Action liftLowBasket(double seconds) {return new LiftTarget(50, seconds);} //not tested i think
//    public Action liftFloor(double seconds) {return new LiftTarget(164.6352, seconds);}
//    public Action liftDown(double seconds) {return new LiftTarget(5.0199819357, seconds);}
//
//    public class waitLiftTarget implements Action {
//        ElapsedTime time = new ElapsedTime();
//        boolean start;
//        boolean startMove;
//        double target2;
//        double waitTime;
//        double runTime;
//        public waitLiftTarget(double pos) {
//            target2 = pos;
//            start = false;
//            startMove = false;
//            waitTime = 1;
//            runTime = 2;
//        }
//        public waitLiftTarget(double pos, double tim) {
//            target2 = pos;
//            start = false;
//            startMove = false;
//            waitTime = tim;
//            runTime = 2;
//        }
//        public waitLiftTarget(double pos, double tim, double runtime) {
//            target2 = pos;
//            start = false;
//            startMove = false;
//            waitTime = tim;
//            runTime = runtime;
//        }
//        public double ARM_Control_PID() {
//            int arm2Pos = arm2.getCurrentPosition();
//            double pid2 = controller2.calculate(arm2Pos, (int) (target2 * ticks_in_degree_2)); //PID calculation
//            double ff2 = 0; //feedforward calculation, change when equation is derived
//            return ((pid2/* + ff2*/));
//        }
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            if (!start) {
//                time.reset();
//                start = true;
//            }
//            packet.addLine("time.seconds():"+(time.seconds()));
//            packet.addLine("arm2pos:"+(arm2.getCurrentPosition()));
//            packet.addLine("target2pos:"+target2);
//            packet.addLine("target2pos:"+(int)(target2*ticks_in_degree_2));
//            if (time.seconds() < runTime+waitTime) {
//                packet.addLine("still running 2");
//                if (time.seconds()>waitTime){
//                    //NEW
//                    if (!startMove){
//                        PoseStorage.target2 = target2;
//                        startMove = true;
//                    }
//                    //
//                    double power = ARM_Control_PID();
//                    packet.addLine("power2:"+power);
//                    arm2.setPower(power);}
//                else{
//                    arm2.setPower(0);
//                }
//                return true;
//            } else {
//                arm2.setPower(0);
//                return false;
//            }
//        }
//    }
//    public Action waitLiftHighBasket() {return new waitLiftTarget(169.4201);}
//    public Action waitLiftRung() {return new waitLiftTarget(95.3431);}
//    public Action waitLiftWall() {return new waitLiftTarget(154.1794);}
//    public Action waitLiftLowBasket() {return new waitLiftTarget(50);} //not tested i think
//    public Action waitLiftFloor() {return new waitLiftTarget(164.6352);}
//    public Action waitLiftDown() {return new waitLiftTarget(5.0199819357);}
//    public Action waitLiftHighBasket(double waitseconds) {return new waitLiftTarget(169.4201,waitseconds);}
//    public Action waitLiftRung(double waitseconds) {return new waitLiftTarget(95.3431,waitseconds);}
//    public Action waitLiftWall(double waitseconds) {return new waitLiftTarget(154.1794,waitseconds);}
//    public Action waitLiftLowBasket(double waitseconds) {return new waitLiftTarget(50,waitseconds);} //not tested i think
//    public Action waitLiftFloor(double waitseconds) {return new waitLiftTarget(164.6352,waitseconds);}
//    public Action waitLiftDown(double waitseconds) {return new waitLiftTarget(5.0199819357,waitseconds);}
//    public Action waitLiftHighBasket(double waitseconds, double seconds) {return new waitLiftTarget(169.4201,waitseconds,seconds);}
//    public Action waitLiftRung(double waitseconds, double seconds) {return new waitLiftTarget(95.3431,waitseconds,seconds);}
//    public Action waitLiftWall(double waitseconds, double seconds) {return new waitLiftTarget(154.1794,waitseconds,seconds);}
//    public Action waitLiftLowBasket(double waitseconds, double seconds) {return new waitLiftTarget(50,waitseconds,seconds);} //not tested i think
//    public Action waitLiftFloor(double waitseconds, double seconds) {return new waitLiftTarget(164.6352,waitseconds,seconds);}
//    public Action waitLiftDown(double waitseconds, double seconds) {return new waitLiftTarget(5.0199819357,waitseconds,seconds);}
//}