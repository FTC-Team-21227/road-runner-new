//package org.firstinspires.ftc.teamcode.autons.archive;
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
//import org.firstinspires.ftc.teamcode.Subsystem_Constants;
//import org.firstinspires.ftc.teamcode.TunePID;
//import org.firstinspires.ftc.teamcode.TunePID_MotionProfile;
//import org.firstinspires.ftc.teamcode.autons.PoseStorage;
//
//public class ARM2_V2Robot {
//    private DcMotor arm2;
//    //PID controllers for ARM1 and ARM2
//    private PIDController controller2;
//    //PIDF gains
//    double p2 = TunePID.p2, i2 = TunePID.i2, d2 = TunePID.d2;
//    double f2 = TunePID.f2;
//    //ticks to degrees conversion
//    private final double ticks_in_degree_2 = TunePID_MotionProfile.ticks_in_degree_2;
//    //length, COM, mass values for feedforward calculation (not even performed in arm2)
//    private final double L1 = 43.2;
//    private final double L2 = 43.2;
//    private final double x1 = 36.96;
//    private final double x2 = 26.4;
//    private final double m1 = 810;
//    private final double m2 = 99.79;
//    private final double highBasket2 = Subsystem_Constants.highBasket2_auto;
//    final double highRung2 = Subsystem_Constants.highRung2;
//    final double highRung2_2 = Subsystem_Constants.highRung2_2;
//    final double highRung2_First = Subsystem_Constants.highRung2_First;
//    private final double wall2 = Subsystem_Constants.wall2;
//    private final double wall2_2 = Subsystem_Constants.wall2_2;
//    private final double wall2_First = Subsystem_Constants.wall2_First;
//    private final double lowBasket2 = Subsystem_Constants.lowBasket2;
//    private final double floor2 = Subsystem_Constants.floor2;
//    private final double down2 = Subsystem_Constants.down2;
//    private final double sub2 = Subsystem_Constants.sub2;
//    private final double vertSub2 = Subsystem_Constants.vertSub2;
//    private final double vertFloor2 = Subsystem_Constants.vertFloor2;
//
//    public ARM2_V2Robot(HardwareMap hardwareMap) {
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
//        double power = 1;
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
//        public LiftTarget(double pos, double runtime, double pwr) {
//            target2 = pos;
//            start = false;
//            runTime = runtime;
//            power = pwr;
//        }
//        public double ARM_Control_PID() {
//            int arm2Pos = arm2.getCurrentPosition();
//            double pid2 = controller2.calculate(arm2Pos, (int) (target2 * ticks_in_degree_2)); //PID calculation
//            double ff2 = 0; //feedforward calculation, change when equation is derived
//            return ((pid2/* + ff2*/)) * power;
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
////            packet.addLine("target2pos:"+target2);
//            packet.addLine("target2pos:"+(int)(target2*ticks_in_degree_2));
//            if (time.seconds() < runTime) {
////                packet.addLine("still running 2");
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
//    public Action liftHighBasket() {return new LiftTarget(highBasket2);}
//    public Action liftRung() {return new LiftTarget( highRung2);}
//    public Action liftRung2() {return new LiftTarget( highRung2_2);}
//    public Action liftRung2_First() {return new LiftTarget( highRung2_First);}
//    public Action liftWall() {return new LiftTarget(wall2);}
//    public Action liftWall2() {return new LiftTarget(wall2_2);}
//    public Action liftWall2_First() {return new LiftTarget(wall2_First);}
//    public Action liftLowBasket() {return new LiftTarget(lowBasket2);} //not tested i think
//    public Action liftFloor() {return new LiftTarget(floor2);}
//    public Action liftDown() {return new LiftTarget(down2);}
//    public Action liftSub() {return new LiftTarget(sub2);}
//    public Action liftVertSub() {return new LiftTarget(vertSub2);}
//    public Action liftVertFloor() {return new LiftTarget(vertFloor2);}
//    public Action liftHighBasket(double seconds) {return new LiftTarget(highBasket2, seconds);}
//    public Action liftRung(double seconds) {return new LiftTarget(highRung2, seconds);}
//    public Action liftRung2(double seconds) {return new LiftTarget(highRung2_2, seconds);}
//    public Action liftRung2_First(double seconds) {return new LiftTarget(highRung2_First, seconds);}
//    public Action liftWall(double seconds) {return new LiftTarget(wall2, seconds);}
//    public Action liftWall2(double seconds) {return new LiftTarget(wall2_2, seconds);}
//    public Action liftWall2_First(double seconds) {return new LiftTarget(wall2_First, seconds);}
//    public Action liftLowBasket(double seconds) {return new LiftTarget(lowBasket2, seconds);} //not tested i think
//    public Action liftFloor(double seconds) {return new LiftTarget(floor2, seconds);}
//    public Action liftDown(double seconds) {return new LiftTarget(down2, seconds);}
//    public Action liftRung(double seconds, double power) {return new LiftTarget(highRung2, seconds, power);}
//    public Action liftRung2(double seconds, double power) {return new LiftTarget(highRung2_2, seconds, power);}
//    public Action liftRung2_First(double seconds, double power) {return new LiftTarget(highRung2_First, seconds, power);}
//    public Action liftRungFirst(double seconds, double power) {return new LiftTarget(highRung2+0.75,seconds,power);}
//    public Action liftSub(double seconds) {return new LiftTarget(sub2,seconds);}
//    public Action liftVertSub(double seconds) {return new LiftTarget(vertSub2, seconds);}
//    public Action liftVertFloor(double seconds) {return new LiftTarget(vertFloor2, seconds);}
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
////            packet.addLine("target2pos:"+target2);
//            packet.addLine("target2pos:"+(int)(target2*ticks_in_degree_2));
//            if (time.seconds() < runTime+waitTime) {
////                packet.addLine("still running 2");
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
////                else{
////                    arm2.setPower(0);
////                }
//                return true;
//            } else {
//                arm2.setPower(0);
//                return false;
//            }
//        }
//    }
//    public Action waitLiftHighBasket() {return new waitLiftTarget(highBasket2);}
//    public Action waitLiftRung() {return new waitLiftTarget(highRung2);}
//    public Action waitLiftRung2() {return new waitLiftTarget(highRung2_2);}
//    public Action waitLiftRung2_First() {return new waitLiftTarget(highRung2_First);}
//    public Action waitLiftWall() {return new waitLiftTarget(wall2);}
//    public Action waitLiftWall2() {return new waitLiftTarget(wall2_2);}
//    public Action waitLiftWall2_First() {return new waitLiftTarget(wall2_First);}
//    public Action waitLiftLowBasket() {return new waitLiftTarget(lowBasket2);} //not tested i think
//    public Action waitLiftFloor() {return new waitLiftTarget(floor2);}
//    public Action waitLiftDown() {return new waitLiftTarget(down2);}
//    public Action waitLiftSub() {return new waitLiftTarget(sub2);}
//    public Action waitLiftVertSub() {return new waitLiftTarget(vertSub2);}
//    public Action waitLiftVertFloor() {return new waitLiftTarget(vertFloor2);}
//    public Action waitLiftHighBasket(double waitseconds) {return new waitLiftTarget(highBasket2,waitseconds);}
//    public Action waitLiftRung(double waitseconds) {return new waitLiftTarget(highRung2,waitseconds);}
//    public Action waitLiftRung2(double waitseconds) {return new waitLiftTarget(highRung2_2,waitseconds);}
//    public Action waitLiftRung2_First(double waitseconds) {return new waitLiftTarget(highRung2_First,waitseconds);}
//    public Action waitLiftWall(double waitseconds) {return new waitLiftTarget(wall2,waitseconds);}
//    public Action waitLiftWall2(double waitseconds) {return new waitLiftTarget(wall2_2,waitseconds);}
//    public Action waitLiftWall2_First(double waitseconds) {return new waitLiftTarget(wall2_First,waitseconds);}
//    public Action waitLiftLowBasket(double waitseconds) {return new waitLiftTarget(lowBasket2,waitseconds);} //not tested i think
//    public Action waitLiftFloor(double waitseconds) {return new waitLiftTarget(floor2,waitseconds);}
//    public Action waitLiftDown(double waitseconds) {return new waitLiftTarget(down2,waitseconds);}
//    public Action waitLiftSub(double waitseconds) {return new waitLiftTarget(sub2, waitseconds);}
//    public Action waitLiftVertSub(double waitseconds) {return new waitLiftTarget(vertSub2,waitseconds);}
//    public Action waitLiftVertFloor(double waitseconds) {return new waitLiftTarget(vertFloor2,waitseconds);}
//    public Action waitLiftHighBasket(double waitseconds, double seconds) {return new waitLiftTarget(highBasket2,waitseconds,seconds);}
//    public Action waitLiftRung(double waitseconds, double seconds) {return new waitLiftTarget(highRung2,waitseconds,seconds);}
//    public Action waitLiftRung2(double waitseconds, double seconds) {return new waitLiftTarget(highRung2_2,waitseconds,seconds);}
//    public Action waitLiftRung2_First(double waitseconds, double seconds) {return new waitLiftTarget(highRung2_First,waitseconds,seconds);}
//    public Action waitLiftRung2First(double waitseconds, double seconds) {return new waitLiftTarget(highRung2_2+0.3,waitseconds,seconds);}
//    public Action waitLiftWall(double waitseconds, double seconds) {return new waitLiftTarget(wall2,waitseconds,seconds);}
//    public Action waitLiftWall2(double waitseconds, double seconds) {return new waitLiftTarget(wall2_2,waitseconds,seconds);}
//    public Action waitLiftWall2_First(double waitseconds, double seconds) {return new waitLiftTarget(wall2_First,waitseconds,seconds);}
//    public Action waitLiftLowBasket(double waitseconds, double seconds) {return new waitLiftTarget(lowBasket2,waitseconds,seconds);} //not tested i think
//    public Action waitLiftFloor(double waitseconds, double seconds) {return new waitLiftTarget(floor2,waitseconds,seconds);}
//    public Action waitLiftDown(double waitseconds, double seconds) {return new waitLiftTarget(down2,waitseconds,seconds);}
//    public Action waitLiftSub(double waitseconds, double seconds) {return new waitLiftTarget(sub2, waitseconds, seconds);}
//    public Action waitLiftVertSub(double waitseconds,double seconds) {return new waitLiftTarget(vertSub2,waitseconds,seconds);}
//    public Action waitLiftVertFloor(double waitseconds,double seconds) {return new waitLiftTarget(vertFloor2,waitseconds,seconds);}
//}