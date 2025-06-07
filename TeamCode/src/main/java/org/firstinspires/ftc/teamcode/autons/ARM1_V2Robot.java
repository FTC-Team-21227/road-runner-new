package org.firstinspires.ftc.teamcode.autons;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem_Constants;
import org.firstinspires.ftc.teamcode.TunePID;
import org.firstinspires.ftc.teamcode.TunePID_MotionProfile;

public class ARM1_V2Robot {
    private DcMotor arm1;
    //PID controllers for ARM1 and ARM2
    private PIDController controller1;
    //PIDF gains
    double p1 = TunePID.p1, i1 = TunePID.i1, d1 = TunePID.d1;
    double f1 = TunePID.f1;
    //ticks to degrees conversion
    private final double ticks_in_degree_1 = TunePID_MotionProfile.ticks_in_degree_1;
    //length, COM, mass values for feedforward calculation
    private final double L1 = 43.2;
    private final double L2 = 43.2;
    private final double x1 = 36.96;
    private final double x2 = 26.4;
    private final double m1 = 810;
    private final double m2 = 99.79;
    private final double highBasket = Subsystem_Constants.highBasket1;
    private final double highRung = Subsystem_Constants.highRung1;
    private final double highRung2 = Subsystem_Constants.highRung1_2;
    private final double highRung_First = Subsystem_Constants.highRung1_First;
    private final double wall = Subsystem_Constants.wall1;
    private final double wallwall = Subsystem_Constants.wallwall1;
    private final double wall2 = Subsystem_Constants.wall1_2;
    private final double wall_First = Subsystem_Constants.wall1_First;
    private final double lowBasket = Subsystem_Constants.lowBasket1;
    private final double floor = Subsystem_Constants.floor1;
    private final double down = Subsystem_Constants.down1;
    private final double sub = Subsystem_Constants.sub1;
    private final double vertSub1 = Subsystem_Constants.vertSub1;
    private final double vertFloor1 = Subsystem_Constants.vertFloor1;

    public ARM1_V2Robot(HardwareMap hardwareMap) {
        arm1 = hardwareMap.get(DcMotor.class, "ARM1");
        arm1.setDirection(DcMotor.Direction.REVERSE); //CHANGE BACK TO DCMOTORSIMPLE IF SOMETHING DOESN'T WORK
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        controller1 = new PIDController(p1, i1, d1);
    }
    public class LiftTarget implements Action {
        ElapsedTime time = new ElapsedTime();
        boolean start;
        double target1;
        double runTime;
        public LiftTarget(double pos){
            target1 = pos;
            start = false;
            runTime = 2;
        }
        public LiftTarget(double pos, double runtime){
            target1 = pos;
            start = false;
            runTime = runtime;
        }
        public double ARM_Control_PID(@NonNull TelemetryPacket packet /*NEW*/){
            double target2 = PoseStorage.target2; //NEW
            packet.addLine("target2pos1:"+target2); //NEW
            int arm1Pos = arm1.getCurrentPosition();
            double pid1 = controller1.calculate(arm1Pos,(int)(target1*ticks_in_degree_1)); //PID calculation
            double ff1 = (m1*Math.cos(Math.toRadians(target1))*x1 +
            /*NEW*/        m2*Math.cos(Math.atan(((x2*Math.sin(Math.toRadians(target1+target2)))+(L1*Math.sin(Math.toRadians(target1))))/((L1*Math.cos(Math.toRadians(target1)))+(x2*Math.cos(Math.toRadians(target1+target2))))))*
                            Math.sqrt(Math.pow((x2*Math.sin(Math.toRadians(target1+target2))+L1*Math.sin(Math.toRadians(target1))),2)+Math.pow((x2*Math.cos(Math.toRadians(target1+target2))+L1*Math.cos(Math.toRadians(target1))),2))) * f1; // feedforward calculation
            return ((pid1+ff1));
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!start) {
                PoseStorage.target1 = target1; //NEW
                time.reset();
                start = true;
            }
            packet.addLine("time.seconds():"+(time.seconds()));
            packet.addLine("arm1pos:"+(arm1.getCurrentPosition()));
//            packet.addLine("target1pos:"+target1);
            packet.addLine("target1pos:"+(int)(target1*ticks_in_degree_1));
            if (time.seconds() < runTime) {
//                packet.addLine("still running 1");
                double power = ARM_Control_PID(packet /*NEW*/);
                packet.addLine("power1:"+power);
                arm1.setPower(power);
                return true;
            } else {
                arm1.setPower(0);
                return false;
            }
        }
    }
    public Action liftHighBasket() {return new LiftTarget(highBasket);}
    public Action liftRung() {return new LiftTarget(highRung);}
    public Action liftRung2() {return new LiftTarget(highRung2);}
    public Action liftRung_First() { return new LiftTarget(highRung_First);}
    public Action liftWall() {return new LiftTarget(wall);}
    public Action liftWall2() {return new LiftTarget(wall2);}
    public Action liftWall_First() {return new LiftTarget(wall_First);}
    public Action liftLowBasket() {return new LiftTarget(lowBasket);} //not tested i think
    public Action liftFloor() {return new LiftTarget(floor);}
    public Action liftDown() {return new LiftTarget(down);}
    public Action liftSub() {return new LiftTarget(sub);}
    public Action liftVertSub() {return new LiftTarget(vertSub1);}
    public Action liftVertFloor() {return new LiftTarget(vertFloor1);}
    public Action liftHighBasket(double seconds) {return new LiftTarget(highBasket, seconds);}
    public Action liftRung(double seconds) {return new LiftTarget(highRung, seconds);}
    public Action liftRung2(double seconds) {return new LiftTarget(highRung2, seconds);}
    public Action liftRung_First(double seconds) {return new LiftTarget(highRung_First, seconds);}
    public Action liftWall(double seconds) {return new LiftTarget(wall, seconds);}
    public Action liftWall2(double seconds) {return new LiftTarget(wall2, seconds);}
    public Action liftWall_First(double seconds) {return new LiftTarget(wall_First, seconds);}
    public Action liftLowBasket(double seconds) {return new LiftTarget(lowBasket, seconds);} //not tested i think
    public Action liftFloor(double seconds) {return new LiftTarget(floor, seconds);}
    public Action liftDown(double seconds) {return new LiftTarget(down, seconds);}
    public Action liftSub(double seconds) {return new LiftTarget(sub, seconds);}
    public Action liftVertSub(double seconds) {return new LiftTarget(vertSub1, seconds);}
    public Action liftVertFloor(double seconds) {return new LiftTarget(vertFloor1, seconds);}

    public class waitLiftTarget implements Action {
        ElapsedTime time = new ElapsedTime();
        boolean start;
        double target1;
        double waitTime;
        double runTime;
        boolean startMove;
        double power = 1;
        public waitLiftTarget(double pos){
            target1 = pos;
            start = false;
            startMove = false;
            waitTime = 1;
            runTime = 2;
        }
        public waitLiftTarget(double pos, double tim){
            target1 = pos;
            start = false;
            startMove = false;
            waitTime = tim;
            runTime = 2;
        }
        public waitLiftTarget(double pos, double tim, double runtime){
            target1 = pos;
            start = false;
            startMove = false;
            waitTime = tim;
            runTime = runtime;
        }
        public waitLiftTarget(double pos, double tim, double runtime, double pwr){
            target1 = pos;
            start = false;
            startMove = false;
            waitTime = tim;
            runTime = runtime;
            power = pwr;
        }
        public double ARM_Control_PID(@NonNull TelemetryPacket packet /*NEW*/){
            double target2 = PoseStorage.target2; //NEW
            packet.addLine("target2pos1:"+target2); //NEW
            int arm1Pos = arm1.getCurrentPosition();
            double pid1 = controller1.calculate(arm1Pos,(int)(target1*ticks_in_degree_1)); //PID calculation
            double ff1 = (m1*Math.cos(Math.toRadians(target1))*x1 +
            /*NEW*/        m2*Math.cos(Math.atan(((x2*Math.sin(Math.toRadians(target1+target2)))+(L1*Math.sin(Math.toRadians(Math.toRadians(target1)))))/((L1*Math.cos(Math.toRadians(target1)))+(x2*Math.cos(Math.toRadians(target1+target2))))))*
                            Math.sqrt(Math.pow((x2*Math.sin(Math.toRadians(target1+target2))+L1*Math.sin(Math.toRadians(target1))),2)+Math.pow((x2*Math.cos(Math.toRadians(target1+target2))+L1*Math.cos(Math.toRadians(target1))),2))) * f1; // feedforward calculation
            return ((pid1+ff1)) * power;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!start) {
                time.reset();
                start = true;
            }
            packet.addLine("time.seconds():"+(time.seconds()));
            packet.addLine("arm1pos:"+(arm1.getCurrentPosition()));
//            packet.addLine("target1pos:"+target1);
            packet.addLine("target1pos:"+(int)(target1*ticks_in_degree_1));
            if (time.seconds() < runTime+waitTime) {
//                packet.addLine("still running 1");
                if (time.seconds()>waitTime) {
                    //NEW
                    if (!startMove){
                        PoseStorage.target1 = target1;
                        startMove = true;
                    }
                    //
                    double power = ARM_Control_PID(packet /*new*/);
                    packet.addLine("power1:" + power);
                    arm1.setPower(power);
                }
//                else{
//                    arm1.setPower(0);
//                }
                return true;
            } else {
                arm1.setPower(0);
                return false;
            }
        }
    }
    public Action waitLiftHighBasket() {return new waitLiftTarget(highBasket);}
    public Action waitLiftRung() {return new waitLiftTarget(highRung);}
    public Action waitLiftRung2() {return new waitLiftTarget(highRung2);}
    public Action waitLiftRung_First() {return new waitLiftTarget(highRung_First);}
    public Action waitLiftWall() {return new waitLiftTarget(wall);}
    public Action waitLiftWallWall() {return new waitLiftTarget(wallwall);}
    public Action waitLiftWall2() {return new waitLiftTarget(wall2);}
    public Action waitLiftWall_First() {return new waitLiftTarget(wall_First);}
    public Action waitLiftLowBasket() {return new waitLiftTarget(lowBasket);} //not tested i think
    public Action waitLiftFloor() {return new waitLiftTarget(floor);}
    public Action waitLiftDown() {return new waitLiftTarget(down);}
    public Action waitLiftSub() {return new waitLiftTarget(sub);}
    public Action waitLiftVertSub() {return new waitLiftTarget(vertSub1);}
    public Action waitLiftVertFloor() {return new waitLiftTarget(vertFloor1);}
    public Action waitLiftHighBasket(double waitseconds) {return new waitLiftTarget(highBasket, waitseconds);}
    public Action waitLiftRung(double waitseconds) {return new waitLiftTarget(highRung, waitseconds);}
    public Action waitLiftRung2(double waitseconds) {return new waitLiftTarget(highRung2, waitseconds);}
    public Action waitLiftRung_First(double waitseconds) {return new waitLiftTarget(highRung_First, waitseconds);}
    public Action waitLiftRung_FirstSecond(double waitseconds) {return new waitLiftTarget(highRung_First-4, waitseconds);}
    public Action waitLiftWall(double waitseconds) {return new waitLiftTarget(wall, waitseconds);}
    public Action waitLiftWall2(double waitseconds) {return new waitLiftTarget(wall2, waitseconds);}
    public Action waitLiftWall_First(double waitseconds) {return new waitLiftTarget(wall_First, waitseconds);}
    public Action waitLiftLowBasket(double waitseconds) {return new waitLiftTarget(lowBasket, waitseconds);} //not tested i think
    public Action waitLiftFloor(double waitseconds) {return new waitLiftTarget(floor, waitseconds);}
    public Action waitLiftDown(double waitseconds) {return new waitLiftTarget(down, waitseconds);}
    public Action waitLiftSub(double waitseconds) {return new waitLiftTarget(sub, waitseconds);}
    public Action waitLiftVertSub(double waitseconds) {return new waitLiftTarget(vertSub1,waitseconds);}
    public Action waitLiftVertFloor(double waitseconds) {return new waitLiftTarget(vertFloor1,waitseconds);}
    public Action waitLiftWallWall(double waitseconds) {return new waitLiftTarget(wallwall,waitseconds);}
    public Action waitLiftHighBasket(double waitseconds, double seconds) {return new waitLiftTarget(highBasket, waitseconds, seconds);}
    public Action waitLiftRung(double waitseconds, double seconds) {return new waitLiftTarget(highRung, waitseconds, seconds);}
    public Action waitLiftRung2(double waitseconds, double seconds) {return new waitLiftTarget(highRung2, waitseconds, seconds);}
    public Action waitLiftRung_First(double waitseconds, double seconds) {return new waitLiftTarget(highRung_First, waitseconds, seconds);}
    public Action waitLiftWall(double waitseconds, double seconds) {return new waitLiftTarget(wall, waitseconds, seconds);}
    public Action waitLiftWall2(double waitseconds, double seconds) {return new waitLiftTarget(wall2, waitseconds, seconds);}
    public Action waitLiftWall_First(double waitseconds, double seconds) {return new waitLiftTarget(wall_First, waitseconds, seconds);}
    public Action waitLiftLowBasket(double waitseconds, double seconds) {return new waitLiftTarget(lowBasket, waitseconds, seconds);} //not tested i think
    public Action waitLiftFloor(double waitseconds, double seconds) {return new waitLiftTarget(floor, waitseconds, seconds);}
    public Action waitLiftDown(double waitseconds, double seconds) {return new waitLiftTarget(down, waitseconds, seconds);}
    public Action waitLiftSub(double waitseconds, double seconds) {return new waitLiftTarget(sub, waitseconds, seconds);}
    public Action waitLiftVertSub(double waitseconds,double seconds) {return new waitLiftTarget(vertSub1,waitseconds,seconds);}
    public Action waitLiftVertFloor(double waitseconds,double seconds) {return new waitLiftTarget(vertFloor1,waitseconds,seconds);}
    public Action waitLiftFloor(double waitseconds, double seconds, double power) {return new waitLiftTarget(floor, waitseconds, seconds, power);}
    public Action waitLiftSub(double waitseconds, double seconds, double power) {return new waitLiftTarget(sub, waitseconds, seconds, power);}
    public Action waitLiftDown(double waitseconds, double seconds, double power) {return new waitLiftTarget(down, waitseconds, seconds, power);}
    public Action waitLiftVertSub(double waitseconds,double seconds,double power) {return new waitLiftTarget(vertSub1,waitseconds,seconds,power);}
    public Action waitLiftVertFloor(double waitseconds,double seconds,double power) {return new waitLiftTarget(vertFloor1,waitseconds,seconds,power);}
}