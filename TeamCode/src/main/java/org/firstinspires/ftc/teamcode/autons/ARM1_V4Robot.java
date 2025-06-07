package org.firstinspires.ftc.teamcode.autons;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem_Constants;
import org.firstinspires.ftc.teamcode.TunePID_BangBang;
import org.firstinspires.ftc.teamcode.TunePID_MotionProfile;

public class ARM1_V4Robot {
    private DcMotor arm1;
    //PID controllers for ARM1 and ARM2
    private PIDController controller1;
    //PIDF gains
    double p1 = TunePID_MotionProfile.p1, i1 = TunePID_MotionProfile.i1, d1 = TunePID_MotionProfile.d1;
    double f1 = TunePID_MotionProfile.f1;
    //ticks to degrees conversion
    private final double ticks_in_degree_1 = TunePID_MotionProfile.ticks_in_degree_1;
    //length, COM, mass values for feedforward calculation
    private final double L1 = TunePID_MotionProfile.L1;
    private final double L2 = TunePID_MotionProfile.L2;
    private final double x1 = TunePID_MotionProfile.x1;
    private final double x2 = TunePID_MotionProfile.x2;
    private final double m1 = TunePID_MotionProfile.m1;
    private final double m2 = TunePID_MotionProfile.m2;
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
    double ARM1_OFFSET = TunePID_MotionProfile.ARM1_OFFSET;
    double ARM2_OFFSET = TunePID_MotionProfile.ARM2_OFFSET;

    private final double BANG_BANG_POWER = TunePID_BangBang.BANG_BANG_POWER;  // Full power
    private  final double PID_ZONE = TunePID_BangBang.PID_ZONE_1;       // Switch to PID within this distance (ticks)
    private boolean arm1InPIDZone = false;              // Tracks control mode

    public ARM1_V4Robot(HardwareMap hardwareMap) {
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
        double waitTime;
        double runTime;
        boolean startMove;
        double power = 1;
        public LiftTarget(double pos){
            target1 = pos;
            start = false;
            startMove = false;
            waitTime = 0;
            runTime = 2;
        }
        public LiftTarget(double pos, double tim){
            target1 = pos;
            start = false;
            startMove = false;
            waitTime = tim;
            runTime = 2;
        }
        public LiftTarget(double pos, double tim, double runtime){
            target1 = pos;
            start = false;
            startMove = false;
            waitTime = tim;
            runTime = runtime;
        }
        public LiftTarget(double pos, double tim, double runtime, double pwr){
            target1 = pos;
            start = false;
            startMove = false;
            waitTime = tim;
            runTime = runtime;
            power = pwr;
        }

        public double ARM_Control_PID(@NonNull TelemetryPacket packet /*NEW*/){
            double power1;
            double target2 = PoseStorage.target2; //NEW
            packet.addLine("target2pos1:"+target2); //NEW

            int target1Ticks = (int)(target1 * ticks_in_degree_1);
            int current1 = arm1.getCurrentPosition();

            // ARM1: Switch between bang-bang and PID
            double error1 = target1Ticks - current1;
            arm1InPIDZone = (Math.abs(error1) < PID_ZONE);

            double theta1_actual = Math.toRadians(target1 + ARM1_OFFSET);
            double theta2_actual = Math.toRadians(target1 + ARM1_OFFSET + target2 + ARM2_OFFSET);

            // ARM1: PID near target
            if (arm1InPIDZone) {
                double pid1 = controller1.calculate(current1, target1 * ticks_in_degree_1);
                double ff1 = (m1 * x1 * Math.cos(theta1_actual) + m2 * (L1 * Math.cos(theta1_actual) + x2 * Math.cos(theta2_actual))) * f1;
                power1 = (pid1+ff1);
            }
            else{
                power1 = (Math.signum(error1) * BANG_BANG_POWER);
            }

            return power1*power;

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
            if (time.seconds() < runTime+waitTime/* && Math.abs(arm1.getCurrentPosition() - arm1TargetPos) > 5*/) {
//                packet.addLine("still running 1");
                if (time.seconds()>waitTime) {
                    //NEW
                    if (!startMove){
                        PoseStorage.target1 = target1;
                        startMove = true;
                    }
                    double power = ARM_Control_PID(packet /*new*/);
                    packet.addLine("power1:" + power);
                    arm1.setPower(power);
                }
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                return true;
            } else {
                packet.addLine("Arm 1 time:" + time.seconds());
                arm1.setPower(0);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                return false;
            }
        }
    }
    public Action liftHighBasket() {return new LiftTarget(highBasket);}
    public Action liftRung() {return new LiftTarget(highRung);}
    public Action liftRung2() {return new LiftTarget(highRung2);}
    public Action liftRung_First() { return new LiftTarget(highRung_First);}
    public Action liftWall() {return new LiftTarget(wall);}
    public Action waitLiftWallWall() {return new LiftTarget(wallwall);}
    public Action liftWall2() {return new LiftTarget(wall2);}
    public Action liftWall_First() {return new LiftTarget(wall_First);}
    public Action liftLowBasket() {return new LiftTarget(lowBasket);} //not tested i think
    public Action liftFloor() {return new LiftTarget(floor);}
    public Action liftDown() {return new LiftTarget(down);}
    public Action liftSub() {return new LiftTarget(sub);}
    public Action liftVertSub() {return new LiftTarget(vertSub1);}
    public Action liftVertFloor() {return new LiftTarget(vertFloor1);}
    public Action liftHighBasket(double waitseconds) {return new LiftTarget(highBasket, waitseconds);}
    public Action liftRung(double waitseconds) {return new LiftTarget(highRung, waitseconds);}
    public Action liftRung2(double waitseconds) {return new LiftTarget(highRung2, waitseconds);}
    public Action liftRung_First(double waitseconds) {return new LiftTarget(highRung_First, waitseconds);}
    public Action liftRung_FirstSecond(double waitseconds) {return new LiftTarget(highRung_First-4, waitseconds);}
    public Action liftWall(double waitseconds) {return new LiftTarget(wall, waitseconds);}
    public Action waitLiftWallWall(double waitseconds) {return new LiftTarget(wallwall,waitseconds);}
    public Action liftWall2(double waitseconds) {return new LiftTarget(wall2, waitseconds);}
    public Action liftWall_First(double waitseconds) {return new LiftTarget(wall_First, waitseconds);}
    public Action liftLowBasket(double waitseconds) {return new LiftTarget(lowBasket, waitseconds);} //not tested i think
    public Action liftFloor(double waitseconds) {return new LiftTarget(floor, waitseconds);}
    public Action liftDown(double waitseconds) {return new LiftTarget(down, waitseconds);}
    public Action liftSub(double waitseconds) {return new LiftTarget(sub, waitseconds);}
    public Action liftVertSub(double waitseconds) {return new LiftTarget(vertSub1, waitseconds);}
    public Action liftVertFloor(double waitseconds) {return new LiftTarget(vertFloor1, waitseconds);}
    public Action liftHighBasket(double waitseconds, double seconds) {return new LiftTarget(highBasket, waitseconds, seconds);}
    public Action liftRung(double waitseconds, double seconds) {return new LiftTarget(highRung, waitseconds, seconds);}
    public Action liftRung2(double waitseconds, double seconds) {return new LiftTarget(highRung2, waitseconds, seconds);}
    public Action liftRung_First(double waitseconds, double seconds) {return new LiftTarget(highRung_First, waitseconds, seconds);}
    public Action liftWall(double waitseconds, double seconds) {return new LiftTarget(wall, waitseconds, seconds);}
    public Action liftWall2(double waitseconds, double seconds) {return new LiftTarget(wall2, waitseconds, seconds);}
    public Action liftWall_First(double waitseconds, double seconds) {return new LiftTarget(wall_First, waitseconds, seconds);}
    public Action liftLowBasket(double waitseconds, double seconds) {return new LiftTarget(lowBasket, waitseconds, seconds);} //not tested i think
    public Action liftFloor(double waitseconds, double seconds) {return new LiftTarget(floor, waitseconds, seconds);}
    public Action liftDown(double waitseconds, double seconds) {return new LiftTarget(down, waitseconds, seconds);}
    public Action liftSub(double waitseconds, double seconds) {return new LiftTarget(sub, waitseconds, seconds);}
    public Action liftVertSub(double waitseconds,double seconds) {return new LiftTarget(vertSub1,waitseconds,seconds);}
    public Action liftVertFloor(double waitseconds,double seconds) {return new LiftTarget(vertFloor1,waitseconds,seconds);}
    public Action liftFloor(double waitseconds, double seconds, double power) {return new LiftTarget(floor, waitseconds, seconds, power);}
    public Action liftSub(double waitseconds, double seconds, double power) {return new LiftTarget(sub, waitseconds, seconds, power);}
    public Action liftDown(double waitseconds, double seconds, double power) {return new LiftTarget(down, waitseconds, seconds, power);}
    public Action liftVertSub(double waitseconds,double seconds,double power) {return new LiftTarget(vertSub1,waitseconds,seconds,power);}
    public Action liftVertFloor(double waitseconds,double seconds,double power) {return new LiftTarget(vertFloor1,waitseconds,seconds,power);}
}