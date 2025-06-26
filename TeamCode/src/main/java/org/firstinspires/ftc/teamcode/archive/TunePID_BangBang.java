package org.firstinspires.ftc.teamcode.archive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
//@TeleOp
public class TunePID_BangBang extends OpMode {
    private DcMotorEx ARM1; //bottom arm
    private DcMotorEx ARM2; //top arm
    //PID controllers for ARM1 and ARM2
    private PIDController controller1;
    private PIDController controller2;
    //PIDF gains
    public static double p1 = 0.008, i1 = 0, d1 = 0.0002;
    public static double f1 = 0.000001;
    public static double p2 = 0.006, i2 = 0, d2 = 0.0001;
    public static double f2 = 0.000006;
    //ARM1, ARM2 target positions, in degrees
    public static double target1 = 0;
    public static double target2 = 0;
    //ticks to degrees conversion, very useful
    public final static double ticks_in_degree_1 = 384.539792388 * 28.0/360; // = 29.9086505191 //537.7*28/360; // = 41.8211111111
    public final static double ticks_in_degree_2 = 103.764705882 * 28.0/360; // = 8.07058823527 //145.1*28/360; // = 11.2855555556
    public static final double L1 = 43.2;
    public static final double L2 = 43.2;
    public static final double x1 = 36.96;
    public static final double x2 = 31.48; //26.4; //31.48;
    public static final double m1 = 810;
    public static final double m2 = 404.273; //99.79;
    public static final double ARM1_OFFSET = -17.039;
    public static final double ARM2_OFFSET = 17.443;

    public static final double BANG_BANG_POWER = 1.0;  // Full power
    public static final double PID_ZONE_1 = 100.0;       // Switch to PID within this distance (ticks)
    public static final double PID_ZONE_2 = 100.0;       // Switch to PID within this distance (ticks)
    private boolean arm1InPIDZone = false;              // Tracks control mode
    private boolean arm2InPIDZone = false;
    @Override
    public void init(){
        controller1 = new PIDController(p1,i1,d1);
        controller2 = new PIDController(p2,i2,d2);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        ARM1 = hardwareMap.get(DcMotorEx.class, "ARM1");
        ARM2 = hardwareMap.get(DcMotorEx.class, "ARM2");
        ARM1.setDirection(DcMotorEx.Direction.REVERSE);
        ARM2.setDirection(DcMotorEx.Direction.REVERSE);
    }
    @Override
    public void loop() {
        int target1Ticks = (int)(target1 * ticks_in_degree_1);
        int target2Ticks = (int)(target2 * ticks_in_degree_2);
        int current1 = ARM1.getCurrentPosition();
        int current2 = ARM2.getCurrentPosition();

        // ARM1: Switch between bang-bang and PID
        double error1 = target1Ticks - current1;
        arm1InPIDZone = (Math.abs(error1) < PID_ZONE_1);

        // ARM2: Same logic
        double error2 = target2Ticks - current2;
        arm2InPIDZone = (Math.abs(error2) < PID_ZONE_2);

        double theta1_actual = Math.toRadians(target1 + ARM1_OFFSET);
        double theta2_actual = Math.toRadians(target1 + ARM1_OFFSET + target2 + ARM2_OFFSET);

        // ARM1: PID near target
        if (arm1InPIDZone) {
            double pid1 = controller1.calculate(ARM1.getCurrentPosition(), target1 * ticks_in_degree_1);
            double ff1 = (m1 * x1 * Math.cos(theta1_actual) + m2 * (L1 * Math.cos(theta1_actual) + x2 * Math.cos(theta2_actual))) * f1;
            ARM1.setPower(pid1 + ff1);
        }
        else{
            ARM1.setPower(Math.signum(error1) * BANG_BANG_POWER);
        }

        // ARM2: PID near target
        if (arm2InPIDZone) {
            double pid2 = controller2.calculate(ARM2.getCurrentPosition(), target2 * ticks_in_degree_2);
            double ff2 = m2 * x2 * Math.cos(theta2_actual) * f2;
            ARM2.setPower(pid2 + ff2);
        }
        else{
            ARM2.setPower(Math.signum(error2) * BANG_BANG_POWER);
        }

        telemetry.addData("ARM1 Mode", arm1InPIDZone ? "PID" : "Bang-Bang");
        telemetry.addData("ARM2 Mode", arm2InPIDZone ? "PID" : "Bang-Bang");
        telemetry.addData("ARM1POS",current1);
        telemetry.addData("ARM1Target",(int)(target1*ticks_in_degree_1));
        telemetry.addData("ARM1Power", ARM1.getPower());
        telemetry.addData("ARM1PID", arm1InPIDZone);
        telemetry.addData("ARM2POS",current2);
        telemetry.addData("ARM2Target",(int)(target2*ticks_in_degree_2));
        telemetry.addData("ARM2Power", ARM2.getPower());
        telemetry.addData("ARM2PID", arm2InPIDZone);
        telemetry.update();
    }
}
