package org.firstinspires.ftc.teamcode.autons;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem_Constants;

public class INTAKE_ANGLE_NEW {
    private Servo Intake_Angle;
    final double intake_AngleScale0 = Subsystem_Constants.intake_AngleScale0;
    final double intake_AngleScale1 = Subsystem_Constants.intake_AngleScale1;
    final double intake_AngleFloor = Subsystem_Constants.intake_AngleFloor;
    final double intake_AngleBasket = Subsystem_Constants.intake_AngleBasket;
    final double intake_AngleRung = Subsystem_Constants.intake_AngleRung;
    final double intake_AngleStart = Subsystem_Constants.intake_AngleStart;
    final double intake_AngleWall = Subsystem_Constants.intake_AngleWall;
    final double intake_AngleVertical = Subsystem_Constants.intake_AngleVertical;

    public INTAKE_ANGLE_NEW(HardwareMap hardwareMap) {
        Intake_Angle = hardwareMap.get(Servo.class, "Intake_Angle");
        Intake_Angle.scaleRange(intake_AngleScale0,intake_AngleScale1);
    }

    public class MoveIntakeAngle implements Action {
        ElapsedTime time = new ElapsedTime();
        boolean start;
        double runTime;
        double pos;
        public MoveIntakeAngle(double pos){
            start = false;
            runTime = 0;
            this.pos = pos;
        }
        public MoveIntakeAngle(double pos, double runt){
            start = false;
            runTime = runt;
            this.pos = pos;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!start) {
                time.reset();
                start = true;
            }
            if (time.seconds() < runTime) {
                return true;
            } else {
                Intake_Angle.setPosition(pos);
                return false;
            }
        }
    }
    public Action RotatePosition0_left() {
        return new MoveIntakeAngle(intake_AngleFloor);
    }
    public Action RotatePosition0_left(double runt) {return new MoveIntakeAngle(intake_AngleFloor,runt);}
    public Action RotatePosition0_basket() {
        return new MoveIntakeAngle(intake_AngleBasket);
    }
    public Action RotatePosition0_basket(double runt) {return new MoveIntakeAngle(intake_AngleBasket,runt);}
    public Action RotatePosition0() {
        return new MoveIntakeAngle(intake_AngleRung);
    }
    public Action RotatePosition0(double runt) {return new MoveIntakeAngle(intake_AngleRung, runt);}
    public Action RotatePosition1() {
        return new MoveIntakeAngle(intake_AngleStart);
    }
    public Action RotatePosition1(double runt) {return new MoveIntakeAngle(intake_AngleStart, runt);}
    public Action RotatePositionNegative1() {
        return new MoveIntakeAngle(intake_AngleWall);
    }
    public Action RotatePositionNegative1(double runt) {return new MoveIntakeAngle(intake_AngleWall,runt);}
    public Action RotatePosition2() {
        return new MoveIntakeAngle(intake_AngleVertical);
    }
    public Action RotatePosition2(double runt) {return new MoveIntakeAngle(intake_AngleVertical, runt);}
}