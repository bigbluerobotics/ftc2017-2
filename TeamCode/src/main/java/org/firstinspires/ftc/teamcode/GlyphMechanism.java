package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Big Blue Robotics on 10/4/2017.
 */

public class GlyphMechanism {

    private double collectionPower = 0.8;
    private double leftServoUpPos = 0.8;
    private double leftServoDownPos = 0.5;
    private double rightServoUpPos = 0.8;
    private double rightServoDownPos = 0.5;

    public Servo liftLeftServo;
    public Servo liftRightServo;
    public DcMotor leftCollection;
    public DcMotor rightCollection;
    public DcMotor liftMotor;
    public DistanceSensor distanceSensor;


    public GlyphMechanism(HardwareMap hardwareMap){
        leftCollection = hardwareMap.get(DcMotor.class, RobotMap.glyphCollectionLeft);
        rightCollection = hardwareMap.get(DcMotor.class, RobotMap.glyphCollectionRight);

        liftLeftServo = hardwareMap.get(Servo.class, RobotMap.glyphLiftLeftServo);
        liftRightServo = hardwareMap.get(Servo.class, RobotMap.glyphLiftRightServo);

        liftMotor = hardwareMap.get(DcMotor.class, RobotMap.glyphLiftMotor);

        rightCollection.setDirection(DcMotor.Direction.REVERSE);

        distanceSensor = hardwareMap.get(DistanceSensor.class, RobotMap.collectionDistanceSensor);
    }

    public void flipUp(){
        liftRightServo.setPosition(rightServoUpPos);
        liftLeftServo.setPosition(leftServoUpPos);
    }

    public void collect(){
        leftCollection.setPower(collectionPower);
        rightCollection.setPower(collectionPower);
    }

    public boolean blockIsStuck(){
        if(distanceSensor.getDistance(DistanceUnit.INCH) < 4.0){

        }
        return false;
    }

    public void eject(){
        leftCollection.setPower(-collectionPower);
        rightCollection.setPower(-collectionPower);
    }

}
