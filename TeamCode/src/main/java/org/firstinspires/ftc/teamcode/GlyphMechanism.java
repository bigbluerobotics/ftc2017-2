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

    private double collectionPower = 1.0;
    private double ejectPower = 0.5;

    private double topGrabClosedPos = 0.49;
    private double topGrabOpenPos = 0.42;
    private double bottomGrabClosedPos = 0.1;
    private double bottomGrabOpenPos = 0.05;

    private double leftServoUpPos = 0.115;
    private double leftServoDownPos = 0.655;
    private double rightServoUpPos = 0.64;
    private double rightServoDownPos = 0.11;

    public Servo liftLeftServo;
    public Servo liftRightServo;
    public Servo grabTopServo;
    public Servo grabBottomServo;
    public DcMotor leftCollection;
    public DcMotor rightCollection;
    public DcMotor liftMotor;
    public DistanceSensor distanceSensor;


    public GlyphMechanism(HardwareMap hardwareMap){
        leftCollection = hardwareMap.get(DcMotor.class, RobotMap.glyphCollectionLeft);
        rightCollection = hardwareMap.get(DcMotor.class, RobotMap.glyphCollectionRight);

        liftLeftServo = hardwareMap.get(Servo.class, RobotMap.glyphLiftLeftServo);
        liftRightServo = hardwareMap.get(Servo.class, RobotMap.glyphLiftRightServo);

        grabTopServo = hardwareMap.get(Servo.class, RobotMap.glyphHoldTop);
        grabBottomServo = hardwareMap.get(Servo.class, RobotMap.glyphHoldBottom);

        liftMotor = hardwareMap.get(DcMotor.class, RobotMap.glyphLiftMotor);

        distanceSensor = hardwareMap.get(DistanceSensor.class, RobotMap.collectionDistanceSensor);

        leftCollection.setDirection(DcMotor.Direction.REVERSE);
    }

    public void grabTop(){
        grabTopServo.setPosition(topGrabClosedPos);
    }

    public void grabBottom(){
        grabBottomServo.setPosition(bottomGrabClosedPos);
    }

    public void releaseTop(){
        grabTopServo.setPosition(topGrabOpenPos);
    }

    public void releaseBottom(){
        grabBottomServo.setPosition(bottomGrabOpenPos);
    }

    public void flipUp(){
        liftRightServo.setPosition(rightServoUpPos);
        liftLeftServo.setPosition(leftServoUpPos);
    }

    public void flipDown(){
        liftRightServo.setPosition((rightServoDownPos));
        liftLeftServo.setPosition((leftServoDownPos));
    }

    public void collect(){
        leftCollection.setPower(collectionPower);
        rightCollection.setPower(collectionPower);
    }

    public boolean hasBlock(){
        if(distanceSensor.getDistance(DistanceUnit.INCH) < 4.0){

        }
        return false;
    }

    public void moveUp(){
        liftMotor.setPower(1.0);
    }

    public void moveDown(){
        liftMotor.setPower(-1.0);
    }

    public void stopRaise(){
        liftMotor.setPower(0);
    }

    public void eject(){
        leftCollection.setPower(-ejectPower);
        rightCollection.setPower(-ejectPower);
    }

    public void stopCollect() {
        leftCollection.setPower(0);
        rightCollection.setPower(0);
    }

}
