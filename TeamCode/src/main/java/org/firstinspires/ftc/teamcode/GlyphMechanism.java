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

    public DcMotor leftCollection;
    public DcMotor rightCollection;
    public DistanceSensor distanceSensor;


    public GlyphMechanism(HardwareMap hardwareMap){
        leftCollection = hardwareMap.get(DcMotor.class, "motor_collection_left");
        rightCollection = hardwareMap.get(DcMotor.class, "motor_collection_right");

        rightCollection.setDirection(DcMotor.Direction.REVERSE);

        distanceSensor = hardwareMap.get(DistanceSensor.class, "collection_distance_sensor");
    }

    public void collect(){
        leftCollection.setPower(collectionPower);
        rightCollection.setPower(collectionPower);
    }

    public boolean blockStuck(){
        if(distanceSensor.getDistance(DistanceUnit.INCH) < 4.0){

        }
        return false;
    }

    public void eject(){
        leftCollection.setPower(-collectionPower);
        rightCollection.setPower(-collectionPower);
    }

}
