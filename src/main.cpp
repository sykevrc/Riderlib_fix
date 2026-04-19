#include "main.h"
#include <cmath>
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "robodash/api.h"
const double circ = 7.861; // find this by pushing the chassis forward 60 inches 5 times and average all motor revolution counts.
// the value of circ will be (60*motor_rpm) / (average_rev_counts*wheel_rpm)+7.742
//2846.8
//2824.4
const double calc = 60/(circ*0.75);
// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-2,-19,-12},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({9,20,13}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

pros::Motor intake(-3);
pros::Motor top(10);

pros::Motor out(6);

pros::adi::Pneumatics outtake(1, false);
pros::adi::Pneumatics descore(4, false);
pros::adi::Pneumatics match(3, false);
pros::adi::Pneumatics lift(2, false);

//pros::adi::Pneumatics park(7, false);
pros::ADIDigitalIn button (8);
// Inertial Sensor on port 11
pros::Imu imu(11);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
//pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
//pros::Rotation verticalEnc(3);
// distance sensor, right side on port 12
pros::Distance rightdist(7);
pros::Distance leftdist(1);

pros::Optical colorsens(8);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
//lemlib::TrackingWheel horizontal(&horizontalEnc, 2, -5.75);
// vertical tracking wheel. 2" diameter, .5" offset, right of the robot (negative)
//lemlib::TrackingWheel vertical(&verticalEnc, 2.1, .5);
// use distance sensor in the drivetrain
lemlib::DistanceSensor right(&rightdist, 9.25);
lemlib::DistanceSensor left(&leftdist, 5);
// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              14.25, // 14 inch track width
                              3.25, // found using empirical testing
                              450, // drivetrain rpm is 450
                              8 // horizontal drift is 8. Since we had traction wheels, it is 8
);


// lateral motion controller
lemlib::ControllerSettings linearController(4, // proportional gain (kP)
                                            0.25, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            7 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(1.3, // proportional gain (kP)
                                             0.3, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &right, // right side distance
                            &left, // left side distance
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);
double front_reset(double a, double b, bool dir){
    double theta = chassis.getPose(true).theta-M_PI/2;
    if(dir){
        return 70.5-fabs((leftdist.get_distance()/25.4-a*tan(theta)+b)*cos(theta))
        ;
    }else{
        return -70.5+fabs((leftdist.get_distance()/25.4-a*tan(theta)+b)*cos(theta));
    }
}
double right_reset(double a, double b, bool dir){
    double theta = chassis.getPose(true).theta-M_PI/2;
    if(dir){
        return 70.5-fabs((rightdist.get_distance()/25.4-a*tan(theta)+b)*cos(theta));
    }else{
        return -70.5+fabs((rightdist.get_distance()/25.4-a*tan(theta)+b)*cos(theta));
    }
}
void run_intake(){
    intake.move_voltage(13000);
    top.move_voltage(5000);
    out.move_voltage(5000);
    if(out.get_power()>5)out.move_voltage(1000);

    outtake.retract();
    lift.retract();
}
void wiggle(){
    int speed = 2500;
    int forwards_scale = 2500;
    leftMotors.move_voltage(speed+forwards_scale);
    rightMotors.move_voltage(-speed);
    pros::delay(200);
    rightMotors.move_voltage(speed+forwards_scale);
    leftMotors.move_voltage(-speed);
    pros::delay(200);
    
}
void outtakefast(){
    intake.move_voltage(-9000);
    top.move_voltage(-13000);
    out.move_relative(-100,200);
    lift.extend();
}
void outtake_no_lift(){
    intake.move_voltage(-13000);
    top.move_voltage(-13000);
    out.move_relative(-100,200);
    //lift.extend();
}
void outtakeslow(){
    intake.move_voltage(-5000);
    top.move_voltage(-13000);
    out.move_relative(-100,200);
    lift.extend();
}
void scoretop(){
    intake.move_voltage(13000);
    top.move_voltage(13000);
    out.move_voltage(13000);
    outtake.extend();
    //descore.retract();
    lift.retract();
}
void scorebottom(){
    intake.move_voltage(13000);
    top.move_voltage(6000);
    out.move_voltage(-13000);

    //outtake.retract();  
}
void scorebottomslow(){
    intake.move_voltage(13000);
    top.move_voltage(13000);
    out.move_voltage(-5000);
}
void scorebottomrllyslow(){
    intake.move_voltage(13000);
    top.move_voltage(8000);
    out.move_voltage(-2000);
}
void scoot(){
    intake.move_voltage(13000);
    top.move_voltage(0);
    ////out.move_voltage(10000);
}
void stop(){
    intake.move_voltage(0);
    top.move_voltage(0);
    out.move_voltage(0);

}
void reclaim(){
    top.move_voltage(-13000);
    out.move_voltage(13000);
}
void push(bool side){
    if(side){//left
    match.retract();
    chassis.moveToPoint(-44,chassis.getPose().y,400);
    
    chassis.turnToHeading(135,500);
    chassis.moveToPoint(-29,34,600);

    chassis.turnToHeading(90,500);

    chassis.moveToPoint(-6,35,1200,{.minSpeed=43});
    chassis.setBrakeMode(MOTOR_BRAKE_HOLD);
    match.retract();
    chassis.turnToHeading(140,500,{.minSpeed=40});
    }else{
    match.retract();
    chassis.moveToPoint(-44,chassis.getPose().y,400);
    
    chassis.turnToHeading(-135,500);
    chassis.moveToPoint(-25,-35,600,{.forwards=false});

    chassis.turnToHeading(-90,500);

    chassis.moveToPoint(-6,-36,1000,{.forwards=false,.minSpeed=40});
    chassis.setBrakeMode(MOTOR_BRAKE_HOLD);
    match.retract();
    chassis.turnToHeading(-140,500,{.minSpeed=40});
    }
}
void redloaderquick(bool color){
    chassis.moveToPoint(-95,chassis.getPose().y,500,{.maxSpeed=45,.minSpeed=40});
    chassis.moveToPoint(-95,chassis.getPose().y,500,{.maxSpeed=24,.minSpeed=24});
    
    //pros::delay(100);
    //chassis.moveToPoint(chassis.getPose().x+.5,chassis.getPose().y,300,{.forwards=false,.minSpeed=30});
    //chassis.moveToPoint(chassis.getPose().x-15,chassis.getPose().y,1000,{.minSpeed=30});

}
void redloaderskillsclose(){
    chassis.moveToPoint(-750,48,800,{.maxSpeed=50,.minSpeed=50});
    run_intake();
    chassis.waitUntil(20);
    chassis.cancelMotion();
    wiggle();
    leftMotors.move_voltage(0);
    rightMotors.move_voltage(0);
    chassis.moveToPoint(-750,chassis.getPose().y,1000,{.maxSpeed=35,.minSpeed=35});
    
    //chassis.moveToPoint(chassis.getPose().x+3,chassis.getPose().y,500);
    //chassis.moveToPoint(-75,chassis.getPose().y,500,{.maxSpeed=45,.minSpeed=40});
    //pros::delay(500);
    
}
void redloaderskillsfar(){
    chassis.moveToPoint(750,chassis.getPose().y,800,{.maxSpeed=50,.minSpeed=50});
    run_intake();
    chassis.waitUntil(20);
    chassis.cancelMotion();
    wiggle();
    leftMotors.move_voltage(0);
    rightMotors.move_voltage(0);
    chassis.moveToPoint(750,chassis.getPose().y,1000,{.maxSpeed=35,.minSpeed=35});
    
    //chassis.moveToPoint(chassis.getPose().x-3,chassis.getPose().y,500);
    //chassis.moveToPoint(75,chassis.getPose().y,500,{.maxSpeed=45,.minSpeed=40});
}
void bluezoneclear(){
    chassis.setPose(0,0,-90);
    run_intake();
    //lift.extend();
    chassis.moveToPoint(-150,0,600,{.maxSpeed=50,.minSpeed=50});
    
    chassis.waitUntilDone();
    chassis.moveToPoint(chassis.getPose().x+5, chassis.getPose().y,400,{.forwards=false,.maxSpeed=30,.minSpeed=30});
    chassis.waitUntilDone();
    pros::delay(700);

    chassis.moveToPoint(chassis.getPose().x-20, chassis.getPose().y,1200,{.maxSpeed=50,.minSpeed=50});
    //chassis.moveToPoint(chassis.getPose().x+7, chassis.getPose().y,400,{.forwards=false,.maxSpeed=30,.minSpeed=30});
    chassis.moveToPoint(chassis.getPose().x+2, chassis.getPose().y,400,{.forwards=false,.maxSpeed=30,.minSpeed=30});

    //chassis.waitUntilDone();
    // rightMotors.move_voltage(7000);
    // pros::delay(200);
    // for(int i = 0; i < 3; i++){
    // wiggle();
    // }
    //leftMotors.move_voltage(0);
    //rightMotors.move_voltage(0);
    chassis.moveToPoint(chassis.getPose().x+32, chassis.getPose().y,1000,{.forwards=false,.maxSpeed=60});
    chassis.turnToHeading(-90,700);
    chassis.waitUntilDone();
    pros::delay(10);
    chassis.setPose(front_reset(3.5,5.25,false),right_reset(0.5,3.5,true),chassis.getPose().theta);
    pros::delay(10);

}
void bluezoneclearside(){
    chassis.setPose(-63,18,-180);
    run_intake();
    chassis.moveToPoint(-65,-80,700,{.maxSpeed=79,.minSpeed=79});
    
    chassis.moveToPoint(-65,-60,1000,{.maxSpeed=60,.minSpeed=60});
    chassis.waitUntil(8);
    match.extend();
    chassis.waitUntilDone();
    chassis.setPose(-70.5+(rightdist.get_distance()/25.4+3.5),chassis.getPose().y,chassis.getPose().theta);
    
    chassis.moveToPoint(-61.5,-20,400,{.forwards=false,.minSpeed=20});
    chassis.moveToPoint(-61.5,-10,400,{.forwards=false,.maxSpeed=45});
    chassis.waitUntilDone();
    chassis.setPose(-70.5+(rightdist.get_distance()/25.4+3.5),-13.5,-180);
    pros::delay(50);

    chassis.moveToPoint(chassis.getPose().x+2,-20,800,{.maxSpeed=40});
    match.retract();
    
    chassis.swingToHeading(80,DriveSide::LEFT, 700);
    match.retract();
}
void redzoneclear(){
    chassis.moveToPoint(36,48,400,{.minSpeed=20});
    chassis.turnToHeading(180,800);

    chassis.moveToPoint(39,1,1500);
    run_intake();
    chassis.turnToHeading(90,800);
    chassis.moveToPoint(75,0,1200,{ .maxSpeed=75,.minSpeed=75});
    
    chassis.moveToPoint(chassis.getPose().x-5, chassis.getPose().y,400,{.forwards=false,.maxSpeed=30,.minSpeed=30});
    chassis.waitUntilDone();
    pros::delay(700);

    chassis.moveToPoint(chassis.getPose().x+20, chassis.getPose().y,800,{.maxSpeed=60,.minSpeed=30});
    //chassis.moveToPoint(chassis.getPose().x+7, chassis.getPose().y,400,{.forwards=false,.maxSpeed=30,.minSpeed=30});
    chassis.moveToPoint(chassis.getPose().x-2, chassis.getPose().y,400,{.forwards=false,.maxSpeed=30,.minSpeed=30});

    chassis.waitUntilDone();
    for(int i = 0; i < 3; i++){
    wiggle();
    }
    leftMotors.move_voltage(0);
    rightMotors.move_voltage(0);
    chassis.moveToPoint(chassis.getPose().x-32, chassis.getPose().y,1000,{.forwards=false});
    chassis.turnToHeading(90,500);
    chassis.waitUntilDone();
    chassis.setPose(70.5-(leftdist.get_distance()/25.4+5.92),-70.5+(rightdist.get_distance()/25.4+3.5),chassis.getPose().theta);
}
void skillsv2() {
   
    //left red
    chassis.setPose(-70.5+(leftdist.get_distance()/25.4+4.5),14,0); 
    chassis.moveToPose(chassis.getPose().x, 48,-90,3000,{.earlyExitRange=2});
    outtake.extend();
    run_intake();
    match.toggle();
    chassis.waitUntilDone();
    redloaderskillsclose();

    
    // chassis.moveToPoint(-24,48,1000,{.forwards=false,.maxSpeed=50});
    // chassis.waitUntilDone();
    // chassis.setPose(-28.75,48,chassis.getPose().theta);
    // scoretop();
    // pros::delay(1000);
    // outtake();
    // pros::delay(200);
    // scoretop();
    // pros::delay(2000);
    // match.retract();
   
    chassis.moveToPoint(-44,48,700,{.forwards=false});
    top.move_relative(600,600);
    //cross
    chassis.turnToHeading(-45,400);
    match.retract();
    run_intake();
    chassis.moveToPoint(-24,30,1000,{.forwards=false,.minSpeed=20,.earlyExitRange=4});//switch to far
    chassis.turnToHeading(-90,400);

    chassis.moveToPoint(30,30,1000,{.forwards=false,.minSpeed=20,.earlyExitRange=4});
    chassis.moveToPoint(40,48,1200,{.forwards=false,.minSpeed=20});
    chassis.turnToHeading(90,1000,{.maxSpeed=60});
    chassis.waitUntilDone();
    chassis.moveToPoint(22,49,700,{.forwards=false,.maxSpeed=50});
    chassis.waitUntilDone();
    chassis.setPose(28.75,48,chassis.getPose().theta);
    scoretop();
    pros::delay(1000);
    //outtake();
    pros::delay(500);
    scoretop();
    pros::delay(2000);
    chassis.moveToPoint(55,47,1000);
    run_intake();
    match.extend();
    chassis.turnToHeading(90,300);
    chassis.waitUntilDone();
    redloaderskillsfar();   
    chassis.moveToPoint(20,48,1000,{.forwards=false,.maxSpeed=50});
    chassis.waitUntilDone();
    chassis.setPose(28.75,48,chassis.getPose().theta);

    scoretop();
    pros::delay(1000);
    //outtake();
    pros::delay(500);
    scoretop();
    pros::delay(2000);

    chassis.moveToPoint(33,48,500);
    match.retract();
    
    chassis.moveToPose(43,-24,180,3000,{.minSpeed=20,.earlyExitRange=4});
    run_intake();
    //far right 
    chassis.waitUntilDone();
    chassis.setPose(70.5-(leftdist.get_distance()/25.4+4),chassis.getPose().y,chassis.getPose().theta);
    pros::delay(50);
    chassis.moveToPoint(37,-48,700);
    chassis.turnToHeading(90,1000,{.maxSpeed=40});
    match.extend();
    chassis.waitUntilDone();
    chassis.setPose(chassis.getPose().x, -70.5+(rightdist.get_distance()/25.4+4),chassis.getPose().theta);
    chassis.waitUntilDone();
    chassis.moveToPoint(54,-48,700);
    chassis.turnToHeading(90,500);
    chassis.waitUntilDone();
    redloaderskillsfar();

    
    

    chassis.moveToPoint(44,-48,800,{.forwards=false});
    top.move_relative(600,600);
    match.retract();
    //right
    chassis.turnToHeading(135,800);
    run_intake();
    chassis.moveToPoint(24,-30,1000,{.forwards=false,.minSpeed=20});
    chassis.turnToHeading(90,400);
    chassis.moveToPoint(-30,-30,1000,{.forwards=false,.minSpeed=20});
    chassis.moveToPoint(-40,-49,1200,{.forwards=false,.minSpeed=20});
    chassis.turnToHeading(-90,1000,{.maxSpeed=60});
    chassis.waitUntilDone();
    chassis.moveToPoint(-22,-48,1000,{.forwards=false,.maxSpeed=50});
    chassis.waitUntilDone();
    chassis.setPose(-28.75,-48,chassis.getPose().theta);
    scoretop();
    pros::delay(1000);
    //outtake();
    pros::delay(500);
    scoretop();
    pros::delay(2000);
    chassis.moveToPoint(-55,-46.5,1000);
    run_intake();
    match.extend();
    chassis.turnToHeading(-90,400);
    chassis.waitUntilDone();
    redloaderskillsclose();   
    chassis.moveToPoint(-20,-48,1000,{.forwards=false,.maxSpeed=50});
    chassis.waitUntilDone();
    chassis.setPose(-28.75,-48,chassis.getPose().theta);

    scoretop();
    pros::delay(1000);
    //outtake();
    pros::delay(200);
    scoretop();
    pros::delay(2000);

    chassis.moveToPoint(-37,-48,800);
    // chassis.turnToHeading(-45,600);

    // chassis.moveToPoint(-62,-24,1000);
     match.toggle();
    // chassis.turnToHeading(0,1000);
    // chassis.waitUntilDone();
    // chassis.setPose(-70.5+(leftdist.get_distance()/25.4+3.5),chassis.getPose().y, chassis.getPose().theta);
    chassis.moveToPose(-62,-20,0,1000);
    chassis.moveToPoint(-68, 0, 2000, {.minSpeed=100});
    chassis.moveToPoint(-68, -1, 1000, {.forwards=false,.minSpeed=70});

}
void sev_l(){
    chassis.setPose(-49.5,16,90);
    chassis.moveToPoint(-24,24,800,{.minSpeed=10});
    run_intake();
    chassis.waitUntil(16);
    match.extend();
    chassis.moveToPoint(-48,49,900,{.forwards=false,.minSpeed=30});
    chassis.swingToHeading(-90,DriveSide::LEFT,400,{.minSpeed=30});
    chassis.moveToPoint(-10,49,1000,{.forwards=false,.maxSpeed=80});
    chassis.waitUntil(9);
    scoretop();
    chassis.swingToHeading(-90,DriveSide::LEFT,800);
    chassis.waitUntilDone();
    chassis.setPose(-28.75,48,chassis.getPose().theta);
    chassis.moveToPoint(-70,47.5,400,{.minSpeed=40});
    out.move_voltage(0);
    top.move_voltage(0);
    outtake.retract();
    redloaderquick(true);
    chassis.waitUntilDone();
    chassis.setPose(front_reset(3.5,5.25,false),right_reset(0.5,3.5,true),chassis.getPose().theta);
    chassis.moveToPoint(-40,chassis.getPose().y,300,{.forwards=false});
    match.retract();
    chassis.turnToHeading(-35,300);
    chassis.moveToPoint(-8,9,1200,{.forwards=false,.minSpeed=30});
    chassis.waitUntil(26);
    scorebottomslow();
    chassis.turnToHeading(-45,300);
    chassis.waitUntilDone();   
    pros::delay(1000);
}
void sev_r(){
    chassis.setPose(-49.5,-19.75,90);
    chassis.moveToPoint(-24,-24,800,{.minSpeed=10});
    run_intake();
    chassis.waitUntil(16);
    match.extend();
    chassis.moveToPoint(-48,-49,750,{.forwards=false,.minSpeed=30});
    chassis.swingToHeading(-75,DriveSide::RIGHT,400,{.minSpeed=30});
    chassis.moveToPoint(-10,-48,1000,{.forwards=false,.maxSpeed=80});
    chassis.waitUntil(9);
    scoretop();
    chassis.swingToHeading(-90,DriveSide::RIGHT,800);
    chassis.waitUntilDone();
    chassis.setPose(-28.75,-48,chassis.getPose().theta);
    chassis.moveToPoint(-70,-47.5,400,{.minSpeed=40});
    out.move_voltage(0);
    top.move_voltage(0);
    outtake.retract();
    redloaderquick(true);
    chassis.waitUntilDone();
    chassis.setPose(front_reset(3.5,5.25,false),-48,chassis.getPose().theta);
    chassis.moveToPoint(-40,chassis.getPose().y,300,{.forwards=false});
    match.retract();
    chassis.turnToHeading(35,300);
    chassis.moveToPoint(-13,-12,1300,{.minSpeed=30});
    chassis.waitUntil(26);
    outtakefast(); 
    chassis.turnToHeading(45,300);
    chassis.waitUntilDone();   
    pros::delay(1000);
}
void four_l(){
    chassis.setPose(-49.5,16,90);
    chassis.moveToPoint(-24,24,800,{.minSpeed=10});
    run_intake();
    chassis.waitUntil(16);
    match.extend();
    chassis.moveToPoint(-48,49,900,{.forwards=false,.minSpeed=30});
    chassis.swingToHeading(-90,DriveSide::LEFT,400,{.minSpeed=30});
    chassis.moveToPoint(-10,49,1000,{.forwards=false,.maxSpeed=80});
    chassis.waitUntil(9);
    scoretop();
    chassis.swingToHeading(-90,DriveSide::LEFT,800);
    chassis.waitUntilDone();
    chassis.setPose(-28.75,48,chassis.getPose().theta);

    descore.extend();
    chassis.moveToPoint(-37,chassis.getPose().y,500,{.minSpeed=40});
    chassis.swingToHeading(135,DriveSide::LEFT,600,{.minSpeed=40});
    descore.retract();
    chassis.moveToPoint(-20,33,500,{.minSpeed=40});
    chassis.turnToHeading(90,200,{.minSpeed=40});
    chassis.moveToPoint(-9,33,800,{.minSpeed=40});
    chassis.setBrakeMode(MOTOR_BRAKE_HOLD);
    match.retract();
    chassis.turnToHeading(140,500,{.minSpeed=40});
}
void four_r(){
    chassis.setPose(-49.5,-19.75,90);
    chassis.moveToPoint(-26,-24,900,{.minSpeed=10});
    run_intake();
    chassis.waitUntil(16);
    match.extend();
    chassis.moveToPoint(-48,-48,750,{.forwards=false,.minSpeed=30});
    chassis.swingToHeading(-75,DriveSide::RIGHT,400,{.minSpeed=30});
    chassis.moveToPoint(-10,-48,1000,{.forwards=false,.maxSpeed=80});
    chassis.waitUntil(9);
    scoretop();
    chassis.swingToHeading(-90,DriveSide::RIGHT,600);
    chassis.waitUntilDone();
    chassis.setPose(-28.75,-48,chassis.getPose().theta);
    descore.extend();
    chassis.moveToPoint(-37,chassis.getPose().y,500,{.minSpeed=40});
    
    chassis.turnToHeading(-135,200,{.minSpeed=40});
    descore.retract();

    chassis.moveToPoint(-25,-35,500,{.forwards=false,.minSpeed=40});
    chassis.turnToHeading(-90,200,{.minSpeed=40});

    chassis.moveToPoint(-9,-35,800,{.forwards=false,.minSpeed=40});
    chassis.setBrakeMode(MOTOR_BRAKE_HOLD);
    match.retract();
    chassis.turnToHeading(-140,500,{.minSpeed=40});

}
void sawp(){
    chassis.setPose(-70.5+(rightdist.get_distance()/25.4+3.5),-13.5,180); 
    chassis.moveToPoint(chassis.getPose().x, -47.5,700,{.minSpeed=20});
    //outtake.extend();
    run_intake();
    
    chassis.turnToHeading(-90,400);
    
    match.toggle();
    chassis.moveToPoint(-100,chassis.getPose().y,50);
    redloaderquick(true);

    chassis.moveToPoint(-20,chassis.getPose().y,1200,{.forwards=false,.minSpeed=40});
    chassis.waitUntil(16);
    scoretop();
    chassis.waitUntilDone();
    pros::delay(400);
    match.retract();
    chassis.setPose(-28.75,-48,chassis.getPose().theta);
    chassis.turnToHeading(0,1200,{.minSpeed=70});
    chassis.waitUntilDone();
   
    chassis.setPose(-27,-39,chassis.getPose().theta);
    run_intake();
    chassis.moveToPoint(-25,40,800,{.minSpeed=40,.earlyExitRange=4});
    chassis.moveToPoint(-50,45.5,800,{.minSpeed=40,.earlyExitRange=4});
    chassis.turnToHeading(-90,600,{.minSpeed=30});
    chassis.waitUntilDone();
    chassis.setPose(chassis.getPose().x,right_reset(0.5,3.5,true),chassis.getPose().theta);
    chassis.moveToPoint(-10,47,1200,{.forwards=false});
    chassis.waitUntil(6);
    
    scoretop();
    chassis.waitUntilDone();
    if(chassis.getPose().theta<270){
    leftMotors.move_voltage(-10000);
    rightMotors.move_voltage(-5000);
    }else{
    leftMotors.move_voltage(-5000);
    rightMotors.move_voltage(-10000);
    }
    
    pros::delay(700);
    match.extend();
    chassis.setPose(-28.75,48,chassis.getPose().theta);
    //chassis.setPose(-28.75,48,-90);
    chassis.moveToPoint(-70,47.5,400,{.minSpeed=40});
    out.move_voltage(0);
    top.move_voltage(0);
    outtake.retract();
    redloaderquick(true);
    chassis.waitUntilDone();
    chassis.setPose(front_reset(3.5,5.25,false),right_reset(0.5,3.5,true),chassis.getPose().theta);
    chassis.moveToPoint(-40,chassis.getPose().y,300,{.forwards=false});
    chassis.turnToHeading(-45,300);
    chassis.moveToPoint(-6,7,1200,{.forwards=false,.earlyExitRange=4});
    match.retract();
    chassis.waitUntil(10);
    top.move_voltage(13000);
    chassis.waitUntil(26);
    scorebottomslow(); 
    chassis.waitUntilDone();   
    chassis.setPose(-8,8,chassis.getPose().theta);
    pros::delay(500);
    chassis.moveToPoint(-24,29,700,{.minSpeed=40});
    chassis.swingToHeading(80,DriveSide::RIGHT,300,{.minSpeed=40});
    chassis.moveToPoint(-5,37,500,{.minSpeed=40});
    chassis.swingToHeading(135,DriveSide::RIGHT,300,{.minSpeed=40});

}
void dump_r(){
    chassis.setPose(-49.5,-19.75,90);
    chassis.moveToPoint(-24,-24,800,{.minSpeed=10});
    run_intake();
    chassis.waitUntil(16);
    match.extend();
    chassis.moveToPoint(-48,-49,750,{.forwards=false,.minSpeed=30});
    chassis.swingToHeading(-75,DriveSide::RIGHT,400,{.minSpeed=30});
    chassis.moveToPoint(-10,-48.5,1000,{.forwards=false,.maxSpeed=80});
    chassis.waitUntil(9);
    scoretop();
    chassis.swingToHeading(-90,DriveSide::RIGHT,800,{.minSpeed=40});
    chassis.waitUntilDone();
    chassis.setPose(-28.75,-48,chassis.getPose().theta);
    chassis.moveToPoint(-70,-47.5,400,{.minSpeed=40});
    out.move_voltage(0);
    top.move_voltage(0);
    //outtake.retract();
    redloaderquick(true);
    chassis.moveToPoint(-10,-48,1000,{.forwards=false,.minSpeed=40});
    chassis.waitUntil(16);
    scoretop();
    chassis.waitUntilDone();
    pros::delay(700);
    chassis.moveToPoint(chassis.getPose().x-5,chassis.getPose().y,400,{.minSpeed=40});
    chassis.moveToPoint(chassis.getPose().x+15,chassis.getPose().y,400,{.forwards=false,.minSpeed=40});
    outtake.retract();
}
void dump_l(){
    chassis.setPose(-49.5,16,90);
    chassis.moveToPoint(-24,24,800,{.minSpeed=10});
    run_intake();
    chassis.waitUntil(16);
    match.extend();
    chassis.moveToPoint(-48,49,900,{.forwards=false,.minSpeed=30});
    chassis.swingToHeading(-90,DriveSide::LEFT,400,{.minSpeed=30});
    chassis.moveToPoint(-10,49,1000,{.forwards=false,.maxSpeed=80});
    chassis.waitUntil(9);
    scoretop();
    chassis.swingToHeading(-90,DriveSide::LEFT,800);
    chassis.waitUntilDone();
    chassis.setPose(-28.75,48,chassis.getPose().theta);
    chassis.moveToPoint(-70,47.5,400,{.minSpeed=40});
    redloaderquick(true);
    chassis.moveToPoint(-10,48,1000,{.forwards=false,.minSpeed=40});
    chassis.waitUntil(16);
    scoretop();
    chassis.waitUntilDone();
    pros::delay(700);
    chassis.moveToPoint(chassis.getPose().x-15,chassis.getPose().y,400,{.minSpeed=40});
    chassis.moveToPoint(chassis.getPose().x+15,chassis.getPose().y,400,{.forwards=false,.minSpeed=40});
    outtake.retract(); 
}
void far(){
    chassis.setPose(28.75,48,90);
    chassis.moveToPose(63.5,0,180,1000);
    match.retract();

    chassis.moveToPoint(63.5,-40,1500,{.maxSpeed=74,.minSpeed=74});
    run_intake();
    chassis.waitUntil(33);
    match.extend();
    chassis.moveToPoint(64,-80,900,{.maxSpeed=55,.minSpeed=33});
    
    chassis.moveToPoint(64,-10,400,{.forwards=false,.minSpeed=30});
    chassis.moveToPoint(64,-10,800,{.forwards=false,.maxSpeed=25});
    chassis.waitUntilDone();
    chassis.setPose(70.5-(leftdist.get_distance()/25.4+3.5),-14.5,chassis.getPose().theta);
    pros::delay(50);

    chassis.moveToPoint(chassis.getPose().x-1,-20,1000,{.maxSpeed=40});
    match.retract();

    chassis.swingToHeading(-80,lemlib::DriveSide::RIGHT,800);
    chassis.moveToPoint(24,-24,1200);
    chassis.waitUntil(12);
    match.extend();
    chassis.turnToHeading(135,600);
    chassis.moveToPoint(6,-5,1200,{.forwards=false,.maxSpeed=50,.minSpeed=30});
    chassis.waitUntil(20);
    top.move_voltage(-13000);
    out.move_voltage(-13000);
    intake.move_relative(-200,600);
    chassis.waitUntilDone();
    pros::delay(500);
    scorebottomslow();
    pros::delay(500);
    scorebottomrllyslow();
    while(colorsens.get_hue()<200||colorsens.get_hue()>300){
        pros::delay(10);
    }
    out.move_voltage(10000);
    top.move_voltage(-10000);
    chassis.setPose(9,-9,chassis.getPose().theta);


    //last half-
    chassis.moveToPoint(46,-49,1300);
}
void nine_r(){
    chassis.setPose(-49.5,-19.75,90);
    chassis.moveToPoint(-26,-24,400,{.minSpeed=30});
    run_intake();
    chassis.moveToPoint(-11,-30,700,{.minSpeed=30});
    chassis.swingToHeading(170,DriveSide::RIGHT,400,{.minSpeed=30});
    chassis.moveToPoint(-8,-42,600,{.minSpeed=30});
    chassis.moveToPoint(-36,-30,800,{.forwards=false,.minSpeed=30});
    chassis.moveToPoint(-46,-45,1200,{.forwards=false,.minSpeed=30});
    chassis.turnToHeading(-90,600,{.maxSpeed=60});
    match.extend();
    chassis.moveToPoint(-70,chassis.getPose().y,600,{.minSpeed=40});
    //redloaderquick(true);
    chassis.waitUntilDone();
    chassis.setPose(-55.25,-47,chassis.getPose().theta);
    chassis.moveToPoint(-20, chassis.getPose().y,200,{.forwards=false,.minSpeed=40});
    chassis.moveToPose(-24,-36,-90,700,{.forwards=false,.minSpeed=40});
    match.retract();
    descore.extend();
    chassis.turnToHeading(-90,300);
    chassis.moveToPoint(-5,chassis.getPose().y,1000,{.forwards=false,.minSpeed=40});
    chassis.moveToPoint(-51,chassis.getPose().y,800,{.minSpeed=40});
    descore.retract();
    chassis.turnToHeading(-50,300);
    chassis.moveToPose(-10,-47,-90,800,{.forwards=false,.minSpeed=40});
    chassis.waitUntil(16);
    scoretop();
    chassis.moveToPoint(-10,-48,500,{.forwards=false});
    chassis.waitUntilDone();
    outtake.retract();
    out.move_voltage(-100);
    top.move_voltage(-10000);
    
    chassis.setPose(-28.75,-48,chassis.getPose().theta);
    
    chassis.moveToPoint(-40,chassis.getPose().y,300);
    chassis.turnToHeading(45,300);
    chassis.moveToPoint(-13,-13,1300);
    chassis.waitUntil(26);
    outtakefast(); 
    chassis.turnToHeading(45,300);
    chassis.waitUntilDone();   
    pros::delay(10000);
}
void nine_l(){
    chassis.setPose(-49.5,16,90);
    chassis.moveToPoint(-26,24,400,{.minSpeed=30});
    run_intake();
    chassis.moveToPoint(-11,30,700,{.minSpeed=30});
    chassis.swingToHeading(10,DriveSide::RIGHT,400,{.minSpeed=30});
    chassis.moveToPoint(-8,42,600,{.minSpeed=30});
    chassis.moveToPoint(-36,30,800,{.forwards=false,.minSpeed=30});
    chassis.moveToPoint(-46,45,1200,{.forwards=false,.minSpeed=30});
    chassis.turnToHeading(-90,600,{.maxSpeed=60});
    match.extend();
    chassis.moveToPoint(-70,chassis.getPose().y,600,{.minSpeed=40});
    //redloaderquick(true);
    chassis.waitUntilDone();
    chassis.setPose(-55.25,47,chassis.getPose().theta);
    chassis.moveToPoint(-20, chassis.getPose().y,200,{.forwards=false,.minSpeed=40});
    chassis.moveToPose(-24,36,-90,700,{.forwards=false,.minSpeed=40});
    match.retract();
    descore.extend();
    chassis.turnToHeading(-90,300);
    chassis.moveToPoint(-5,chassis.getPose().y,1000,{.forwards=false,.minSpeed=40});
    chassis.moveToPoint(-51,chassis.getPose().y,800,{.minSpeed=40});
    descore.retract();
    chassis.turnToHeading(-130,300);
    chassis.moveToPose(-10,47,-90,800,{.forwards=false,.minSpeed=40});
    chassis.waitUntil(16);
    scoretop();
    chassis.moveToPoint(-10,48,500,{.forwards=false});
    chassis.waitUntilDone();
    stop();
    chassis.setPose(-28.75,48,chassis.getPose().theta);
    
    chassis.moveToPoint(-40,chassis.getPose().y,300);
    out.move_relative(-400,600);
    top.move_relative(-600,600);
    chassis.turnToHeading(-45,300);
    chassis.moveToPoint(-8,-8,1300,{.forwards=false});
    chassis.waitUntil(26);
    scorebottomslow(); 
    chassis.turnToHeading(-45,300);
    chassis.waitUntilDone();   
    pros::delay(10000);
}
void drivskills(){
    chassis.setPose(-50,6,90);
    
    intake.move_voltage(13000);
    top.move_voltage(13000);
    chassis.moveToPoint(-24,24,1200,{.maxSpeed=70});
    chassis.waitUntil(12);
    match.extend();

    //chassis.turnToHeading(-45,1000);
    chassis.moveToPose(-7,6,-45,1400,{.forwards=false,.maxSpeed=70});
    chassis.waitUntil(20);
    scorebottomslow();
    pros::delay(700);
    chassis.moveToPoint(-50,50,1100);
    chassis.waitUntil(2);
    reclaim();
    chassis.turnToHeading(-90,500);
    stop();

    redloaderskillsclose();
    
    //crosstop

    chassis.moveToPoint(-40,60,1000,{.forwards=false});
    chassis.waitUntil(6);
    match.retract();

    chassis.turnToHeading(90,1000);
    
}
void midskills(){
    drivskills();
    stop();
    chassis.moveToPoint(34,59,1000,{.minSpeed=30});
    //match.retract();
    chassis.moveToPose(40,42,180,1200,{.minSpeed=30});
    chassis.turnToHeading(90,800);


    chassis.waitUntilDone();
    chassis.setPose(chassis.getPose().x,70.5-((leftdist.get_distance()/25.4+3.5)),chassis.getPose().theta);
    chassis.moveToPoint(20,48,1000,{.forwards=false,.maxSpeed=70});
    chassis.waitUntil(6);
    scoretop();
    pros::delay(900);
    outtakefast();
    pros::delay(200);
    scoretop();
    pros::delay(1500);

    //stop();
    
    chassis.setPose(28.75,48,chassis.getPose().theta);
    
    chassis.moveToPoint(48,47,1000,{.minSpeed=40});
    outtakefast();
    match.extend();
    //chassis.turnToHeading(90,600);
    chassis.waitUntil(8);
    run_intake();
    chassis.waitUntil(24);

    chassis.cancelMotion();
    redloaderskillsfar();
    
    chassis.moveToPoint(40,48,800,{.forwards=false});
    chassis.moveToPoint(0,48,1200,{.forwards=false,.maxSpeed=50});
    chassis.waitUntil(8);
    scoretop();
    pros::delay(900);
    outtakefast();
    pros::delay(200);
    scoretop();
    pros::delay(2000);

    //crossred
    chassis.waitUntilDone();
    far();
    chassis.turnToHeading(90,500);
    stop();
    chassis.waitUntilDone();
    chassis.setPose(chassis.getPose().x,-70.5+((rightdist.get_distance()/25.4+3.5)),chassis.getPose().theta);
    
    run_intake();

    redloaderskillsfar();
    
    //crossbottom
    chassis.moveToPoint(47,-50,600,{.forwards=false});

    chassis.moveToPoint(40,-62,600,{.forwards=false});
    chassis.waitUntil(6);
    match.retract();

    chassis.turnToHeading(-90,1000);
    stop();
    chassis.waitUntilDone();
    chassis.setPose(chassis.getPose().x,-70.5+((leftdist.get_distance()/25.4+3.5)),chassis.getPose().theta);

    chassis.moveToPoint(-34,-61,1000,{.minSpeed=30});
     chassis.moveToPose(-40,-43.8,0,1200,{.minSpeed=30});
    chassis.turnToHeading(-90,800);


    chassis.waitUntilDone();
    chassis.setPose(chassis.getPose().x,-70.5+((leftdist.get_distance()/25.4+3.5)),chassis.getPose().theta);
    chassis.moveToPoint(20,-48,1000,{.forwards=false});
    chassis.waitUntil(6);
    scoretop();
    pros::delay(900);
    outtakefast();
    pros::delay(200);
    scoretop();
    pros::delay(1500);

    stop();
    
    chassis.setPose(28.75,48,90);
    
    chassis.moveToPoint(50,47,1000,{.minSpeed=40});
    outtakefast();
    match.extend();
    //chassis.turnToHeading(90,600);
    chassis.waitUntil(24);
    run_intake();
    chassis.cancelMotion();
    redloaderskillsfar();
    
    chassis.moveToPoint(40,48,800,{.forwards=false});
    chassis.moveToPoint(0,48,1200,{.forwards=false,.maxSpeed=50});
    chassis.waitUntil(10);
    scoretop();
    pros::delay(900);
    outtakefast();
    pros::delay(200);
    scoretop();
    pros::delay(1800);

    //stop();
    //crossred
    //chassis.moveToPoint(40,48,800);

    chassis.moveToPoint(0,48,300,{.forwards=false,.maxSpeed=50});
    chassis.waitUntilDone();
    chassis.setPose(28.75,48,90);
    chassis.moveToPose(63.5,0,180,1000);
    match.retract();

    run_intake();
    chassis.moveToPoint(63.5,-40,2000,{.maxSpeed=75,.minSpeed=65});
    match.extend();
    chassis.waitUntil(12);
    match.retract(); 
    chassis.waitUntil(40);  
    chassis.cancelAllMotions();
}
void skills119(){
    descore.extend();

    //bottom mid 7
    bluezoneclear();
    chassis.turnToPoint(-24,-20,600);
    stop();
    chassis.moveToPoint(-26,-24,900);
    chassis.waitUntil(17);
    intake.move_relative(500,600);
    chassis.turnToHeading(45,800);
    chassis.moveToPoint(-11,-12,1000);
    out.move_relative(-200,200);
    //chassis.turnToHeading(48,800,{.minSpeed=40});
    chassis.waitUntilDone();
    pros::delay(20);
    outtakefast();
    //chassis.waitUntilDone();
    pros::delay(200);
    outtakeslow();
    pros::delay(1000);
    intake.move_voltage(12000);
    pros::delay(200);
    outtakeslow();
    pros::delay(1000);
    chassis.moveToPoint(chassis.getPose().x+2,chassis.getPose().y+2,300);

    //moveto1stloader
    chassis.moveToPoint(-20,-20,600,{.forwards=false,.minSpeed=30});
    chassis.turnToHeading(2,400,{.minSpeed=30});
    run_intake();
    chassis.moveToPoint(-40,47,1400);
    chassis.turnToHeading(-90,500);
    match.extend();
    chassis.waitUntilDone();
    chassis.setPose(chassis.getPose().x,right_reset(0.5,3.5,true),chassis.getPose().theta);
    redloaderskillsclose();
    chassis.turnToHeading(-90,300);
    chassis.waitUntilDone();
    chassis.setPose(-55.25,right_reset(0.5,3.5,true),chassis.getPose().theta);
    //chassis.setPose(-55.25,70.5-((rightdist.get_distance()/25.4+3.5)),-90);
    

    //crosstop
    //chassis.moveToPoint(chassis.getPose().x-20, chassis.getPose().y,400,{.forwards=false});
    chassis.moveToPose(-12,60,-90,1000,{.forwards=false,.minSpeed=50,.earlyExitRange=4});
    chassis.waitUntil(6);
    match.retract();
    stop();
    chassis.moveToPoint(34,60,1300,{.forwards=false,.minSpeed=30});
    chassis.waitUntilDone();
    chassis.setPose(chassis.getPose().x,right_reset(0.5,3.5,true),chassis.getPose().theta);
    chassis.moveToPose(38,47,90,1000,{.forwards=false,.minSpeed=30});
    chassis.moveToPoint(10,48,1000,{.forwards=false,.maxSpeed=80});
    match.extend();
    chassis.waitUntil(4);
    scoretop();
    chassis.waitUntilDone();
    pros::delay(1000);
    top.move_voltage(0);
    chassis.setPose(28.75,48,chassis.getPose().theta);   
    
    //moveto2ndloader
    chassis.moveToPoint(60,47,800,{.minSpeed=40});
    outtake_no_lift();
    chassis.waitUntil(16);
    run_intake();
    chassis.cancelMotion();
    redloaderskillsfar();    
    chassis.moveToPoint(40,49,600,{.forwards=false,.minSpeed=20});
    chassis.moveToPoint(0,49,1200,{.forwards=false,.maxSpeed=50});
    chassis.waitUntil(8);
    scoretop();
    pros::delay(900);
    outtake_no_lift();
    pros::delay(200);
    scoretop();
    pros::delay(500);
    out.move_voltage(5000);
    pros::delay(800);
    chassis.setPose(28.75,48,90); 
    match.retract();

    //red clear
    redzoneclear();

    //score
    chassis.turnToPoint(23,-23,600);
    chassis.moveToPoint(20,-23,700);
    chassis.waitUntil(30);
    match.extend();
    chassis.turnToHeading(135,500);
    out.move_relative(-300,200);
    top.move_voltage(-10000);
    //intake.move_relative(-200,600);
    chassis.moveToPoint(7,-5,1500,{.forwards=false});
    chassis.waitUntilDone();
    scorebottomslow();
    pros::delay(300);
    scorebottomrllyslow();
    int startTime = pros::millis();
    while ((colorsens.get_hue() < 200 || colorsens.get_hue() > 300) &&
        (pros::millis() - startTime < 3000)) {
        pros::delay(10);}
    out.move_voltage(7000);
    top.move_voltage(-10000);
    chassis.setPose(9,-9,chassis.getPose().theta);
    

    //moveto3rdloader
    chassis.moveToPoint(46,-48,1000);
    chassis.turnToHeading(90,800);
    redloaderskillsfar();
    chassis.turnToHeading(90,300);
    chassis.waitUntilDone();
    chassis.setPose(55.25,right_reset(0.5,3.5,false),chassis.getPose().theta);
    //chassis.setPose(-55.25,70.5-((rightdist.get_distance()/25.4+3.5)),-90);
    

    //crosstop
    //chassis.moveToPoint(chassis.getPose().x-20, chassis.getPose().y,400,{.forwards=false});
    chassis.moveToPose(20,-60,90,1000,{.forwards=false,.minSpeed=50,.earlyExitRange=4});
    chassis.waitUntil(6);
    match.retract();
    stop();
    chassis.moveToPoint(-34,-60,1300,{.forwards=false,.minSpeed=30});
    chassis.waitUntilDone();
    chassis.setPose(chassis.getPose().x,right_reset(0.5,3.5,false),chassis.getPose().theta);
    chassis.moveToPose(-38,-47,-90,1000,{.forwards=false,.minSpeed=30});
    chassis.moveToPoint(-10,-48,1000,{.forwards=false,.maxSpeed=80});
    match.extend();
    chassis.waitUntil(4);
    scoretop();
    chassis.waitUntilDone();
    pros::delay(1000);
    top.move_voltage(0);
    chassis.setPose(28.75,48,90);   
    
    


    //moveto2ndloader
    chassis.moveToPoint(60,48,800,{.minSpeed=40});
    outtake_no_lift();
    chassis.waitUntil(16);
    run_intake();
    chassis.cancelMotion();
    redloaderskillsfar();    
    chassis.moveToPoint(40,49,600,{.forwards=false,.minSpeed=20});
    chassis.moveToPoint(0,49,1200,{.forwards=false,.maxSpeed=70});
    chassis.waitUntil(8);
    scoretop();
    pros::delay(900);
    outtake_no_lift();
    pros::delay(200);
    scoretop();
    pros::delay(1000);
    out.move_voltage(8000);
    pros::delay(1000);
    chassis.setPose(28.75,48,90);   

    //park
    chassis.moveToPose(64,0,180,1000);
    match.retract();
    run_intake();
    chassis.moveToPoint(67,-80,2000,{.minSpeed=80});
    chassis.waitUntil(30);  
    chassis.cancelAllMotions();
}
void testclear(){
//descore.extend();

    //bottom mid 7
    bluezoneclear();
    //chassis.turnToPoint(-24,-22,400);
}
// Create robodash selector
rd::Selector selector({
    {"SAWP", &sawp, "", 50},//done
    {"7 hood L", &dump_l, "", 50},//done
    {"7 hood R", &dump_r, "", 150},
    {"4+3 L", &sev_l, "", 50},
    {"4+3 R", &sev_r, "", 150},//done
    {"fast L", &four_l, "", 50},//done
    {"fast R", &four_r, "", 150},
    {"Skills", &skills119, "", 250},
});
int mode = 0;

// Create robodash console
rd::Console console;
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensor
    //rightMotors.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    //leftMotors.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);

    colorsens.set_led_pwm(100);
    // starthue = colorsens.get_hue();
    //chassis.setPose(28.75,48,90);
    pros::Task selectTask([&]() {
        while(true){
            if(button.get_new_press()){
            selector.next_auton();
            }
            //printf("%f\n", imu.get_roll());
            //fflush(stdout);
            // if(colorsens.get_proximity()>70){
            //     descore.extend();
            //     pros::delay(300);
            // }else if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            //     descore.retract();
            // }
            selectTask.delay(50);
        }
    });
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            //pros::lcd::print(3, "front: %f", front_reset(3.5,5.25,true)); // heading
            //pros::lcd::print(4, "right: %f", right_reset(0.5,3.5,true)); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });

    selector.on_select([](std::optional<rd::Selector::routine_t> routine) {
		if (routine == std::nullopt) {
			std::cout << "No routine selected" << std::endl;
		} else {
			std::cout << "Selected Routine: " << routine.value().name << std::endl;
        
         controller.print(0,0,"%s", routine.value().name);
         controller.rumble("- -");
		}
	});
    
}

/**
 * Runs while the robot is disabled
 */
void disabled() {
    //.retract();
}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {
    //selector.focus();
}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(arc_txt); // '.' replaced with "_" to make c++ happy
ASSET(under_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs in driver control
 */
void autonomous(){
    //selector.run_auton();
    midskills();
    //far();
}
void opcontrol() {
    // controller
    // loop to continuously update motors
    chassis.setBrakeMode(MOTOR_BRAKE_COAST);

    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
        if(!controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)&&!controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)&&!controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)&&!controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)&&!controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
            stop();
            descore.extend();
            
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            descore.retract();
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            run_intake();    
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            scoretop();
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            scorebottom();
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
        {
            match.toggle();
            //chassis.turnToHeading(-90,600);
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
        {
            outtake_no_lift();
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B))
        {
            scorebottomslow();
        }
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
            outtake_no_lift();
        }
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A))
        {
            
            sev_r();
        }
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X))
        {
            nine_r();
        }
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
            nine_l();
        }
        // delay to save resources
        pros::delay(10);
    }
}

