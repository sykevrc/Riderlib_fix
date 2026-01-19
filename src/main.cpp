#include "main.h"
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

pros::Motor intake(1);
pros::Motor top(8);
pros::Motor out(21);

pros::adi::Pneumatics hood(1, false);
pros::adi::Pneumatics descore(2, false);
pros::adi::Pneumatics match(6, false);
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
pros::Distance rightdist(18);
pros::Distance leftdist(5);

pros::Optical colorsens(3);
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

void run_intake(){
    intake.move_voltage(13000);
    top.move_voltage(8000);
    out.move_voltage(12000);
    hood.retract();
    //lift.retract();
}

void outtakefast(){
    intake.move_voltage(-8000);
    top.move_voltage(-8000);
}
void scoretop(){
    intake.move_voltage(13000);
    top.move_voltage(13000);
    out.move_voltage(13000);
    hood.extend();
}
void scorebottom(){
    intake.move_voltage(13000);
    top.move_voltage(13000);
    out.move_voltage(-10000);

    //hood.extend();  
}
void scorebottomslow(){
    intake.move_voltage(13000);
    top.move_voltage(13000);
    out.move_velocity(-90);
}
void scoot(){
    intake.move_voltage(13000);
    top.move_voltage(0);
    //out.move_voltage(10000);
}
void stop(){
    intake.move_voltage(0);
    top.move_voltage(0);
    out.move_voltage(0);

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
    chassis.moveToPoint(-95,chassis.getPose().y,600,{.maxSpeed=14,.minSpeed=10});
    pros::delay(300);
    if(color){
    if(colorsens.get_hue()>100)chassis.cancelMotion();

    }else{
    if(colorsens.get_hue()<180)chassis.cancelMotion();

    }
    //pros::delay(100);
    //chassis.moveToPoint(chassis.getPose().x+.5,chassis.getPose().y,300,{.forwards=false,.minSpeed=30});
    //chassis.moveToPoint(chassis.getPose().x-15,chassis.getPose().y,1000,{.minSpeed=30});

}
void redloaderskillsclose(){
    chassis.moveToPoint(-75,chassis.getPose().y,500,{.maxSpeed=45,.minSpeed=40});
    run_intake();

    chassis.moveToPoint(-75,chassis.getPose().y,800,{.maxSpeed=14,.minSpeed=10});
    chassis.waitUntilDone();
    pros::delay(600);
    //chassis.moveToPoint(chassis.getPose().x+3,chassis.getPose().y,500);
    //chassis.moveToPoint(-75,chassis.getPose().y,500,{.maxSpeed=45,.minSpeed=40});
    //pros::delay(500);
    
}
void redloaderskillsfar(){
    chassis.moveToPoint(75,chassis.getPose().y,500,{.maxSpeed=45,.minSpeed=40});
    run_intake();

    chassis.moveToPoint(75,chassis.getPose().y,800,{.maxSpeed=14,.minSpeed=10});
    chassis.waitUntilDone();
    pros::delay(600);
    //chassis.moveToPoint(chassis.getPose().x-3,chassis.getPose().y,500);
    //chassis.moveToPoint(75,chassis.getPose().y,500,{.maxSpeed=45,.minSpeed=40});
}
/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
//constexpr float degToRad(float deg) { return deg * M_PI / 180; }

void skillsv2() {
   
    //left red
    chassis.setPose(-70.5+(leftdist.get_distance()/25.4+4.5),14,0); 
    chassis.moveToPose(chassis.getPose().x, 48,-90,3000,{.earlyExitRange=2});
    hood.extend();
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
void rush_l(){
    chassis.setPose(-50.5,16.5,90);
    chassis.moveToPoint(-24,24,800,{.minSpeed=10});
    run_intake();
    chassis.waitUntil(10.5);
    match.extend();
    chassis.turnToHeading(-45,600);
    chassis.moveToPoint(-50,49,1000);
    top.move_relative(600,600);
    chassis.turnToHeading(-90,600);
    
    redloaderquick(true);
    top.move_relative(300,600);
    chassis.moveToPoint(-24,chassis.getPose().y+2,1000,{.forwards=false});
    chassis.waitUntil(16);
    scoretop();
    pros::delay(2000);
    chassis.waitUntilDone();
    chassis.setPose(-28.75,48,chassis.getPose().theta);
    
    push(true);
}
void rush_r(){
    chassis.setPose(-50.5,-19.5,90);
    //chassis.setPose(-57,20.5,90);
    chassis.moveToPoint(-24,-24,1000,{.minSpeed=10});
    run_intake();
    chassis.waitUntil(16);
    match.extend();
    chassis.turnToHeading(-135,800);
    chassis.moveToPoint(-47,-48,1500,{.minSpeed=10});
    top.move_relative(600,600);
    chassis.turnToHeading(-90,700,{.maxSpeed=50});
    
    redloaderquick(true);
    top.move_relative(300,600);
    chassis.moveToPoint(-15,-49,1000,{.forwards=false});
    chassis.waitUntil(16);
    scoretop();
    pros::delay(2000);
    chassis.setPose(-28.75,-48,chassis.getPose().theta);
    push(false);
}
void elim_l(){
    chassis.setPose(-50.5,16.5,90);
    chassis.moveToPoint(-24,24,1000,{.minSpeed=10});
    run_intake();
    chassis.waitUntil(10.5);
    match.extend();
    chassis.moveToPose(-6,52,0,1500,{.minSpeed=20});
    match.retract();
    chassis.waitUntil(22);
    match.extend();
    pros::delay(300);
    chassis.moveToPoint(-6,25,1000,{.forwards=false,.minSpeed=30});

    chassis.turnToHeading(-90,400,{.minSpeed=30});

    
    chassis.moveToPose(-42,64,0,1500);
    //top.move_relative(400,600);
    chassis.turnToHeading(-95,600);
    chassis.waitUntilDone();

    chassis.setPose(chassis.getPose().x,70.5-((rightdist.get_distance()/25.4+3)),chassis.getPose().theta);

    chassis.moveToPoint(-20,48,700,{.forwards=false});
    chassis.waitUntil(8);
    scoretop();
    pros::delay(1600);
    chassis.waitUntilDone();
    chassis.setPose(-28.75,48,chassis.getPose().theta);
    chassis.moveToPoint(-56,47,600);
    run_intake();
    redloaderquick(true);
    chassis.moveToPoint(-48,chassis.getPose().y,600,{.forwards=false});
    chassis.turnToHeading(-45,500);
    chassis.moveToPoint(-9,9,1200,{.forwards=false});
    chassis.waitUntil(24);
    scorebottom();    
    chassis.turnToHeading(-45,300);
    match.retract();
    pros::delay(800);
    chassis.moveToPoint(-44,35,600);
    chassis.turnToHeading(90,400);
    chassis.moveToPoint(-10,37,700);
}
void elim_r(){
    chassis.setPose(-50.5,-19.5,90);
    //chassis.setPose(-57,20.5,90);
    chassis.moveToPoint(-24,-24,1000,{.minSpeed=10});
    run_intake();
    chassis.waitUntil(16);
    match.extend();
    chassis.turnToPoint(-8,-30,400);
    match.retract();
    chassis.moveToPoint(-7,-30,700,{.maxSpeed=45});
    chassis.turnToHeading(180,400);
    chassis.moveToPoint(-7,-37,700);
    chassis.waitUntil(8);
    match.extend();
    pros::delay(500);
    chassis.moveToPoint(-8,-25,700,{.forwards=false});
    chassis.turnToHeading(-90,400);

    
    chassis.moveToPoint(-47,-47,1500,{.minSpeed=10});
    top.move_relative(600,600);
    chassis.turnToHeading(-90,700,{.maxSpeed=50});
    
    redloaderquick(true);
    top.move_relative(300,600);
    chassis.moveToPoint(-15,-49,1000,{.forwards=false});
    chassis.waitUntil(16);
    scoretop();
    pros::delay(2400);
    chassis.setPose(-28.75,-48,chassis.getPose().theta);
    descore.extend();
    chassis.moveToPoint(-44,chassis.getPose().y,600);
    
    chassis.turnToHeading(-135,500);
    descore.retract();

    chassis.moveToPoint(-25,-37,600,{.forwards=false});
    chassis.turnToHeading(-90,500);

    chassis.moveToPoint(-9,-37,800,{.forwards=false});
    chassis.setBrakeMode(MOTOR_BRAKE_HOLD);
    match.retract();
    chassis.turnToHeading(-140,500,{.minSpeed=40});

}
void sawp(){
    chassis.setPose(-70.5+(leftdist.get_distance()/25.4+3),-3,0); 
    chassis.moveToPoint(chassis.getPose().x, 4,500,{.minSpeed=40});
    //hood.extend();
    run_intake();
    chassis.moveToPoint(-46,-48.5,1200,{.forwards=false});
    match.toggle();
    chassis.turnToHeading(-90,400);
    redloaderquick(true);

    chassis.moveToPoint(-20,-48,1200,{.forwards=false});
    chassis.waitUntil(13);
    scoretop();
    chassis.waitUntilDone();
    pros::delay(800);
    chassis.setPose(-28.75,-48,chassis.getPose().theta);
    chassis.moveToPoint(-40,-48,500);
    match.retract();
    run_intake();
    chassis.turnToHeading(45,500);

    chassis.moveToPoint(-24,-24,1000);
    chassis.waitUntil(11);
    match.extend();
    chassis.turnToHeading(0,700);
    match.retract();

    chassis.moveToPoint(-24,24,1200);
    chassis.waitUntil(20);
    match.extend();
    chassis.turnToHeading(-45,500);

    chassis.moveToPoint(-10,10,800,{.forwards=false});
    scorebottom();


    match.retract();
    chassis.turnToHeading(-45,500);
    
    pros::delay(700);
    stop();
    chassis.moveToPoint(-50,48.5,1000);
    run_intake();
    match.extend();
    top.move_relative(200,600);
    chassis.turnToHeading(-90,800);
    chassis.waitUntilDone();

    chassis.setPose(chassis.getPose().x,70.5-(rightdist.get_distance()/25.4+3),chassis.getPose().theta);
    
    redloaderquick(true);
    chassis.moveToPoint(-20,48,1000,{.forwards=false});
    chassis.waitUntil(15);
    scoretop();
}
void midskills(){
    chassis.setPose(-50,6,90);
    
    run_intake();

    chassis.moveToPoint(-24,24,1200,{.maxSpeed=70});
    chassis.waitUntil(12);
    match.extend();

    //chassis.turnToHeading(-45,1000);
    chassis.moveToPose(-7,6,-45,1400,{.forwards=false,.maxSpeed=70});
    chassis.waitUntil(15);
    scorebottom();
    pros::delay(200);
    scorebottomslow();
    pros::delay(800);
    chassis.moveToPoint(-50,50,1000);
    chassis.turnToHeading(-90,500);

    redloaderskillsclose();
    
    //crosstop

    chassis.moveToPoint(-40,60,1000,{.forwards=false});
    chassis.waitUntil(6);
    match.retract();

    chassis.turnToHeading(90,1000);
    stop();
    chassis.moveToPoint(34,58,1000,{.minSpeed=30});
    //match.retract();
    chassis.moveToPose(40,43,180,1200,{.minSpeed=30});
    chassis.turnToHeading(90,800);


    chassis.waitUntilDone();
    chassis.setPose(chassis.getPose().x,70.5-((leftdist.get_distance()/25.4+3)),chassis.getPose().theta);
    chassis.moveToPoint(0,48,1000,{.forwards=false,.maxSpeed=70});
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
    chassis.setPose(28.75,48,90);
    chassis.moveToPose(63.5,0,180,1000);
    match.retract();

    chassis.moveToPoint(63.5,-40,2000,{.maxSpeed=70,.minSpeed=70});
    run_intake();
    chassis.waitUntil(28);
    match.extend();
    chassis.moveToPoint(64,-80,500,{.maxSpeed=45,.minSpeed=33});
    
    chassis.moveToPoint(64,-10,400,{.forwards=false,.minSpeed=30});
    chassis.moveToPoint(64,-10,800,{.forwards=false,.maxSpeed=30});
    chassis.waitUntilDone();
    chassis.setPose(70.5-(leftdist.get_distance()/25.4+3),-14.5,chassis.getPose().theta);
    pros::delay(50);

    chassis.moveToPoint(chassis.getPose().x-1,-25,1000,{.maxSpeed=40});
    match.retract();

    chassis.swingToHeading(-80,lemlib::DriveSide::RIGHT,800);
    
    chassis.moveToPoint(20,-15,1200);
    chassis.waitUntil(20);
    chassis.turnToHeading(132,800,{.maxSpeed=50});
    chassis.moveToPoint(12,-13,800,{.forwards=false});
    chassis.turnToHeading(135,300,{.minSpeed=20});
    outtakefast();
    chassis.waitUntil(3);
    scorebottom();
    
    pros::delay(200);
    scorebottomslow();
    pros::delay(2400);
    


    //last half
    chassis.moveToPoint(48,-50,1200);
    match.extend();
    
    chassis.turnToHeading(90,500);
    scorebottom();
    chassis.waitUntilDone();
    chassis.setPose(chassis.getPose().x,-70.5+((rightdist.get_distance()/25.4+3)),chassis.getPose().theta);
    
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
    chassis.setPose(chassis.getPose().x,-70.5+((leftdist.get_distance()/25.4+3)),chassis.getPose().theta);

    chassis.moveToPoint(-34,-61,1000,{.minSpeed=30});
     chassis.moveToPose(-40,-44,0,1200,{.minSpeed=30});
    chassis.turnToHeading(-90,800);


    chassis.waitUntilDone();
    chassis.setPose(chassis.getPose().x,-70.5+((leftdist.get_distance()/25.4+3)),chassis.getPose().theta);
    chassis.moveToPoint(0,-48,1000,{.forwards=false});
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
    chassis.waitUntil(33);  
    chassis.cancelAllMotions();
}
void drivskills(){
    chassis.setPose(-50,6,90);
    
    run_intake();

    chassis.moveToPoint(-24,24,1200,{.maxSpeed=70});
    chassis.waitUntil(12);
    match.extend();

    //chassis.turnToHeading(-45,1000);
    chassis.moveToPose(-7,6,-45,1400,{.forwards=false,.maxSpeed=70});
    chassis.waitUntil(15);
    scorebottom();
    pros::delay(200);
    scorebottomslow();
    pros::delay(800);
    chassis.moveToPoint(-50,50,1000);
    chassis.turnToHeading(-90,500);

    redloaderskillsclose();
    
    //crosstop

    chassis.moveToPoint(-40,60,1000,{.forwards=false});
    chassis.waitUntil(6);
    match.retract();

    chassis.turnToHeading(90,1000);
    stop();
    
}
void far(){
    chassis.setPose(28.75,48,90);
    chassis.moveToPose(63.5,0,180,1000);
    match.retract();

    chassis.moveToPoint(63.5,-40,2000,{.maxSpeed=70,.minSpeed=70});
    run_intake();
    chassis.waitUntil(28);
    match.extend();
    chassis.moveToPoint(64,-80,500,{.maxSpeed=45,.minSpeed=33});
    
    chassis.moveToPoint(64,-10,400,{.forwards=false,.minSpeed=30});
    chassis.moveToPoint(64,-10,800,{.forwards=false,.maxSpeed=30});
    chassis.waitUntilDone();
    chassis.setPose(70.5-(leftdist.get_distance()/25.4+3),-14.5,chassis.getPose().theta);
    pros::delay(50);

    chassis.moveToPoint(chassis.getPose().x-1,-25,1000,{.maxSpeed=40});
    match.retract();

    chassis.swingToHeading(-80,lemlib::DriveSide::RIGHT,800);
    
    chassis.moveToPoint(20,-18,1200);
    chassis.waitUntil(20);
    match.extend();
    chassis.turnToHeading(132,800,{.maxSpeed=50});
    chassis.moveToPoint(12,-13,800,{.forwards=false});
    chassis.turnToHeading(135,300,{.minSpeed=20});
    outtakefast();
    chassis.waitUntil(3);
    scorebottom();
    
    pros::delay(200);
    scorebottomslow();
}
// Create robodash selector
rd::Selector selector({
    {"SAWP", &sawp, "", 50},//done
    {"ElimR", &elim_r, "", 150},
    {"ElimL", &elim_l, "", 50},//done
    {"Q L", &rush_l, "", 50},//done
    {"Q R", &rush_r, "", 150},
    {"Skills", &midskills, "", 250},
});
int mode = 0;

// Create robodash console
rd::Console console;
void initialize() {
    //pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensor
    //rightMotors.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    //leftMotors.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);

    colorsens.set_led_pwm(100);
    // starthue = colorsens.get_hue();

    // pros::Task selectTask([&]() {
    //     while(true){
    //         if(button.get_value()){
    //         selector.next_auton();
    //         }
            
    //     }
    // });
    

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
    hood.retract();
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
    selector.run_auton();
    
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
            descore.retract();
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            descore.extend();
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
            outtakefast();
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B))
        {
            scorebottomslow();
        }
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A))
        {
            
            chassis.setPose(28.75,48,90);
            chassis.moveToPose(63.5,0,180,1000);
            match.retract();

            run_intake();
            chassis.moveToPoint(63.5,-40,2000,{.maxSpeed=75,.minSpeed=65});
            match.extend();
            chassis.waitUntil(12);
            match.retract(); 
            chassis.waitUntil(33);  
            chassis.cancelAllMotions();
        }
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
        {
            chassis.setPose(28.75,48,90);
            chassis.moveToPose(63.5,0,180,1000);
            match.retract();

            chassis.moveToPoint(63.5,-40,2000,{.maxSpeed=70,.minSpeed=70});
            run_intake();
            chassis.waitUntil(28);
            match.extend();
            chassis.moveToPoint(64,-80,500,{.maxSpeed=45,.minSpeed=33});
        }
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
            drivskills();
            }
        // delay to save resources
        pros::delay(10);
    }
}

