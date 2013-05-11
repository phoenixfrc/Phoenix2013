#include "WPILib.h"
#include "Robot.h"
#include "Control.h"
#include "Drive.h"
#include "Encoder.h"
#include "Log.h"

// If the climber is within this distance of the target in either direction,
// it is considered "good enough"
#define ClimberCloseTolerance 0.5

#define DriveDistancePerPulse (12./128.)

// Climber - 
#define ClimberDistancePerPulse (1.0/490.0)


static const int DisplayInterval = 2;
static int displayCount = 0;
static double currentDistance = 0;
static bool display() { return true; }

class BcdSwitch {
    DigitalInput *_port[4];
public:
    BcdSwitch(DigitalInput* port1, DigitalInput* port2,
    				DigitalInput* port3, DigitalInput* port4) {
        _port[0] = port1;
        _port[1] = port2;
        _port[2] = port3;
        _port[3] = port4;
    }
    int value() {
        int ret = 0;
        for(int i = 0; i < 4; ++i) {
            if (!_port[i]->Get()) // func returns 0 for closed
                ret |= 1<<i;
        }
        return ret;
    }

};

class Robot : public IterativeRobot {
public:
    enum HookSwitch {
        NoHookSwitch,
        UpperHookSwitch,
        LowerHookSwitch
    };
    
    enum ClimbState {
        NotInitialized,    // climbers not yet deployed at all
        InitialGrab,     // Pulling both arms down enough to latch (low power consumption)
        Abort,
            ClimbStateLast
    };
    ClimbState climbState;
    static const char* ClimbStateString(ClimbState climbState) {
        static const char* climbStateString[ClimbStateLast] = {
            "NotInitialized",
            "InitialGrab",
            "Abort"
        };
        return climbStateString[climbState];
    }
    enum Bar {
        LowerBar,
        MiddleBar,
        UpperBar
    };
    Bar bar;
    static const char* BarString(Bar bar) {
        static const char* barString[] = {
            "LowerBar",
            "MiddleBar",
            "UpperBar"    
        };
        return barString[bar];
    }
    class ControlledMotor{
    public:
    	Robot* robot;
        const char* name;
        CANJaguar* motor;
        Encoder* encoder;
        double distancePerPulse;
        DigitalInput* lowerLimitSwitch;
        PIDController* motorController;
        ControlledMotor(
        		Robot* r,
				const char* n,
				CANJaguar* m,
				Encoder* e,
				double dpp,
				DigitalInput* l) :
			robot(r), name(n), motor(m), encoder(e),
			distancePerPulse(dpp),lowerLimitSwitch(l),
			motorController()
		{
        	motorController = new PIDController(.1, .001, 0.0, encoder, motor);
        	encoder->SetDistancePerPulse(dpp);
		}

    };
    class Climber: public ControlledMotor {
	public:
		DigitalInput* lowerHookSwitch;
		DigitalInput* upperHookSwitch;
		Climber (
                Robot* r,
                const char* n,
                CANJaguar* m,
                Encoder* e,
                double dpp,
                DigitalInput* l,
                DigitalInput* lowerHook,
                DigitalInput* upperHook) :
                	ControlledMotor(r,n,m,e,dpp,l), lowerHookSwitch(lowerHook), upperHookSwitch(upperHook)
        {}
		bool UpdateState(
				float powerLevel,    //typically 1.0 for forward -1.0 for backward
				double targetPosition,
				HookSwitch hookSwitch=NoHookSwitch)
		{
		    
			if (powerLevel < 0.0 && !lowerLimitSwitch->Get()){
				if (motorController) {
					motorController->Disable();
				}
				motor->Set(0.0);
            	encoder->Reset();
				encoder->Start();
				return true;
			}
			double position = encoder->GetDistance();
			if (motorController) {
				if (!motorController->IsEnabled())
					motorController->Enable();
				motorController->SetSetpoint(targetPosition);
			} else {
            	if (position < targetPosition-ClimberCloseTolerance) {
            		motor->Set(1.0);
            	} else if (position > targetPosition+ClimberCloseTolerance){
            		motor->Set(-1.0);            		
            	}
			}

			if ( (hookSwitch == UpperHookSwitch && !upperHookSwitch->Get())||
				 (hookSwitch == LowerHookSwitch && !lowerHookSwitch->Get())||
				 (hookSwitch == NoHookSwitch &&
							targetPosition -ClimberCloseTolerance <= position && position <= targetPosition + ClimberCloseTolerance ) ) {
				if (motorController) {
					motorController->Disable();
				}
				motor->Set(0.0);
				encoder->Start();
				return true;
			} else if (hookSwitch != NoHookSwitch &&
					( (powerLevel > 0 && position > targetPosition) ||
						(powerLevel < 0 && position < targetPosition) ) ) {
				robot->AbortClimb("Grab did not occur when expected");
				return false;
			} // else if (hookSwitch != NoHookSwitch && Check motor power draw)
			// abort here if looking for switch and started drawing a lot of power 
			return false;
		}
    };
    Log* log;

    BcdSwitch* bcd;
    int bcdValue;

    Control* control;

    Drive* drive;
    Encoder* leftDriveEncoder;
    Encoder* rightDriveEncoder;
   
    //Compressor *compressor;
    
    Climber* climber;
    Climber* leftClimber;
    Climber* rightClimber;

    Relay* loaderMotor;
    DigitalInput* loaderSwitch;
    bool loading;
    bool loaderDisengageDetected;

    CANJaguar* shooterMotor;
    double shooterMotorVolts;
    
    CANJaguar* blowerMotor;

    double goalDistance;
    int autonomousShooterDelay;
    bool startingState;
    
    Servo* cameraPivotMotor;
    Servo* cameraElevateMotor;
    float cameraElevateAngle;
    float cameraPivotAngle;
    
    Relay* lightRing;
    
	int loadSwitchDelay;
	int loadCount;
	bool loadSwitchOldState;
public:
    Robot() {
        log = new Log(this);

        bcd = new BcdSwitch(new DigitalInput(2,11), new DigitalInput(2,12),
        					new DigitalInput(2,13), new DigitalInput(2,14));
        bcdValue = bcd->value();

        control = new Control(
                new Joystick(1), new Joystick(2), new Joystick(3), 
                Control::Tank, log);
        control->setLeftScale(-1);
        control->setRightScale(-1);
        control->setGamepadScale(-1);

        drive = new Drive(this, 1,2);
        drive->addMotor(Drive::Left, 2, 1);
        drive->addMotor(Drive::Left, 3, 1);
        drive->addMotor(Drive::Right, 4, 1);
        drive->addMotor(Drive::Right, 5, 1);

        rightDriveEncoder = new Encoder(1,1, 1,2, true, Encoder::k2X);
        leftDriveEncoder = new Encoder(1,3, 1,4, false, Encoder::k2X);
        
        //compressor = new Compressor(2,9, 2,3); //press, relay
		//compressor->Start();

        leftClimber = new Climber(
        	this,
            "leftClimber",
            new CANJaguar(6),
            new Encoder(1,5, 1,6, true),
            ClimberDistancePerPulse,
            new DigitalInput(2,1),//lower limit switch
            new DigitalInput(2,2),//lower hook switch
            new DigitalInput(2,3) );//upper hook switch
        
        rightClimber = new Climber(
        	this,
            "rightClimber",
            new CANJaguar(7),
            new Encoder(1,7, 1,8, true),
            ClimberDistancePerPulse,
            new DigitalInput(2,4),//lower limit switch
			new DigitalInput(2,5),//lower hook switch
			new DigitalInput(2,6) );//upper hook switch	

        loaderMotor = new Relay(2,1);
        loaderSwitch = new DigitalInput(2,8);

        shooterMotor = new CANJaguar(10, CANJaguar::kVoltage);
        
        blowerMotor = new CANJaguar(9);

        cameraPivotMotor = new Servo(1,9);
        cameraElevateMotor = new Servo(1,10);

        lightRing = new Relay(2,4);
    }
    
    // WHen robot is enabled
    void init() {
        log->info("initializing");
        log->print();
        climber = NULL;
        leftDriveEncoder->Reset();
        leftDriveEncoder->SetDistancePerPulse(DriveDistancePerPulse);
        leftDriveEncoder->Start();
		rightDriveEncoder->Reset();
        rightDriveEncoder->SetDistancePerPulse(DriveDistancePerPulse);
        rightDriveEncoder->Start();
        //leftClimber->motorController->Disable();
		leftClimber->encoder->Reset();
        leftClimber->encoder->Start();
        //rightClimber->motorController->Disable();
        rightClimber->encoder->Reset();
        rightClimber->encoder->Start();
        bcdValue = bcd->value();
        loadSwitchOldState = loaderSwitch->Get();
        #if 0
		bool leftDone = false;
		bool rightDone = false;
        // Only do this for some BCD values?
        while (!leftDone || !rightDone){
			if (!leftDone)
				leftDone = leftClimber->UpdateState(-1.0, -30);
			if (!rightDone)
				rightDone = rightClimber->UpdateState(-1.0, -30);
	        log->info("Wait: Ll Rl: %d %d",
	        		leftClimber->lowerLimitSwitch->Get(),
	        		rightClimber->lowerLimitSwitch->Get());
	        log->print();
		}
#endif
        climbState = NotInitialized;
        cameraElevateAngle =
        		(cameraElevateMotor->GetMaxAngle()-cameraElevateMotor->GetMinAngle()) * 2/3;
        cameraPivotAngle = 0;
        cameraPivotMotor->SetAngle(cameraPivotAngle);
        cameraElevateMotor->SetAngle(cameraElevateAngle);
        loading = false;
        loaderDisengageDetected = false;
        //This is a rough guess of motor power it should be based on voltage
        shooterMotorVolts = 8.0; // volts as a fraction of 12V
    	loadCount = 0;
    }

    void AutonomousInit() {
        init();
        currentDistance = 0;
        goalDistance = 6.0*12.0; // 6 feet
        autonomousShooterDelay = 0;
        shooterMotor->Set(0);
        loaderMotor->Set(Relay::kOff);
    }
    
    void AutonomousPeriodic() { 
    	
    	currentDistance = leftDriveEncoder->GetDistance();
		shooterMotor->Set(-shooterMotorVolts);
		
    	if (currentDistance >= goalDistance){
    		drive->setLeft(0);
    		drive->setRight(0);
    		loaderMotor->Set(Relay::kForward);
    	} else {
    		drive->setLeft(.49);
    		drive->setRight(.5);
    	}
        //log->info("LRD %f %f",
        //		leftDriveEncoder->GetDistance(),
        //		rightDriveEncoder->GetDistance());
    	//log->print();
    }
    void AutonomousDisabled() {
        //delete autonomous;
    }

    void setClimbState(ClimbState newState) {
        if (climbState == Abort)
            return;
        climbState = newState;
        startingState = true;
        log->info("new state: %s", ClimbStateString(newState));
        log->print();
    }

    void AbortClimb(const char* message) {
        log->info("%s",message);
        log->info("state was: %s", ClimbStateString(climbState));
        log->print();
        climbState = Abort;
        // ensure all motors are shut down
        leftClimber->motor->Set(0.0);
        rightClimber->motor->Set(0.0);
    }
     
    void ClimbPeriodic() {
    	displayCount++;
        // This is the distance in inches the climber must be initially raised
        static const float PullupPostion = -5.0f;
        log->info("current state is: %s\n",ClimbStateString(climbState));
        // depending on state, cont
        switch (climbState) {
        case NotInitialized: {
            leftClimber->encoder->Start();
            rightClimber->encoder->Start();
            bar = LowerBar;
            climber = NULL;
            setClimbState(InitialGrab);
            break; }
        //when they set up the robot they have to set the climbers 6" above the bar
       
        case InitialGrab: {
            // Pulling both arms down enough to latch (low power consumption)
            // We have several options here:
            // 1. Move a specific distance and assume it is right
            // 2. Move until the power goes up, and then check the encoders to see if the distance
            //    is plausible
            // 3. Move until a limit HookSwitch inside the grip for each climber trips
            bool leftDone = leftClimber->UpdateState(-1.0, PullupPostion);
            bool rightDone = rightClimber->UpdateState(-1.0, PullupPostion);
            startingState = false;
            // moving climbers into initial position
            if (leftDone && rightDone)
                setClimbState(Abort);
            break; }
       
        case Abort: {
            return;
            }
        default:
            log->info("unexpected climber state!");
            log->print();
        }
        log->info("LR %f %f",
    			leftClimber->encoder->GetDistance(),
    			rightClimber->encoder->GetDistance());
    	log->info("LRL %d %d %d %d %d %d %d",
    		leftClimber->lowerLimitSwitch->Get(),
    		leftClimber->lowerHookSwitch->Get(),
    		leftClimber->upperHookSwitch->Get(),
    		rightClimber->lowerLimitSwitch->Get(),
    		rightClimber->lowerHookSwitch->Get(),
    		rightClimber->upperHookSwitch->Get(),
    		loaderSwitch->Get());
    	log->print();
    }

    void TeleopInit() {
        init();
        drive->setShiftMode(Drive::Manual);
    	shooterMotor->Set(0.0);  
		loaderMotor->Set(Relay::kOff);
    }

    void TeleopPeriodic() {
    	displayCount++;
    	//tbs change button number
        if ((control->gamepadButton(9) && control->gamepadButton(10)) ||	// start
        			climbState != NotInitialized) {							// continue
        	// Do we want a manual abort here?
            ClimbPeriodic();
            return;
        }

        // drive
        drive->setLeft(control->left());
        // control->setRightScale(.95);
        drive->setRight(control->right());
        drive->setScale(control->throttle());
        //drive->setLowShift(control->button(1)); // right trigger
        drive->setReversed(control->toggleButton(11)); // right JS button 11
        
        //turn light on or off
        //lightRing->Set(control->gamepadToggleButton(4) ? Relay::kForward : Relay::kOff );
       
        blowerMotor->Set(control->gamepadButton(6) ? 1.0 : 0.0 );

        // For the loader, if we are rotating, wait for at least 100 counts before
        // checking the switch or adjusting motor power
        if (loading) {
        	//if (loaderSwitch->Get()) {
        		//loaderDisengageDetected = true;
        	//}
        	loadSwitchDelay++;
        	// If already rotating, and the switch trips, power down the motor
        	if (/*loaderDisengageDetected*/ loaderSwitch->Get() != loadSwitchOldState) {
				loaderMotor->Set(Relay::kOff);
				loading = false;
				loadSwitchOldState = loaderSwitch->Get();
        	}
        } else {
           	// If not rotating and the gamepad button is set, start rotating
        	if (control->gamepadButton(7)) {
        		loadSwitchDelay = 0;
        		loadCount++;
        		loaderMotor->Set(Relay::kForward);
        		loading = true;
        		loaderDisengageDetected = false;
        	}
        }
        
        if (control->gamepadToggleButton(4)){
        	if (control->gamepadRightVertical() > 0.05 ||
				control->gamepadRightVertical() < -0.05) {
				shooterMotorVolts += control->gamepadRightVertical();
				if (shooterMotorVolts < 6.0)
					shooterMotorVolts = 6.0;
				if (shooterMotorVolts > 12.0)
					shooterMotorVolts = 12.0;
		   }
        }
        
        // For the shooter, spin it up or down based on the toggle
        //
        if (control->gamepadToggleButton(8)) {
        	shooterMotor->Set(-shooterMotorVolts);// negative because motor is wired backwerds
        	log->info("shooter on");
        } else {
        	shooterMotor->Set(0.0);
        	log->info("shooter off");
        	
        }
        
        if (control->gamepadLeftVertical() > 0.05 ||
        		control->gamepadLeftVertical() < -0.05) {
        	cameraElevateAngle += control->gamepadLeftVertical()*5;
			if (cameraElevateAngle < cameraElevateMotor->GetMinAngle())
				cameraElevateAngle = cameraElevateMotor->GetMinAngle();
			if (cameraElevateAngle > cameraElevateMotor->GetMaxAngle())
				cameraElevateAngle = cameraElevateMotor->GetMaxAngle();
        }
        if (control->gamepadLeftHorizontal() > 0.05 ||
        		control->gamepadLeftHorizontal() < -0.05) {
        	cameraPivotAngle += control->gamepadLeftHorizontal()*5;
			if (cameraPivotAngle < cameraPivotMotor->GetMinAngle())
				cameraPivotAngle = cameraPivotMotor->GetMinAngle();
			if (cameraPivotAngle > cameraPivotMotor->GetMaxAngle())
				cameraPivotAngle = cameraPivotMotor->GetMaxAngle();
        }
        cameraPivotMotor->SetAngle(cameraPivotAngle);
        cameraElevateMotor->SetAngle(cameraElevateAngle);

        if (control->gamepadToggleButton(1)) {
        	// If the climber lower limit switch is set, and the distance is >0, set it to 0
        	if (!leftClimber->lowerLimitSwitch->Get()) {
    			leftClimber->encoder->Reset();
    			leftClimber->encoder->Start();
        		// Only allow forward motion
        		if (control->gamepadRightVertical() > 0) {
        			leftClimber->motor->Set(control->gamepadRightVertical());
        		} else {
        			leftClimber->motor->Set(0.0);
        		}
        	} else {
        		leftClimber->motor->Set(control->gamepadRightVertical());
        	}
        }
        if (control->gamepadToggleButton(3)) {
        	// If the climber lower limit switch is set, and the distance is >0, set it to 0
        	if (!rightClimber->lowerLimitSwitch->Get()) {
    			rightClimber->encoder->Reset();
    			rightClimber->encoder->Start();
        		// Only allow forward motion
        		if (control->gamepadRightVertical() > 0) {
        			rightClimber->motor->Set(control->gamepadRightVertical());
        		} else {
        			rightClimber->motor->Set(0.0);
        		}
        	} else {
        		rightClimber->motor->Set(control->gamepadRightVertical());
        	}
        }

        // assorted debug
        //log->info("Shift %s", control->toggleButton(8) 
        //        ? "low" : "high");
        //log->info("ArmPot: %.0f", arm->encoderValue());
        //log->info("pidf: %.2f", arm->pidFactor());
        //log->info("piden: %s", arm->isPidEnabled() ? "true" : "false");

        //double myTest = drive->rightPosition();
        //log->info("renc: %f", myTest);
        //myTest = drive->leftPosition();
        //log->info("lenc: %f", myTest);
        //log->info("gplh %f %f", control->gamepadLeftHorizontal(), cameraPivotAngle);
        //log->info("gplv %f %f", control->gamepadLeftVertical(), cameraElevateAngle);
        //log->info("gprh %f", control->gamepadRightHorizontal());
        //log->info("gprv %f", control->gamepadRightVertical());
        if (display()) {
    		//log->info("lg %d ldd %d", loading, loaderDisengageDetected);
            //log->info("BCD: %d", bcd->value());
    		log->info("SHV: %f", shooterMotorVolts);
    		//log->info("LdCDS: %d %d %d", loadCount, loaderDisengageDetected, loaderSwitch->Get());
    		
    		log->info("ena: %c%c%c%c",
    				" L"[control->gamepadToggleButton(1)],
    				" R"[control->gamepadToggleButton(3)],
    				" J"[control->gamepadToggleButton(2)],
					" S"[control->gamepadToggleButton(4)]);
            log->info("LRC %f %f",
        			leftClimber->encoder->GetDistance(),
        			rightClimber->encoder->GetDistance());
            log->info("LRD %f %f",
            		leftDriveEncoder->GetDistance(),
            		rightDriveEncoder->GetDistance());
        	log->info("LRL %d %d %d %d %d %d %d",
        		leftClimber->lowerLimitSwitch->Get(),
        		leftClimber->lowerHookSwitch->Get(),
        		leftClimber->upperHookSwitch->Get(),
        		rightClimber->lowerLimitSwitch->Get(),
        		rightClimber->lowerHookSwitch->Get(),
        		rightClimber->upperHookSwitch->Get(),
        		loaderSwitch->Get());
        	log->print();
        }
        
    }

};


START_ROBOT_CLASS(Robot);

#if 0
#include <iostream>
using namespace std;
void ClimbPeriodic(std::string s, float f) {
    static Robot* robot = NULL;
    if (!robot) {
        robot = new Robot;
        robot->init();
    }
    // Update the sensors
    if (s=="re") {
        robot->rightClimber->encoder->Set(f);
    } else if (s=="le") {
        robot->leftClimber->encoder->Set(f);
    } else if (s=="rls") {
        robot->rightClimber->lowerHookSwitch->Set(f != 0);
    } else if (s=="rus") {
        robot->rightClimber->upperHookSwitch->Set(f != 0);
    } else if (s=="lls") {
        robot->leftClimber->lowerHookSwitch->Set(f != 0);
    } else if (s=="lus") {
        robot->leftClimber->upperHookSwitch->Set(f != 0);
    } else {
        std::cout << "unrecognized sensor: " << s << "\n";
        return;
    }

    // Call the periodic function
    robot->ClimbPeriodic();
}
#endif
