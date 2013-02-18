#include "WPILib.h"
#include "Robot.h"
#include "Control.h"
#include "Drive.h"
#include "Encoder.h"
#include "Log.h"

#define DriveTicksPerInch 18.1

// This is not a real value!  Just a temporary definition
#define ClimberTicksPerInch 1

// This is not a real value!  Just a temporary definition
#define JackTicksPerInch 1

static const int DisplayInterval = 50;
static int displayCount = 0;
static bool display() {
	return displayCount % DisplayInterval == 0;
}

class BcdSwitch {
    DigitalInput *_port[4];
public:
    BcdSwitch(DigitalInput* port1, DigitalInput* port2,
    				DigitalInput* port3, DigitalInput* port4) {
        this->_port[0] = port1;
        this->_port[1] = port2;
        this->_port[2] = port3;
        this->_port[3] = port4;
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
        Initializing,    // moving climbers into initial position
        DeployingJack,    //
        InitialGrab,     // Pulling both arms down enough to latch (low power consumption)
        InitialLift,    // Continue pulling, but now high power consumption
        // The follow series repeats
        MoveArmUpToMiddle,
        GrabMiddle,
        PullUpToMiddle,
        MoveArmUpToTop,
        GrabTop,
        PullUpToTop,
        FinalLift,
        FinalShooting,
        Finished,
        Abort,
            ClimbStateLast
    };
    ClimbState climbState;
    static const char* ClimbStateString(ClimbState climbState) {
        static const char* climbStateString[ClimbStateLast] = {
            "NotInitialized",
            "Initializing",
            "DeployingJack",
            "InitialGrab",
            "InitialLift",
            "MoveArmUpToMiddle",
            "GrabMiddle",
            "PullUpToMiddle",
            "MoveArmUpToTop",
            "GrabTop",
            "PullUpToTop",
            "FinalLift",
            "FinalShooting",
            "Finished",
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
    // Generalize this to "Actuator" or something that would also cover the Jack?
    class ControlledMotor{
    public:
    	Robot* robot;
        const char* name;
        CANJaguar* motor;
        Encoder* encoder;
        float ticksPerInch;
        DigitalInput* lowerLimitSwitch;
        PIDController motorController;
        ControlledMotor(
        		Robot* r,
				const char* n,
				CANJaguar* m,
				Encoder* e,
				float t,
				DigitalInput* l) :
			robot(r), name(n), motor(m), encoder(e),
			ticksPerInch(t),lowerLimitSwitch(l),
			motorController(.1, .001, 0.0, encoder, motor)
		{}

        bool UpdateState(
                float powerLevel,    //typically 1.0 for forward -1.0 for backward
                float targetPosition)
        {
            if (powerLevel < 0.0 && !lowerLimitSwitch->Get()){
            	motorController.Disable();
            	motor->Set(0.0);
            	encoder->Reset();
            	encoder->Start();
            	return true;
            }
            if (!motorController.IsEnabled())
            	motorController.Enable();
            motorController.SetSetpoint(targetPosition);    
            
            float position = encoder->Get() / ticksPerInch;
            if (targetPosition -0.5 <= position && position <= targetPosition + 0.5 ) {
                motorController.Disable();
            	motor->Set(0.0);
                return true;
            }
            return false;
        }
    };
    class Jack: public ControlledMotor {
	public: 
    	Jack(
    		Robot* r,
			const char* n,
			CANJaguar* m,
			Encoder* e,
			float t,
			DigitalInput* l) :
				ControlledMotor(r,n,m,e,t,l)
	{}
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
                float t,
                DigitalInput* l,
                DigitalInput* lowerHook,
                DigitalInput* upperHook) :
                	ControlledMotor(r,n,m,e,t,l), lowerHookSwitch(lowerHook), upperHookSwitch(upperHook)
        {}
		bool UpdateState(
				float powerLevel,    //typically 1.0 for forward -1.0 for backward
				float targetPosition,
				HookSwitch hookSwitch=NoHookSwitch)
		{
			 if (powerLevel < 0.0 && !lowerLimitSwitch->Get()){
				motorController.Disable();
				motor->Set(0.0);
            	encoder->Reset();
				encoder->Start();
				return true;
			}
			if (!motorController.IsEnabled())
				motorController.Enable();
			motorController.SetSetpoint(targetPosition);    
			
			float position = encoder->Get() / ClimberTicksPerInch;
			if ( (hookSwitch == UpperHookSwitch && !upperHookSwitch->Get())||
				 (hookSwitch == LowerHookSwitch && !lowerHookSwitch->Get())||
				 (hookSwitch == NoHookSwitch &&
							targetPosition -0.5 <= position && position <= targetPosition + 0.5 ) ) {
				motorController.Disable();
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
   
    Compressor *compressor;
    
    Climber* climber;
    Climber* leftClimber;
    Climber* rightClimber;
    
    Jack* jack;

    Relay* loaderMotor;
    DigitalInput* loaderSwitch;
    int loaderCount;

    CANJaguar* shooterMotor;
    
    Relay* blowerMotor;

    double goalDistance;
    bool startingState;
    
    Servo* cameraPivotMotor;
    Servo* cameraElevateMotor;
    float cameraElevateAngle;
    float cameraPivotAngle;
    
    Relay* lightRing;
public:
    Robot() {
        log = new Log(this);

/*      bcd = new BcdSwitch(new DigitalInput(2,11), new DigitalInput(2,12),
        					new DigitalInput(2,13), new DigitalInput(2,14));
        bcdValue = bcd->value();
*/
        control = new Control(
                new Joystick(1), new Joystick(2), new Joystick(3), 
                Control::Tank, log);
        control->setLeftScale(-1);
        control->setRightScale(-1);
        control->setGamepadScale(-1);

        drive = new Drive(2, this);
        drive->addMotor(Drive::Left, 2, 1);
        drive->addMotor(Drive::Left, 3, 1);
        drive->addMotor(Drive::Right, 4, -1);
        drive->addMotor(Drive::Right, 5, -1);

        rightDriveEncoder = new Encoder(1,1, 1,2, true, Encoder::k2X);
        leftDriveEncoder = new Encoder(1,3, 1,4, false, Encoder::k2X);
        
        compressor = new Compressor(2,9, 2,3); //press, relay
		compressor->Start();

        leftClimber = new Climber(
        	this,
            "leftClimber",
            new CANJaguar(6),
            new Encoder(1,5, 1,6, false),
            ClimberTicksPerInch,
            new DigitalInput(2,1),//lower limit switch
            new DigitalInput(2,2),//lower hook switch
            new DigitalInput(2,3) );//upper hook switch
        
        rightClimber = new Climber(
        	this,
            "rightClimber",
            new CANJaguar(7),
            new Encoder(1,7, 1,8, false),
            ClimberTicksPerInch,
            new DigitalInput(2,4),//lower limit switch
			new DigitalInput(2,5),//lower hook switch
			new DigitalInput(2,6) );//upper hook switch	

        jack = new Jack(
        	this,
        	"jack",
        	new CANJaguar(8),
        	new Encoder(1,9, 1,10, false),
        	JackTicksPerInch,
        	new DigitalInput(2,7));//lower limit switch);

        loaderMotor = new Relay(2,1);
        loaderSwitch = new DigitalInput(2,8);

        shooterMotor = new CANJaguar(10);
        
        blowerMotor = new Relay(2,2);

        cameraPivotMotor = new Servo(1,9);
        cameraElevateMotor = new Servo(1,10);

        lightRing = new Relay(2,4);
    }
    
    void init() {
        climber = NULL;
        leftDriveEncoder->Reset();
        leftDriveEncoder->Start();
		rightDriveEncoder->Reset();
        rightDriveEncoder->Start();
		leftClimber->encoder->Reset();
        leftClimber->encoder->Start();
		rightClimber->encoder->Reset();
        rightClimber->encoder->Start();
		jack->encoder->Reset();
		jack->encoder->Start();
#if 0
		bool leftDone = false;
		bool rightDone = false;
		bool jackDone = false;
        //bcdValue = bcd->value();
        // Only do this for some BCD values?
        while (!leftDone || !rightDone || !jackDone){
			if (!leftDone)
				leftDone = leftClimber->UpdateState(-1.0, -30);
			if (!rightDone)
				rightDone = rightClimber->UpdateState(-1.0, -30);
			if (!jackDone)
				jackDone = jack->UpdateState(-1.0, -30);
	        log->info("Wait: Ll Rl jl: %d %d %d",
	        		leftClimber->lowerLimitSwitch->Get(),
	        		rightClimber->lowerLimitSwitch->Get(),
	        		jack->lowerLimitSwitch->Get());
	        log->print();
		}
#endif
        climbState = NotInitialized;
        cameraElevateAngle =
        		(cameraElevateMotor->GetMaxAngle()-cameraElevateMotor->GetMinAngle()) * 2/3;
        cameraPivotAngle = 0;
        cameraPivotMotor->SetAngle(cameraPivotAngle);
        cameraElevateMotor->SetAngle(cameraElevateAngle);
        loaderCount = 0;
    }

    void AutonomousInit() {
        init();
        goalDistance = 120;
    }
    
    void AutonomousPeriodic() { 
        double currentDist = drive->leftPosition() / DriveTicksPerInch;
        double remainingMoveDist = goalDistance - currentDist;
        if(remainingMoveDist>0){
            drive->setLeft(.6);
            drive->setRight(.5);
        } else{
            drive->setLeft(0);
            drive->setRight(0);
        }
        double dist;
        dist = drive->rightPosition() / DriveTicksPerInch;
        log->info("renc in inches: %f\n", dist);
        dist = drive->leftPosition() / DriveTicksPerInch;
        log->info("lenc in inches: %f\n", dist);
        log->print();
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
        jack->motor->Set(0.0);
    }
     
    void ClimbPeriodic() {
    	displayCount++;
        //amount to move past the bar to extend hook
        static const float CatchDistance = 6.0f;
        // This is the distance in inches the climber must be initially raised
        static const float InitialClimberPosition = 8.0f + CatchDistance; // Inches
        static const float LowerHookPosition = 17.f + CatchDistance;
        static const float UpperHookPosition = 17.f + CatchDistance;
        // This is the distance in inches the jack must be raised
        static const float JackPosition = 15.0f;
        // Distance to pull down on the grabber before it is expected to engage.
        // If it does NOT engage in this distance, that suggests something may be wrong
        // and we should abort.
        static const float ClimberGrabGuardDistance = CatchDistance + 2.0f;
        log->info("current state is: %s\n",ClimbStateString(climbState));
        log->print();
        // depending on state, cont
        switch (climbState) {
        case NotInitialized: {
            leftClimber->encoder->Start();
            rightClimber->encoder->Start();
            jack->encoder->Start();
            bar = LowerBar;
            climber = NULL;
            setClimbState(Initializing);
            break; }
        case Initializing: {
            bool leftDone = leftClimber->UpdateState(1.0, InitialClimberPosition);
            bool rightDone = rightClimber->UpdateState(1.0, InitialClimberPosition);
            startingState = false;
            // moving climbers into initial position
            if (leftDone && rightDone)
                setClimbState(DeployingJack);
            break; }
        case DeployingJack: {
            // moving jack into position
        	bool jackDone = jack->UpdateState(1.0, JackPosition);
			startingState = false;
            if (jackDone){
                setClimbState(InitialGrab);
            }
            break; }
        case InitialGrab: {
            // Pulling both arms down enough to latch (low power consumption)
            // We have several options here:
            // 1. Move a specific distance and assume it is right
            // 2. Move until the power goes up, and then check the encoders to see if the distance
            //    is plausible
            // 3. Move until a limit HookSwitch inside the grip for each climber trips
            bool leftDone = leftClimber->UpdateState(-1.0, InitialClimberPosition - ClimberGrabGuardDistance, UpperHookSwitch);
            bool rightDone = rightClimber->UpdateState(-1.0, InitialClimberPosition - ClimberGrabGuardDistance, UpperHookSwitch);
            startingState = false;
            // moving climbers into initial position
            if (leftDone && rightDone)
                setClimbState(InitialLift);
            break; }
        case InitialLift: {
            // Continue pulling, but now high power consumption
            //Note that this may run into the bottom hard
            bool leftDone = leftClimber->UpdateState(-1.0, 0, NoHookSwitch);
            bool rightDone = rightClimber->UpdateState(-1.0, 0, NoHookSwitch);
            startingState = false;
            // moving climbers into initial position
            if (leftDone && rightDone)
                setClimbState(MoveArmUpToMiddle);
            break; }
        case MoveArmUpToMiddle: {
            // moving a climber lower hook over the bar
            if (startingState) {
                // set motors, etc.
                if (climber != leftClimber){
                    climber = leftClimber;
                } else {
                    climber = rightClimber;    
                }                 
            }
            bool done = climber->UpdateState(1.0, LowerHookPosition);
            startingState = false;
            if (done)
                setClimbState(GrabMiddle);
            break; }
        case GrabMiddle: {
            bool done = climber->UpdateState(-1.0, LowerHookPosition - ClimberGrabGuardDistance, LowerHookSwitch);
            startingState = false;
            if (done){
                if (climber == leftClimber){
                    setClimbState(MoveArmUpToMiddle);
                    // this causes the control to return to MoveArmUpToMiddle for the right climber.
                }else{
                    setClimbState(PullUpToMiddle);
                }
            }    
            break; }
        case PullUpToMiddle: {
            bool leftDone = leftClimber->UpdateState(-1.0, 0);
            bool rightDone = rightClimber->UpdateState(-1.0, 0);
            startingState = false;
            // moving climbers into initial position
            if (leftDone && rightDone)
                setClimbState(MoveArmUpToTop);
            break; }
        case MoveArmUpToTop: {
            if (startingState) {
                // set motors, etc.
                if (climber != leftClimber){
                    climber = leftClimber;
                } else {
                    climber = rightClimber;    
                }                 
            }
            bool done = climber->UpdateState(1.0, UpperHookPosition);
            startingState = false;
            if (done)
                setClimbState(GrabTop);
            break; }
        case GrabTop: {
            bool done = climber->UpdateState(-1.0, UpperHookPosition - ClimberGrabGuardDistance, UpperHookSwitch);
            startingState = false;
            if (done){
                
                if (climber == leftClimber){
                    setClimbState(MoveArmUpToTop);
                }else{
                    setClimbState(PullUpToTop);
                }
            }
            break; }
        case PullUpToTop: {
            bool leftDone = leftClimber->UpdateState(-1.0, 0);
            bool rightDone = rightClimber->UpdateState(-1.0, 0);
            startingState = false;
            // moving climbers into Final Position
            if (leftDone && rightDone){
                if (bar == LowerBar){
                    bar = MiddleBar;
                    setClimbState(MoveArmUpToMiddle);
                }else{
                    setClimbState(FinalShooting);
                }
            }
            break; }
        case FinalShooting: {
            if (startingState) {
                // set motors, etc.
                startingState = false;
            }
            if (0 /*!endCondition*/)
                return;
            // turn off motors, etc., that were enabled
            setClimbState(Finished);
            break; }
        case Abort: {
            return;
            }
        default:
            log->info("unexpected climber state!");
            log->print();
        }
    }

    void TeleopInit() {
        init();
        drive->setShiftMode(Drive::Manual);
        
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
        drive->setLowShift(control->button(1)); // right trigger
        drive->setReversed(control->toggleButton(11)); // right JS button 11
        
        lightRing->Set(control->gamepadToggleButton(4) ? Relay::kForward : Relay::kOff );
       
        blowerMotor->Set(control->gamepadToggleButton(6) ? Relay::kOn : Relay::kOff );

        // For the loader, if we are rotating, wait for at least 100 counts before
        // checking the switch or adjusting motor power
        if (loaderCount) {
        	loaderCount--;
        	//log->info("down %d", loaderCount);
        } else {
        	// If already rotating, and the switch trips, power down the motor
        	if (!loaderSwitch->Get()) {
        		loaderMotor->Set(Relay::kOff);
            	//log->info("loader off");
            	// If not rotating and the gamepad button is set, start rotating
        	} else if (control->gamepadButton(7)) {
        		loaderMotor->Set(Relay::kForward);
        		loaderCount = 100;
            	//log->info("loader start");
        	} else if (control->gamepadButton(5)) {
        		loaderMotor->Set(Relay::kReverse);
        		loaderCount = 100;
            	//log->info("loader start");
        	} else {
            	//log->info("loader idle");
        	}
        }

        // For the shooter, spin it up or down based on the toggle
        //
        if (control->gamepadToggleButton(8)) {
        	shooterMotor->Set(1.0);
        	//log->info("shooter on");
        } else {
        	shooterMotor->Set(0.0);        	
        	//log->info("shooter off");
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
        	leftClimber->motor->Set(control->gamepadRightVertical());
        }
        if (control->gamepadToggleButton(3)) {
        	rightClimber->motor->Set(control->gamepadRightVertical());
        }
        if (control->gamepadToggleButton(2)) {
        	jack->motor->Set(control->gamepadRightVertical());
        }

        // assorted debug
        //log->info("Shift %s", control->toggleButton(8) 
        //        ? "low" : "high");
        //log->info("ArmPot: %.0f", arm->encoderValue());
        //log->info("pidf: %.2f", arm->pidFactor());
        //log->info("piden: %s", arm->isPidEnabled() ? "true" : "false");

        double myTest = drive->rightPosition();
        //log->info("renc: %f", myTest);
        myTest = drive->leftPosition();
        //log->info("lenc: %f", myTest);
        //log->info("gplh %f %f", control->gamepadLeftHorizontal(), cameraPivotAngle);
        //log->info("gplv %f %f", control->gamepadLeftVertical(), cameraElevateAngle);
        //log->info("gprh %f", control->gamepadRightHorizontal());
        //log->info("gprv %f", control->gamepadRightVertical());
        //log->info("BCD: %d", bcd->value());
        if (display()) {
        	log->info("LRJ %d %d %d",
        			leftClimber->encoder->Get(),
        			rightClimber->encoder->Get(),
        			jack->encoder->Get());
        	log->info("L3R3JL %d %d %d %d %d %d %d %d",
        		leftClimber->lowerLimitSwitch->Get(),
        		leftClimber->lowerHookSwitch->Get(),
        		leftClimber->upperHookSwitch->Get(),
        		rightClimber->lowerLimitSwitch->Get(),
        		rightClimber->lowerHookSwitch->Get(),
        		rightClimber->upperHookSwitch->Get(),
        		jack->lowerLimitSwitch->Get(),
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
    } else if (s=="je") {
        robot->jack->encoder->Set(f);
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
