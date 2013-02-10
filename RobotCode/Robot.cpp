#include "WPILib.h"
#include "Robot.h"
#include "Control.h"
#include "Drive.h"
#include "Encoder.h"
#include "Log.h"

class BcdSwitch {
    DigitalInput *_port[4];
public:
    BcdSwitch(int port1, int port2, int port3, int port4) {
        this->_port[0] = new DigitalInput(port1);
        this->_port[1] = new DigitalInput(port2);
        this->_port[2] = new DigitalInput(port3);
        this->_port[3] = new DigitalInput(port4);
    }
    int value() {
        int ret = 0;
        for(int i = 0; i < 4; ++i) {
            if (_port[i]->Get())
                ret |= 1<<i;
        }
        return ret;
        // Alternative:
        //return port[3]->Get()<<3 |
        //          port[2]->Get()<<2 |
        //          port[1]->Get()<<1 |
        //         port[0]->Get();
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
        AnalogIOButton* lowerLimitSwitch;
        PIDController motorController;
        ControlledMotor(
        		Robot* r,
				const char* n,
				CANJaguar* m,
				Encoder* e,
				float t,
				AnalogIOButton* l) :
			robot(r), name(n), motor(m), encoder(e),
			ticksPerInch(t),lowerLimitSwitch(l),
			motorController(.1, .001, 0.0, encoder, motor)
		{}

        bool UpdateState(
                float powerLevel,    //typically 1.0 for forward -1.0 for backward
                float targetPosition)
        {
            if (powerLevel < 0.0 && lowerLimitSwitch->Get()){
            	motorController.Disable();
            	motor->Set(0.0);
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
			AnalogIOButton* l) :
				ControlledMotor(r,n,m,e,t,l)
	{}
    };
    class Climber: public ControlledMotor {
	public:
		AnalogIOButton* upperHookSwitch;
		AnalogIOButton* lowerHookSwitch;
		Climber (
                Robot* r,
                const char* n,
                CANJaguar* m,
                Encoder* e,
                float t,
                AnalogIOButton* l,
                AnalogIOButton* upperHook,
                AnalogIOButton* lowerHook) :
                	ControlledMotor(r,n,m,e,t,l), upperHookSwitch(upperHook), lowerHookSwitch(lowerHook)
        {}
		bool UpdateState(
				float powerLevel,    //typically 1.0 for forward -1.0 for backward
				float targetPosition,
				HookSwitch hookSwitch=NoHookSwitch)
		{
			 if (powerLevel < 0.0 && lowerLimitSwitch->Get()){
				motorController.Disable();
				motor->Set(0.0);
				encoder->Start();
				return true;
			}
			if (!motorController.IsEnabled())
				motorController.Enable();
			motorController.SetSetpoint(targetPosition);    
			
			float position = encoder->Get() / ClimberTicksPerInch;
			if ( (hookSwitch == UpperHookSwitch && upperHookSwitch->Get())||
				 (hookSwitch == LowerHookSwitch && lowerHookSwitch->Get())||
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
    Climber* climber;
    Climber* leftClimber;
    Climber* rightClimber;
    Jack* jack;

    Drive* drive;
    Control* control;
    BcdSwitch* bcd;
    int bcdValue;

    Log* log;
    Encoder* leftDriveEncoder;
    Encoder* rightDriveEncoder;
    double goalDistance;
    bool startingState;
    Servo* cameraPivotMotor;
    Servo* cameraElevateMotor;
public:
    Robot() {
        log = new Log(this);

        drive = new Drive(2, this);
        drive->addMotor(Drive::Left, 2, 1);
        drive->addMotor(Drive::Left, 3, 1);
        drive->addMotor(Drive::Right, 4, -1);
        drive->addMotor(Drive::Right, 5, -1);

        rightDriveEncoder = new Encoder(1, 2, true);
        leftDriveEncoder = new Encoder(3, 4, false);

        leftClimber = new Climber(
        	this,
            "leftClimber",
            new CANJaguar(6),
            new Encoder(5, 6, false),
            ClimberTicksPerInch,
            new AnalogIOButton(0),//lower limit switch
            new AnalogIOButton(1),//lower hook switch
            new AnalogIOButton(2) );//upper hook switch
        
        rightClimber = new Climber(
        	this,
            "rightClimber",
            new CANJaguar(7),
            new Encoder(7, 8, false),
            ClimberTicksPerInch,
            new AnalogIOButton(3),//lower limit switch
			new AnalogIOButton(4),//lower hook switch
			new AnalogIOButton(5) );//upper hook switch	

        jack = new Jack(
        	this,
        	"jack",
        	new CANJaguar(8),
        	new Encoder(9, 10, false),
        	JackTicksPerInch,
        	new AnalogIOButton(6));//lower limit switch);
        
        bcd = new BcdSwitch(11, 12, 13, 14);
        bcdValue = bcd->value();

        control = new Control(
                new Joystick(1), new Joystick(2), new Joystick(3), 
                Control::Tank, log);
        control->setLeftScale(-1);
        control->setRightScale(-1);
        control->setGamepadScale(-1);
        // Pwm ports
        // Spike on 0
        // Spike on 1
        // Spike on 2
        // Spike on 3
        cameraPivotMotor = new Servo(9);
        cameraElevateMotor = new Servo(10);
    }
    
    void init() {
    	leftDriveEncoder->Start();
		rightDriveEncoder->Start();
		leftClimber->encoder->Start();
		rightClimber->encoder->Start();
		jack->encoder->Start();
		bool leftDone = false;
		bool rightDone = false;
		bool jackDone = false;
		while (!leftDone || !rightDone || !jackDone){
			leftDone = leftClimber->UpdateState(-1.0, -30);
			rightDone = rightClimber->UpdateState(-1.0, -30);
			jackDone = jack->UpdateState(-1.0, -30);
		}
        climbState = NotInitialized;
        bcdValue = bcd->value();
    }

    void AutonomousInit() {
        init();
        goalDistance = 120;
    }
    
    void AutonomousPeriodic() { 
        double currentDist = drive->leftPosition() / TicksPerInch;
        double remainingMoveDist = goalDistance - currentDist;
        if(remainingMoveDist>0){
            drive->setLeft(.6);
            drive->setRight(.5);
        } else{
            drive->setLeft(0);
            drive->setRight(0);
        }
        double dist;
        dist = drive->rightPosition() / TicksPerInch;
        log->info("renc in inches: %f\n", dist);
        dist = drive->leftPosition() / TicksPerInch;
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
    	//tbs change button number
        if (control->button(2)|| climbState != NotInitialized) {
            ClimbPeriodic();
            return;
        }

        // drive
        drive->setLeft(control->left());
        drive->setRight(control->right());
        drive->setScale(control->throttle());
        drive->setReversed(control->toggleButton(11));
        drive->setLowShift(control->gamepadToggleButton(9));
        cameraPivotMotor->Set(control->gamepadLeft());
        cameraElevateMotor->Set(control->gamepadRight());
       
        // assorted debug
        //log->info("Shift %s", control->toggleButton(8) 
        //        ? "low" : "high");
        //log->info("ArmPot: %.0f", arm->encoderValue());
        //log->info("pidf: %.2f", arm->pidFactor());
        //log->info("piden: %s", arm->isPidEnabled() ? "true" : "false");

        double myTest = drive->rightPosition();
        log->info("renc: %f\n", myTest);
        myTest = drive->leftPosition();
        log->info("lenc: %f\n", myTest);
        log->print();
        
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
