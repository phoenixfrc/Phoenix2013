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
		// 		 port[2]->Get()<<2 |
		// 		 port[1]->Get()<<1 |
		//	     port[0]->Get();
	}

};

class Robot : public IterativeRobot {
	// Generalize this to "Actuator" or something that would also cover the Jack?
	struct Climber {
		CANJaguar* motor;
		Encoder* encoder;
		// Make into an array indexed by the enum, and add upper and lower limit switches?
		AnalogIOButton* upperHookSwitch;
		AnalogIOButton* lowerHookSwitch;
		const char* name;
		float climberStart;
		// Add constructor?
	};
	Climber* climber;
	// make these pointers?
	Climber leftClimber;
	Climber rightClimber;
		
		
	Drive* drive;
	Control* control;
	BcdSwitch* bcd;
	Log* log;
	Encoder* leftDriveEncoder;
	Encoder* rightDriveEncoder;
	double goalDistance;
	bool startingState;
	Encoder* jackEncoder;
	CANJaguar *jackMotor;
	
	enum HookSwitch {
		NoHookSwitch,
		UpperHookSwitch,
		LowerHookSwitch
	};
	
	enum ClimbState {
		NotInitialized,	// climbers not yet deployed at all
		Initializing,	// moving climbers into initial position
		DeployingJack,	//
		InitialGrab, 	// Pulling both arms down enough to latch (low power consumption)
		InitialLift,	// Continue pulling, but now high power consumption
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
	
public:
	Robot() {
		log = new Log(this);
		
		bcd = new BcdSwitch(11, 12, 13, 14);
		
		drive = new Drive(2, this);
		drive->addMotor(Drive::Left, 2, 1);
		drive->addMotor(Drive::Left, 3, 1);
		drive->addMotor(Drive::Right, 4, -1);
		drive->addMotor(Drive::Right, 5, -1);

		rightDriveEncoder = new Encoder(1, 2, true);
		leftDriveEncoder = new Encoder(3, 4, false);
		
		leftClimber.motor = new CANJaguar(6);
		leftClimber.encoder = new Encoder(5, 6, false);
		leftClimber.lowerHookSwitch= new AnalogIOButton(0);
		leftClimber.upperHookSwitch= new AnalogIOButton(1);
		rightClimber.motor = new CANJaguar(7);
		rightClimber.encoder = new Encoder(7, 8, false);
		rightClimber.lowerHookSwitch= new AnalogIOButton(2);
		rightClimber.upperHookSwitch= new AnalogIOButton(3);
		
		jackMotor = new CANJaguar(8);
		jackEncoder = new Encoder(9, 10, false);

		control = new Control(
				new Joystick(1), new Joystick(2), new Joystick(3), 
				Control::Tank, log);
		control->setLeftScale(-1);
		control->setRightScale(-1);
		control->setGamepadScale(-1);
		
	}
	
	void init() {
		leftDriveEncoder->Start();
		rightDriveEncoder->Start();
	}
	
	void AutonomousInit() {
		//int value = bcd->value();
		init();
		goalDistance = 120;
	}
	
	void AutonomousPeriodic() { 
		double currentDist = leftDriveEncoder->Get() / TicksPerInch;
		double remainingMoveDist = goalDistance - currentDist;
		if(remainingMoveDist>0){
			drive->setLeft(.6);
		    drive->setRight(.5);
		} else{
			drive->setLeft(0);
			drive->setRight(0);
		}
		double dist;
		dist = rightDriveEncoder->Get() / TicksPerInch;
		log->info("renc in inches: %f\n", dist);
		dist = leftDriveEncoder->Get() / TicksPerInch;
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
		leftClimber.motor->Set(0.0);
		rightClimber.motor->Set(0.0);
		jackMotor->Set(0.0);
	}

		
	bool UpdateState(
			Climber* climber,
			float powerLevel,	//typically 1.0 for forward -1.0 for backward
			HookSwitch hookSwitch,
			float maxDistance)
	{
		if(startingState){
			// set motors, etc.
			climber->climberStart = climber->encoder->Get() / ClimberTicksPerInch;
			climber->motor->Set(powerLevel);	
		}
		float distance = climber->encoder->Get() / ClimberTicksPerInch - climber->climberStart;
		if (distance < 0) distance = -distance;
		if ( (hookSwitch == UpperHookSwitch && climber->upperHookSwitch->Get())||
			 (hookSwitch == LowerHookSwitch && climber->lowerHookSwitch->Get())||
			 (hookSwitch == NoHookSwitch && distance >= maxDistance) ) {
			climber->motor->Set(0.0);
			return true;
		} else if (hookSwitch != NoHookSwitch && distance >= maxDistance) {
			AbortClimb("Grab did not occur when expected");
			return false;
		} // else if (hookSwitch != NoHookSwitch && Check motor power draw)
		// abort here if looking for switch and started drawing a lot of power 
		return false;
	}

	
	// Compress further to this?  The ClimberPeriod then becomes an interpreter.
	// This would use absolute rather than relative positions
	/*
	// Up or down is determined relative to goal position.  Default is NoTrigger
	static Action actions[] = {
		{ "MoveInitial",	{ { leftClimber,  InitialPos }, 
							  { rightClimber, InitialPos } } },
		{ "DeployJack",		{ { jack,         JackDeployedPos } } },
		{ "Grab",			{ { leftClimber,  InitialGrabPos,      UpperTrigger }, 
							  { rightClimber, InitialGrabPos,      UpperTrigger } } },
		{ "LBeforeMidLow",  { { leftClimber,  MiddleHookBeforeGrab } } },
		{ "LGrabMidLow",    { { leftClimber,  MiddleHookAfterGrab, MiddleTrigger } } },
		{ "RBeforeMidLow",  { { rightClimber, MiddleHookBeforeGrab } } },
		{ "RGrabMidLow",	{ { rightClimber, MiddleHookAfterGrab, MiddleTrigger } } },
		�

		NULL };

	currentAction = �
	if (currentAction->done) {
	}
	 */
	
	void ClimbPeriodic() {
		//amount to move past the bar to extend hook
		static const float CatchDistance = 6.0;
		// This is the distance in inches the climber must be initially raised
		static const float InitialClimberDistance = 8.0 + CatchDistance; // Inches
		static const float LowerHookDistance = 17 + CatchDistance;
		static const float UpperHookDistance = 17 + CatchDistance;
		// This is the distance in inches the jack must be raised
		static const float JackDistance = 15.0;
		// Distance to pull down on the grabber before it is expected to engage.
		// If it does NOT engage in this distance, that suggests something may be wrong
		// and we should abort.
		static const float ClimberGrabGuardDistance = CatchDistance + 2.0;
		
		// depending on state, cont
		switch (climbState) {
		case NotInitialized: {
			leftClimber.encoder->Start();
			rightClimber.encoder->Start();
			jackEncoder->Start();
			bar = LowerBar;
			climber = NULL;
			setClimbState(Initializing);
			break; }
		case Initializing: {
			bool leftDone = UpdateState(&leftClimber,1.0, NoHookSwitch, InitialClimberDistance);
			bool rightDone = UpdateState(&rightClimber,1.0, NoHookSwitch, InitialClimberDistance);
			startingState = false;
			// moving climbers into initial position
			if (leftDone && rightDone)
				setClimbState(DeployingJack);
			break; }
		case DeployingJack: {
			// moving jack into position
			if (startingState) {
				// set motors, etc.
				jackMotor->Set(1.0);
				startingState = false;
			}
			float jackDistance = jackEncoder->Get() / JackTicksPerInch;
			if (jackDistance >= JackDistance){
				jackMotor->Set(0.0);
				// turn off motors, etc., that were enabled
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
			bool leftDone = UpdateState(&leftClimber,-1.0, UpperHookSwitch, ClimberGrabGuardDistance);
			bool rightDone = UpdateState(&rightClimber,-1.0, UpperHookSwitch, ClimberGrabGuardDistance);
			startingState = false;
			// moving climbers into initial position
			if (leftDone && rightDone)
				setClimbState(InitialLift);
			break; }
		case InitialLift: {
			// Continue pulling, but now high power consumption
			//Note that this may run into the bottom hard
			bool leftDone = UpdateState(&leftClimber,-1.0, NoHookSwitch, leftClimber.encoder->Get() / ClimberTicksPerInch);
			bool rightDone = UpdateState(&rightClimber,-1.0, NoHookSwitch, rightClimber.encoder->Get() / ClimberTicksPerInch);
			startingState = false;
			// moving climbers into initial position
			if (leftDone && rightDone)
				setClimbState(MoveArmUpToMiddle);
			break; }
		case MoveArmUpToMiddle: {
			// moving a climber lower hook over the bar
			if (startingState) {
				// set motors, etc.
				if (climber != &leftClimber){
					climber = &leftClimber;
				} else {
					climber = &rightClimber;	
				} 				
			}
			bool done = UpdateState(climber,1.0, NoHookSwitch, LowerHookDistance);
			startingState = false;
			if (done)
				setClimbState(GrabMiddle);
			break; }
		case GrabMiddle: {
			bool done = UpdateState(climber,-1.0, LowerHookSwitch, ClimberGrabGuardDistance);
			startingState = false;
			if (done){
				if (climber == &leftClimber){
					setClimbState(MoveArmUpToMiddle);
					// this causes the control to return to MoveArmUpToMiddle for the right climber.
				}else{
					setClimbState(PullUpToMiddle);
				}
			}	
			break; }
		case PullUpToMiddle: {
			bool leftDone = UpdateState(&leftClimber,-1.0, NoHookSwitch, leftClimber.encoder->Get() / ClimberTicksPerInch);
			bool rightDone = UpdateState(&rightClimber,-1.0, NoHookSwitch, rightClimber.encoder->Get() / ClimberTicksPerInch);
			startingState = false;
			// moving climbers into initial position
			if (leftDone && rightDone)
				setClimbState(MoveArmUpToTop);
			break; }
		case MoveArmUpToTop: {
			if (startingState) {
				// set motors, etc.
				if (climber != &leftClimber){
					climber = &leftClimber;
				} else {
					climber = &rightClimber;	
				} 				
			}
			bool done = UpdateState(climber,1.0, NoHookSwitch, UpperHookDistance);
			startingState = false;
			if (done)
				setClimbState(GrabTop);
			break; }
		case GrabTop: {
			bool done = UpdateState(climber,-1.0, UpperHookSwitch, ClimberGrabGuardDistance);
			startingState = false;
			if (done){
				
				if (climber == &leftClimber){
					setClimbState(MoveArmUpToTop);
				}else{
					setClimbState(PullUpToTop);
				}
			}
			break; }
		case PullUpToTop: {
			bool leftDone = UpdateState(&leftClimber,-1.0, NoHookSwitch, leftClimber.encoder->Get() / ClimberTicksPerInch);
			bool rightDone = UpdateState(&rightClimber,-1.0, NoHookSwitch, rightClimber.encoder->Get() / ClimberTicksPerInch);
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
		climbState = NotInitialized;
	}

	void TeleopPeriodic() {
		int myTest;
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

		// assorted debug
		//log->info("Shift %s", control->toggleButton(8) 
		//		? "low" : "high");
		//log->info("ArmPot: %.0f", arm->encoderValue());
		//log->info("pidf: %.2f", arm->pidFactor());
		//log->info("piden: %s", arm->isPidEnabled() ? "true" : "false");

		myTest = rightDriveEncoder->Get();
		log->info("renc: %d\n", myTest);
		myTest = leftDriveEncoder->Get();
		log->info("lenc: %d\n", myTest);
		log->print();
		
	}
};

START_ROBOT_CLASS(Robot);

