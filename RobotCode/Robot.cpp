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
	Drive* drive;
	Control* control;
	BcdSwitch* bcd;
	Log* log;
	Encoder* leftDriveEncoder;
	Encoder* rightDriveEncoder;
	double goalDistance;
	bool startingState;
	Encoder* leftClimberEncoder;
	Encoder* rightClimberEncoder;
	CANJaguar *leftClimberMotor;
	CANJaguar *rightClimberMotor;
	Encoder* jackEncoder;
	CANJaguar *jackMotor;
	float leftClimberStart;
	float rightClimberStart;

	enum Climber {
		NoClimber,
		LeftClimber,
		RightClimber
	};
	Climber currentClimber;
	static const char* ClimberString(Climber climber) {
		static const char* climberString[] = {
			"NoClimber",
			"LeftClimber",
			"RightClimber"	
		};
		return climberString[climber];
	}
	enum ClimbState {
		NotInitialized,	// climbers not yet deployed at all
		Initializing,	// moving climbers into initial position
		DeployingJack,	//
		InitialGrab, 	// Pulling both arms down enough to latch (low power consumption)
		InitialLift,	// Continue pulling, but now high power consumption
		// The follow series repeats
		MoveArmUpToMiddle,
		GrabMiddle,
		MoveArmUpToNext,
		GrabTop,
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
			"MoveArmUpToNext",
			"GrabTop",
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
		
		leftClimberMotor = new CANJaguar(6);
		rightClimberMotor = new CANJaguar(7);
		leftClimberEncoder = new Encoder(5, 6, false);
		rightClimberEncoder = new Encoder(7, 8, false);

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
		leftClimberMotor->Set(0.0);
		rightClimberMotor->Set(0.0);
		jackMotor->Set(0.0);
	}
	
	void ClimbPeriodic() {
		// This is the distance in inches the climber must be initially raised
		static const float InitialClimberDistance = 8.0 + 6.0; // Inches
		// This is the distance in inches the jack must be raised
		static const float JackDistance = 15.0;
		// Distance to pull down on the grabber before it is expected to engage.
		// If it does NOT engage in this distance, that suggests something may be wrong
		// and we should abort.
		static const float ClimberGrabDistance = 8.0;
		
		// depending on state, cont
		switch (climbState) {
		case NotInitialized: {
			leftClimberEncoder->Start();
			rightClimberEncoder->Start();
			jackEncoder->Start();
			setClimbState(Initializing);
			break; }
		case Initializing: {
			if (startingState) {
				// set motors, etc.
				leftClimberMotor->Set(1.0);
				rightClimberMotor->Set(1.0);
				startingState = false;
			}
			// moving climbers into initial position
			float leftDistance = leftClimberEncoder->Get() / ClimberTicksPerInch;
			float rightDistance = rightClimberEncoder->Get() / ClimberTicksPerInch;
			if (leftDistance >= InitialClimberDistance)
				leftClimberMotor->Set(0.0);
			if (rightDistance >= InitialClimberDistance)
				rightClimberMotor->Set(0.0);
			if (leftDistance < InitialClimberDistance || rightDistance < InitialClimberDistance )
				return;
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
			if (jackDistance >= JackDistance)
				jackMotor->Set(0.0);
			if (jackDistance < JackDistance)
				return;
			// turn off motors, etc., that were enabled
			setClimbState(InitialGrab);
			break; }
		case InitialGrab: {
			// Pulling both arms down enough to latch (low power consumption)
			if (startingState) {
				// set motors, etc.
				leftClimberMotor->Set(-1.0);
				rightClimberMotor->Set(-1.0);
				// set motors, etc.
				startingState = false;
			}
			// We have several options here:
			// 1. Move a specific distance and assume it is right
			// 2. Move until the power goes up, and then check the encoders to see if the distance
			//    is plausible
			// 3. Move until a limit switch inside the grip for each climber trips
			float leftDistance = leftClimberEncoder->Get() / ClimberTicksPerInch;
			float rightDistance = rightClimberEncoder->Get() / ClimberTicksPerInch;
			if (0 /*leftClimberSwitch->???  is set */) {
				leftClimberMotor->Set(0.0);
			} else if (leftDistance >= ClimberGrabDistance) {
				AbortClimb("Left grab did not occur when expected");
				return;
			}
			if (0 /*rightClimberSwitch->??? is set */) {
				rightClimberMotor->Set(0.0);
			} else if (rightDistance >= ClimberGrabDistance) {
				AbortClimb("Right grab did not occur when expected");
				return;
			}
			if (leftDistance < ClimberGrabDistance || rightDistance < ClimberGrabDistance )
				return;
			setClimbState(InitialLift);
			break; }
		case InitialLift: {
			// Continue pulling, but now high power consumption
			
			if (startingState) {
				// set motors, etc.
				leftClimberStart = leftClimberEncoder->Get() / ClimberTicksPerInch;
				rightClimberStart = rightClimberEncoder->Get() / ClimberTicksPerInch;
				leftClimberMotor->Set(-1.0);
				rightClimberMotor->Set(-1.0);
				startingState = false;
			}//*** unfinished edit
			// moving climbers into initial position
			float leftDistance = leftClimberEncoder->Get() / ClimberTicksPerInch;
			float rightDistance = rightClimberEncoder->Get() / ClimberTicksPerInch;
			if (leftDistance >= InitialClimberDistance)
				leftClimberMotor->Set(0.0);
			if (rightDistance >= InitialClimberDistance)
				rightClimberMotor->Set(0.0);
			if (leftDistance < InitialClimberDistance || rightDistance < InitialClimberDistance )
				return;
			setClimbState(MoveArmUpToMiddle);
			break; }
		case MoveArmUpToMiddle: {
			if (startingState) {
				// set motors, etc.
				startingState = false;
			}
			if (0 /*!endCondition*/)
				return;
			// turn off motors, etc., that were enabled
			setClimbState(GrabMiddle);
			break; }
		case GrabMiddle: {
			if (startingState) {
				// set motors, etc.
				startingState = false;
			}
			if (0 /*!endCondition*/)
				return;
			// turn off motors, etc., that were enabled
			setClimbState(MoveArmUpToNext);
			break; }
		case MoveArmUpToNext: {
			if (startingState) {
				// set motors, etc.
				startingState = false;
			}
			if (0 /*!endCondition*/)
				return;
			// turn off motors, etc., that were enabled
			setClimbState(GrabTop);
			break; }
		case GrabTop: {
			if (startingState) {
				// set motors, etc.
				startingState = false;
			}
			if (0 /*!endCondition*/)
				return;
			// turn off motors, etc., that were enabled
			break; }
		case FinalLift: {
			if (startingState) {
				// set motors, etc.
				startingState = false;
			}
			if (0 /*!endCondition*/)
				return;
			// turn off motors, etc., that were enabled
			setClimbState(FinalShooting);
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
		if (control->button(2)) {
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

