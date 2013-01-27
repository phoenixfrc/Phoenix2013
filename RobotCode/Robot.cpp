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
	Encoder* lEncoder;
	Encoder* rEncoder;
	double goalDistance;
	bool startingState;

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
		Finished
	};
	ClimbState climbState;
	static const char* ClimbStateString(ClimbState climbState) {
		static const char* climbStateString[] = {
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
			"Finished"
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
		
		control = new Control(
				new Joystick(1), new Joystick(2), new Joystick(3), 
				Control::Tank, log);
		control->setLeftScale(-1);
		control->setRightScale(-1);
		control->setGamepadScale(-1);
		
		rEncoder = new Encoder(1, 2, false);
		lEncoder = new Encoder(3, 4, false);
	}
	
	void init() {
		lEncoder->Start();
		rEncoder->Start();
	}
	
	void AutonomousInit() {
		//int value = bcd->value();
		init();
		goalDistance = 120;
	}
	
	void AutonomousPeriodic() { 
		// TODO - rencoder is returning negative values - needs
		// to be inverted
		double currentDist = lEncoder->Get() / TicksPerInch;
		double remainingMoveDist = goalDistance - currentDist;
		if(remainingMoveDist>0){
			drive->setLeft(.6);
		    drive->setRight(.5);
		} else{
			drive->setLeft(0);
			drive->setRight(0);
		}
		double dist;
		dist = rEncoder->Get() / TicksPerInch;
		log->info("renc in inches: %f\n", dist);
		dist = lEncoder->Get() / TicksPerInch;
		log->info("lenc in inches: %f\n", dist);
		log->print();
	}

	void AutonomousDisabled() {
		//delete autonomous;
	}

	void setClimbState(ClimbState newState) {
		climbState = newState;
		startingState = true;
		log->info("new state: %s", newState);
		log->print();
	}
	
	void ClimbPeriodic() {
		// depending on state, cont
		switch (climbState) {
		case NotInitialized: {
			// climbers not yet deployed at all
			//initializeClimber();	// Start up both the left and right climbing mechanisms
			setClimbState(Initializing);
			break; }
		case Initializing: {
			// moving climbers into initial position
			/*if (leftClimberEncoderDistance() >= initialDistance)
				leftClimberMotor.stop();
			if (rightClimberEncoderDistance() >= initialDistance )
				rightClimberMotor.stop();
			if (leftClimberEncoderDistance() < initialDistance ||
					rightClimberEncoderDistance() < initialDistance )
				return; */
			setClimbState(DeployingJack);
			break; }
		case DeployingJack: {
			if (startingState) {
				// set motors, etc.
				startingState = false;
			}
			if (0 /*!endCondition*/)
				return;
			// turn off motors, etc., that were enabled
			setClimbState(InitialGrab);
			break; }
		case InitialGrab: {
			// Pulling both arms down enough to latch (low power consumption)
			if (startingState) {
				// set motors, etc.
				startingState = false;
			}
			if (0 /*!endCondition*/)
				return;
			// turn off motors, etc., that were enabled
			setClimbState(InitialLift);
			break; }
		case InitialLift: {
			// Continue pulling, but now high power consumption
			if (startingState) {
				// set motors, etc.
				startingState = false;
			}
			if (0 /*!endCondition*/)
				return;
			// turn off motors, etc., that were enabled
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

		myTest = rEncoder->Get();
		log->info("renc: %d\n", myTest);
		myTest = lEncoder->Get();
		log->info("lenc: %d\n", myTest);
		log->print();
		
	}
};

START_ROBOT_CLASS(Robot);

