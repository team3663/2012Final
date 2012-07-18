/***************************************************************************************************
 *           CEDAR PARK CHRISTIAN SCHOOLS - 2012 FIRST ROBOTICS COMPTETION "REBOUND RUMBLE"        *
 *        ____      ____       _____         __________     ___     __     _________    __   ___   *
 *       /    /    /    /    /       |      /    __    /   /   /   /  /   /        /   /  / /  /   *
 *      /    /____/    /   /   ___    |    /   /___/  /   /   /   /  /   /    ____/   /  / /  /    *
 *     /    _____     /  /    /__/     |  /    ___   /   /   /  /  /    /    /___    /  /_/  /     *
 *    /    /    /    /  /    ___      /  /    /   /  /  /   / /  /     /    _____/   |      /      *
 *   /    /    /    /  /    /   /    /  /    /   /  /   /  /_/  /     /    /____     /    /        *
 *  /____/    /____/  /____/   /____/  /____/   /__/   /______/      /__________/   /____/         *
 *                                      CODE VERSION 3.6.6.3 - FINAL                               *
 *  SOFTWARE MENTORS:                                                                              *
 *   -JON WIEDERSPAN & BRIAN KING                                                                  *
 *                                                                                                 *
 *  SOFTWARE TEAM:                                                                                 *
 *   -EVAN WIEDERSPAN, SEAN MUIR, SAWYER KNOBLICH, CASEY JONES, DANIEL POTTER                      *
 ***************************************************************************************************/

#include "WPILib.h"
#include "DashboardDataFormat.h"
#include "math.h"

#define WAIT_TIME .005   // global time to wait in loops
#define EXPIRATION 0.5   // global expiration on motor monitoring
#define SAFETY false     // global setting for set safety enabled

#define TOP_SHOOT_DRIVE_CLOSE 0.26     // top wheel speed for shot from bumper to middle basket
#define BOT_SHOOT_DRIVE_CLOSE -0.41    // bottom wheel speed ...
#define SHOOT_LOC_CLOSE		  40.0     // shoot from the bumper
#define TOP_SHOOT_DRIVE_KEY   0.30     // top wheel speed for shot from the key to middle basket
#define BOT_SHOOT_DRIVE_KEY  -0.45     // bottom wheel speed ...
#define SHOOT_LOC_KEY		  114.0    // shoot from the key
#define TOP_SHOOT_DRIVE_FAR   0.45     // top wheel speed to hurl the ball a long way with topspin
#define BOT_SHOOT_DRIVE_FAR  -0.55     // bottom wheel speed ...
#define SHOOT_LOC_FAR		  162.0    // shoot from the top of key
#define BRIDGE_LOC			  70.0     // distance back to push down bridge
#define GOOSE_LOOP_COUNT       15      // times to loop through when we "goose" the bridge mode
#define BACK_LOOP_COUNT        43	   // times to back up from bridge for back up button
#define PI 3.14159					   // needed for kinect?

/////////////////////////////////////////////
// JAGUARS
// 1,2 - wheel drives
// 6 - top shooter
// 5 - bottom shooter
//
// VICTORS
// 3 - Houser (bridge) arm
// 4 - Caster arm
// 7 - Sweeper drive
// 8 - Riser drive
//
// RELAYS
// 1 - unused (3/22/2012)
//
// SWITCHES DI/O
// 1 - unused
// 2 - unused
// 3 - unused
// 4 - Hybrid switch 1
// 5 - Hybrid switch 2
// 6 - Light sensor
//
// ADXL345
// 3, 4, 5 - Accelerometer x, y, z axis feeds
//
// ANALOG
// 1 - Ultrasonic (proximity sensor)
// 2 - Gyro Direction
// 3 - Gyro Temperature
//
// I2C
// 1 - accelerometer
//
//////////////////////////////////////////////////

AnalogModule  *g_analogModule1    = 0;

RobotDrive    *g_wheelDrive       = 0;            		// base wheel drives on Jaguars
Jaguar        *g_jaguars[2]       = {0, 0};       		// Shooting wheels top/bottom
Victor        *g_victors[5]       = {0, 0, 0, 0, 0}; 	// Houser, caster, sweeper, riser

Joystick      *g_joysticks[2]      = { 0, 0 };    		// drive and button joysticks

Relay		  *g_cmprsrRelay	  = 0;					// relay for the compressor
DigitalInput  *g_digitalInputs[4] = {0, 0, 0, 0}; 		// turret switch (UNUSED), hybrid mode indicator switches, light sensor
AnalogModule  *g_analogInputs[1]  = { 0 };  			// light sensor (UNUSED)
  
Solenoid      *g_solenoid[2]      = {0,0};				// solenoids for ball feed
DigitalModule *g_digitalModule1;						// used for reading hybrid switches

DriverStationLCD *g_driverstationlcd = 0;				// used to create print statements to driver station

bool  g_goose           = false; // controls "goose" mode (short bursts of speed)
bool  g_useCasters      = false; // should we put the caster wheels down
int   g_moveHouser      = 0;     // 0=stop, 1=down, 2= up slow, 3= up to hit casters hard
bool  g_houserIsDown    = false; // is the Houser arm already down?
bool  g_inBridgeMode    = false; // in mode to move onto bridge or not
float g_topShootDriveSpeed = TOP_SHOOT_DRIVE_CLOSE;  // initial speed of top shooter drive
float g_botShootDriveSpeed = BOT_SHOOT_DRIVE_CLOSE;  // initial speed of bottom shooter drive
int   g_shootState      = 1;     // start with shooters at CLOSE setting
int   g_runRiserDrive   = 0;     // riser motor: 0 = stop, 1 = up, -1 = down
int   g_runSweeperDrive = 0;     // sweeper motor: 0 = stop, 1 = in, -1 = out
bool  g_shooterWheelsOn = false; // should shooter wheels be turning
bool  g_solenoidsOff    = true;  // are the ball plate solenoids on or off (and which one means the plate is forward?

bool  g_turnPrintOn     = true;  // should the driver station LCD print statements be enabled

int  g_moveCaster      = 0;     			// start with no caster mode
int  backLoopCount     = BACK_LOOP_COUNT; 	// how much to drive back from bridge
int  g_shootBackLoopCount = 75;	 			// how much to drive back from bridge
bool g_back            = false; 			// are we backing up from bridge or not
bool g_brake           = false; 			// are we braking on bridge or not
bool g_shootBack       = false; 			// are we backing up from bumper or not


/**********************************
 *    THE CAKE IS A LIE!!!         * 
 **********************************/
		string cake = "lie";
/*        END THE LIE!            */

bool  g_keepRunning			   = true;  // use instead of "true" in Tasks - set to false at end of match

void ButtonCheckTask();			// loops through to check if a button is pressed
void MoveHouserTask();			// moves houser motor
void MoveCasterTask();			// raises/lowers the casters
void RunSweeperTask();          // move the sweeper motor
void RunRiserTask();            // move the riser motor
void RunShootersTask();         // push a ball in and shoot - allows for multiple consecutive shots
void ManageCompressorTask();	// regulates compressor pressure
void BrakeTask();				// causes drive motors to lock up (UNTESTED)

class DashboardDataExample : public SimpleRobot
{
	RobotDrive *m_wheelDrive;			// main drive motors
	Jaguar     *m_topShootDrive;		// top shooter Jaguar motor
	Jaguar     *m_botShootDrive;		// bottom shooter Jaguar motor
	Victor     *m_houserDrive;			// houser bride lowering arm Victor motor
	Victor     *m_casterDrive;			// victor motor for lowering caster wheels
	Victor     *m_riserDrive;			// victor for running the ball riser
	Victor     *m_sweeperDrive;			// victor for running the ball sweeper
	Relay      *m_cmprsrRelay;	      	// spike relay for compressor power
	
	Joystick   *m_driveStick;			// main joystick for driving
	Joystick   *m_buttonStick;			// secondary joystick for buttons
	
	DigitalInput  *m_hybridSwitch1;		// switch to allow changing of hybrid modes
	DigitalInput  *m_hybridSwitch2;		// switch to allow changing of hybrid modes
	DigitalInput  *m_ballCountSensor;	// light sensor for checking if a ball is in the robot (UNTESTED)
	AnalogChannel *m_ultrasonic;		// ultrasonic sensor for measuring distance
	
	Task *m_buttonCheckTask;			// checks for button presses
	Task *m_moveHouserTask;				// moves the houser
	Task *m_moveCasterTask;				// rasies/lowers casters
	Task *m_runSweeperTask;				// runs sweeper
	Task *m_runRiserTask;				// runs riser
	Task *m_manageCompressorTask;   	// manages compressor pressure
	Task *m_RunShootersTask;			// runs shooters
	Task *m_brakeTask;					// locks drive motors
	
	Solenoid *m_solenoidA, *m_solenoidB;// solenoids for controlling ball feed
	
	DriverStationLCD    *m_driverstationlcd;	// prints statements back to driver station
	DashboardDataFormat dashboardDataFormat;	// feeds input back to driver station
	DigitalModule       *digitalModuleDash;
	
public:
	DashboardDataExample(void)
	{
		digitalModuleDash = DigitalModule::GetInstance(1);	// reads hybrid switches
		
		GetWatchdog().SetExpiration(0.1);					// sets expiration time on watchdog
				
		m_solenoidA     = new Solenoid(8);					// solenoid for retracting ball feed panel
		m_solenoidB     = new Solenoid(1);					// solenoid for extending ball feed panel (UNUSED)
		m_cmprsrRelay   = new Relay(4);						// relay that controls compressor power

		m_wheelDrive    = new RobotDrive(1, 2);				// drive motors
		m_houserDrive   = new Victor(3);					// motor for houser arm
		m_casterDrive   = new Victor(4);					// motor for casters

		m_topShootDrive = new Jaguar(6);					// motor for top shoot wheels
		m_botShootDrive = new Jaguar(5);					// motor for bottom shoot wheels

		m_sweeperDrive  = new Victor(7);					// motor for sweeper
		m_riserDrive    = new Victor(8);					// motor for riser

		m_driveStick    = new Joystick(1);					// joystick for driving
		m_buttonStick   = new Joystick(2);					// joystick for buttons

		m_ultrasonic    = new AnalogChannel(5);				// ultrasonic for finding distance

		m_hybridSwitch1   = new DigitalInput(4);			// switch for changing hybrid mode
		m_hybridSwitch2   = new DigitalInput(5);			// switch for changing hybrid mode
		m_ballCountSensor = new DigitalInput(7);			// light sensor for telling if ball is in robot

		m_driverstationlcd = DriverStationLCD::GetInstance();	// sends print statements to driver station
		
		m_wheelDrive->SetSafetyEnabled(SAFETY);
		m_wheelDrive->SetExpiration(EXPIRATION);
		m_casterDrive->SetSafetyEnabled(true);
		m_casterDrive->SetExpiration(EXPIRATION);
		m_houserDrive->SetSafetyEnabled(SAFETY);
		m_houserDrive->SetExpiration(EXPIRATION); // thought about increasing setexpiration time, didnt have time to test to see what effect that would have

		m_topShootDrive->SetSafetyEnabled(SAFETY);   // set to true seems to slow things down
		m_topShootDrive->SetExpiration(EXPIRATION);
		m_botShootDrive->SetSafetyEnabled(SAFETY); // set to true seems to slow things down
		m_botShootDrive->SetExpiration(EXPIRATION);

		m_sweeperDrive->SetSafetyEnabled(SAFETY);
		m_sweeperDrive->SetExpiration(EXPIRATION);
		m_riserDrive->SetSafetyEnabled(SAFETY);
		m_riserDrive->SetExpiration(EXPIRATION);

		m_buttonCheckTask  = new Task("ButtonCheckTask", (FUNCPTR) ButtonCheckTask,  Task::kDefaultPriority, 64000);
		m_moveHouserTask   = new Task("MoveHouserTask",  (FUNCPTR) MoveHouserTask,   Task::kDefaultPriority, 64000);
		m_moveCasterTask   = new Task("MoveCasterTask",  (FUNCPTR) MoveCasterTask,   Task::kDefaultPriority, 64000);

		m_manageCompressorTask = new Task("ManageCompressorTask", (FUNCPTR) ManageCompressorTask, Task::kDefaultPriority, 64000);

		m_RunShootersTask        = new Task("RunShootersTask", (FUNCPTR) RunShootersTask, Task::kDefaultPriority, 64000);

		m_runSweeperTask   = new Task("RunSweeperTask",  (FUNCPTR) RunSweeperTask,   Task::kDefaultPriority, 64000);
		m_runRiserTask     = new Task("RunRiserTask",    (FUNCPTR) RunRiserTask,     Task::kDefaultPriority, 64000);

		m_brakeTask        = new Task("BrakeTask", (FUNCPTR) BrakeTask, Task::kDefaultPriority, 64000);
		
		g_keepRunning = true;  // all tasks should start running now
		
		g_analogModule1 = AnalogModule::GetInstance(1);
		
		//////// Assigns class objects to global values ////////////
		
		g_driverstationlcd = m_driverstationlcd;
		g_wheelDrive       = m_wheelDrive;
		g_victors[0]       = m_houserDrive;
		g_victors[1]       = m_casterDrive;

		g_solenoid[0]      = m_solenoidA;
		g_solenoid[1]      = m_solenoidB;
		g_cmprsrRelay      = m_cmprsrRelay;

		g_jaguars[0]       = m_topShootDrive;
		g_jaguars[1]       = m_botShootDrive;

		g_victors[3]       = m_sweeperDrive;
		g_victors[4]       = m_riserDrive;

//		g_ultrasonic       = m_ultrasonic;

		g_digitalInputs[1] = m_hybridSwitch1;
		g_digitalInputs[2] = m_hybridSwitch2;
		g_digitalInputs[3] = m_ballCountSensor;

		g_joysticks[0]     = m_driveStick;
		g_joysticks[1]     = m_buttonStick;
		
		g_digitalModule1   = digitalModuleDash;
	}
	
	~DashboardDataExample(void) // deletes all objects once program is ended
	{
		g_keepRunning = false;
		
		Wait (3*WAIT_TIME);    // wait for tasks to stop running
		
		if (m_runRiserTask)			delete(m_runRiserTask);
		if (m_runSweeperTask)		delete(m_runSweeperTask);	
		if (m_RunShootersTask)		delete(m_RunShootersTask);

		if (m_manageCompressorTask) delete(m_manageCompressorTask);

		if (m_buttonCheckTask)		delete(m_buttonCheckTask);
		if (m_moveHouserTask)		delete(m_moveHouserTask);
		if (m_moveCasterTask)		delete(m_moveCasterTask);
		if (m_brakeTask)            delete(m_brakeTask);
		
		if (m_wheelDrive)			delete(m_wheelDrive);
		if (m_houserDrive)			delete(m_houserDrive);
		if (m_casterDrive)			delete(m_casterDrive);

		if (m_topShootDrive)		delete(m_topShootDrive);
		if (m_botShootDrive)		delete(m_botShootDrive);

		if (m_sweeperDrive)			delete(m_sweeperDrive);
		if (m_riserDrive)			delete(m_riserDrive);

		if (m_ultrasonic)           delete(m_ultrasonic);

		if (m_hybridSwitch1)		delete(m_hybridSwitch1);
		if (m_hybridSwitch2)		delete(m_hybridSwitch2);
		if (m_ballCountSensor)		delete(m_ballCountSensor);
		
		if (m_driveStick)			delete(m_driveStick);
		if (m_buttonStick)			delete(m_buttonStick);
		
		if (m_solenoidA)			delete(m_solenoidA);
		if (m_solenoidB)			delete(m_solenoidB);
		if (m_cmprsrRelay)			delete(m_cmprsrRelay);
	
	}
	/******* end DESTRUCTOR *************/
	
	/************************************
	 * Tele-operator mode
	 * Start all tasks, then loop through driving control 
	 * and dashboard interaction, then stop all tasks
	 ************************************/
	void OperatorControl()
	{
		float currDistance = 0.0;
		
		GetWatchdog().SetEnabled(false);
		m_moveHouserTask->Start();			// starts houser task
		m_brakeTask->Start();				// starts brake task (UNTESTED)
		m_moveCasterTask->Start();			// starts caster task

		//m_manageCompressorTask->Start();	// starts compressor management

	    m_runSweeperTask->Start();			// starts sweeper task
		m_runRiserTask->Start();			// starts riser task
		
		m_RunShootersTask->Start();			// starts shooter task
	
		g_cmprsrRelay->Set(Relay::kOff);  	// start with it off
		g_useCasters = true; 				// start with caster wheels down
		int loopCount = GOOSE_LOOP_COUNT;  	// initialize count for drive Goose function
		float direction = 0.0;				// checks direction for "goose" burst to operate
		int sign = 0;						// sets drive direction for "goose" burst
		m_buttonCheckTask->Start(); 		// start this last so no button input accepted until it is all started up
		
		//// START THE ACTION //////
		while (IsOperatorControl())
		{
			GetWatchdog().Feed();  			// touch connection to robot to keep watchdog process alive
				if (g_goose)  				// use button for a very short burst to aid moving on bridge
				{
					if (loopCount-- > 0)  	// decrement loopCount and check to see if zero yet
					{
						if (direction == 0.0)
						{
							direction = m_driveStick->GetY();
							if (direction == 0.0)
								direction = 1;
							if (direction < 0)
								sign = -1;
							else
								sign = 1;
						}
						m_wheelDrive->Drive(sign, m_driveStick->GetX());
					}
					if (loopCount <= 0)
					{
						g_goose = false;
						loopCount = GOOSE_LOOP_COUNT;
						direction = 0;
					}
				}
				else if (g_back)
				{
					if (backLoopCount-- >= 0)
					{
						m_wheelDrive->Drive(0.5,0);
					}
					else
					{
						g_back = false;
						backLoopCount = BACK_LOOP_COUNT;
					}
				}
				else if (g_shootBack)
				{
					if (g_shootBackLoopCount-- >= 0)
					{
						m_wheelDrive->Drive(-0.5,0);
					}
					else
					{
						g_shootBack = false;
						g_shootBackLoopCount = 75;
					}	
				}
				else if (g_brake)
				{
					
				}
				else  // if we aren't in some special mode, just drive
				{
					m_wheelDrive->ArcadeDrive(m_driveStick);
				}
			if (g_turnPrintOn)
			{
				currDistance = Range();
				m_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line4, "DISTANCE: %f", currDistance);
			}
			// all print statements are sent now in this one LCD update
			m_driverstationlcd->UpdateLCD();
			dashboardDataFormat.SendIOPortData();
			Wait(WAIT_TIME);
		}
		m_buttonCheckTask->Stop();
		m_moveHouserTask->Stop();
		m_moveCasterTask->Stop();
		m_brakeTask->Stop();

		//m_manageCompressorTask->Stop();

		m_runSweeperTask->Stop();
		m_runRiserTask->Stop();
		
		m_RunShootersTask->Stop();
	}
	///////// end OperatorControl ////////////
	/****************************************
	 * Autonomous - automatic mode
	 ****************************************/
	void Autonomous(void)
	{
		m_driverstationlcd->Clear();
		m_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line5, "STARTING AUTONOMOUS");
		m_driverstationlcd->UpdateLCD();

		// get initial switch settings
		unsigned int bits = g_digitalModule1->GetDIO();
		bits = bits >> 12;
		bits = bits & 3;
		if (bits > 3)  // value should be 0,1,2,3 - can't be less than zero. If more than 3 then we got garbage
			bits = 0;
		
		int statusResult = 0;

		m_manageCompressorTask->Start();   // start up the compressor so we can shoot in autonomous

		switch (bits)
		{
		case 0:
		{
			if (IsAutonomous())
				{
					// Drive forward to bumper for 2.5 seconds 
					// updated 12:00 pm 4/6/2012
					for (int i = 1; i < 286; i++) // drive 11 inches every 0.5 seconds - 1 inch every 5 "turns"
					{
						m_wheelDrive->Drive(0.4,0.0);
						Wait (0.01);
					}
					m_wheelDrive->Drive(0.0,0.0);  // stop the drive motors - don't want to coast
				}
			if (IsAutonomous())
				{
					// spin up the shooting drives for 4.1 seconds (200 * 0.01 = 2)
					for (int i = 1; i < 410; i++)
					{
						m_topShootDrive->Set(TOP_SHOOT_DRIVE_CLOSE);
						m_botShootDrive->Set(BOT_SHOOT_DRIVE_CLOSE);
						Wait (0.01);
					}
				}
			if (IsAutonomous())
				{
					// push in the ball plate
					g_solenoid[0]->Set(false);
					g_solenoid[1]->Set(true);
					// run shooters one-half second more to make sure plate is in place
					for (int i = 1; i < 50; i++)
					{
						m_topShootDrive->Set(TOP_SHOOT_DRIVE_CLOSE);
						m_botShootDrive->Set(BOT_SHOOT_DRIVE_CLOSE);
					}
					// run the riser for five seconds to feed balls into shooter
					// continue to run shooting motors
					for (int i = 1; i < 500; i++)
					{
						g_victors[4]->Set(-1.0);
						m_topShootDrive->Set(TOP_SHOOT_DRIVE_CLOSE);
						m_botShootDrive->Set(BOT_SHOOT_DRIVE_CLOSE);
						Wait (0.01);
					}
				}
			if (IsAutonomous())
				{
					// release the ball plate
					g_solenoid[0]->Set(true);
					g_solenoid[1]->Set(false);
					// turn off riser drive and shooter drives
					g_victors[4]->Set(0.0);
					m_topShootDrive->Set(0);
					m_botShootDrive->Set(0);
				}
			/*
			if (IsAutonomous())
				{
					// drive to bridge for 5.5 seconds at higher speed - this should go from 58 to 180 inches from wall
					for (int i = 1; i < 551; i++)
					{
						m_wheelDrive->Drive(-0.6,0.0);
						Wait (0.01);
					}
					m_wheelDrive->Drive(0.0,0.0);  // stop the drive motors - don't want to coast
				}
				*/
			break;
		}
		case 1:	// shoot to low basket from key
		{
			if (IsAutonomous())
				{
					// Drive forward to bumper for 2.5 seconds 
					// updated 12:00 pm 4/6/2012
					for (int i = 1; i < 286; i++) // drive 11 inches every 0.5 seconds - 1 inch every 5 "turns"
					{
						m_wheelDrive->Drive(0.4,0.0);
						Wait (0.01);
					}
					m_wheelDrive->Drive(0.0,0.0);  // stop the drive motors - don't want to coast
				}
			if (IsAutonomous())
				{
					// spin up the shooting drives for 4.1 seconds (200 * 0.01 = 2)
					for (int i = 1; i < 410; i++)
					{
						m_topShootDrive->Set(TOP_SHOOT_DRIVE_KEY);
						m_botShootDrive->Set(TOP_SHOOT_DRIVE_KEY);
						Wait (0.01);
					}
				}
			if (IsAutonomous())
				{
					// push in the ball plate
					g_solenoid[0]->Set(false);
					g_solenoid[1]->Set(true);
					// run shooters one-half second more to make sure plate is in place
					for (int i = 1; i < 50; i++)
					{
						m_topShootDrive->Set(TOP_SHOOT_DRIVE_KEY);
						m_botShootDrive->Set(TOP_SHOOT_DRIVE_KEY);
					}
					// run the riser for five seconds to feed balls into shooter
					// continue to run shooting motors
					for (int i = 1; i < 500; i++)
					{
						g_victors[4]->Set(-1.0);
						m_topShootDrive->Set(TOP_SHOOT_DRIVE_KEY);
						m_botShootDrive->Set(TOP_SHOOT_DRIVE_KEY);
						Wait (0.01);
					}
				}
			if (IsAutonomous())
				{
					// release the ball plate
					g_solenoid[0]->Set(true);
					g_solenoid[1]->Set(false);
					// turn off riser drive and shooter drives
					g_victors[4]->Set(0.0);
					m_topShootDrive->Set(0);
					m_botShootDrive->Set(0);
				}
			if (IsAutonomous())
				{
					// drive to bridge 
					for (int i = 1; i < 601; i++)
					{
						m_wheelDrive->Drive(-0.4,0.0);
						Wait (0.01);
					}
					m_wheelDrive->Drive(0.0,0.0);  // stop the drive motors - don't want to coast
				}
			for (int i = 1; i < 201; i++)
					{
						g_moveHouser = 1; // deploy Houser arm for three seconds
						Wait (0.01);
					}
			break;
		}
		case 4:
		{
			// code to use ultrasonic to find distance to basket for shooting, then back up to bridge and deploy Houser arm
			statusResult = DriveToPoint(SHOOT_LOC_CLOSE);  // drive to the bumper close location to shoot

			m_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line5, "RESULT: %i", statusResult);
			m_driverstationlcd->UpdateLCD();
			m_wheelDrive->Drive(0,0);  // stop the robot from driving further (for some reason it scoots)
			if (IsAutonomous())
				{
					// spin up the shooting drives for 4.1 seconds (200 * 0.01 = 2)
					for (int i = 1; i < 410; i++)
					{
						m_topShootDrive->Set(TOP_SHOOT_DRIVE_CLOSE);
						m_botShootDrive->Set(BOT_SHOOT_DRIVE_CLOSE);
						Wait (0.01);
					}
				}
			if (IsAutonomous())
				{
					// push in the ball plate
					g_solenoid[0]->Set(false);
					g_solenoid[1]->Set(true);
					// run shooters one-half second more to make sure plate is in place
					for (int i = 1; i < 50; i++)
					{
						m_topShootDrive->Set(TOP_SHOOT_DRIVE_CLOSE);
						m_botShootDrive->Set(BOT_SHOOT_DRIVE_CLOSE);
					}
					// run the riser for five seconds to feed balls into shooter
					// continue to run shooting motors
					for (int i = 1; i < 500; i++)
					{
						g_victors[4]->Set(-1.0);
						m_topShootDrive->Set(TOP_SHOOT_DRIVE_CLOSE);
						m_botShootDrive->Set(BOT_SHOOT_DRIVE_CLOSE);
						Wait (0.01);
					}
				}
			if (IsAutonomous())
				{
					// release the ball plate
					g_solenoid[0]->Set(true);
					g_solenoid[1]->Set(false);
					// turn off riser drive and shooter drives
					g_victors[4]->Set(0.0);
					m_topShootDrive->Set(0);
					m_botShootDrive->Set(0);
				}
	/*		statusResult = DriveToPoint(BRIDGE_LOC);       // drive back to the middle bridge
			for (int i = 1; i < 201; i++)
			{
				g_moveHouser = 1; // deploy Houser arm for three seconds
				Wait (0.01);
			}   */
			break;
		}
		case 3:    // shoot low basket
		{
			if (IsAutonomous())
				{
					// Drive forward 
					for (int i = 1; i < 251; i++) // drive 11 inches ever 0.5 seconds
					{
						m_wheelDrive->Drive(0.4,0.0);
						Wait (0.01);
					}
					m_wheelDrive->Drive(0.0,0.0);  // stop the drive motors - don't want to coast
				}
			if (IsAutonomous())
				{
					// spin up the shooting drives for 4.1 seconds (200 * 0.01 = 2)
					for (int i = 1; i < 410; i++)
					{
						m_topShootDrive->Set(TOP_SHOOT_DRIVE_CLOSE);
						m_botShootDrive->Set(BOT_SHOOT_DRIVE_CLOSE);
						Wait (0.01);
					}
				}
			if (IsAutonomous())
				{
					// push in the ball plate
					g_solenoid[0]->Set(false);
					g_solenoid[1]->Set(true);
					// run shooters one-half second more to make sure plate is in place
					for (int i = 1; i < 50; i++)
					{
						m_topShootDrive->Set(TOP_SHOOT_DRIVE_CLOSE);
						m_botShootDrive->Set(BOT_SHOOT_DRIVE_CLOSE);
					}
					// run the riser for five seconds to feed balls into shooter
					// continue to run shooting motors
					for (int i = 1; i < 500; i++)
					{
						g_victors[4]->Set(-1.0);
						m_topShootDrive->Set(TOP_SHOOT_DRIVE_CLOSE);
						m_botShootDrive->Set(BOT_SHOOT_DRIVE_CLOSE);
						Wait (0.01);
					}
				}
			if (IsAutonomous())
				{
					// release the ball plate
					g_solenoid[0]->Set(true);
					g_solenoid[1]->Set(false);
					// turn off riser drive and shooter drives
					g_victors[4]->Set(0.0);
					m_topShootDrive->Set(0);
					m_botShootDrive->Set(0);
				}
			/*
			if (IsAutonomous())
				{
					// drive to bridge for 5.5 seconds at higher speed - this should go from 58 to 180 inches from wall
					for (int i = 1; i < 551; i++)
					{
						m_wheelDrive->Drive(-0.6,0.0);
						Wait (0.01);
					}
					m_wheelDrive->Drive(0.0,0.0);  // stop the drive motors - don't want to coast
				}
				*/
			break;
		}
		case 2: // shoot for low basket from key
		{
			if (IsAutonomous())
			{
				// spin up the shooting drives for 3.1 seconds (200 * 0.01 = 2)
				for (int i = 1; i < 310; i++)
				{
					m_topShootDrive->Set(TOP_SHOOT_DRIVE_KEY);
					m_botShootDrive->Set(BOT_SHOOT_DRIVE_KEY);
					Wait (0.01);
				}
			}
			if (IsAutonomous())
			{
				// push in the ball plate
				g_solenoid[0]->Set(false);
				g_solenoid[1]->Set(true);
				// run shooters one-half second more to make sure plate is in place
				for (int i = 1; i < 50; i++)
				{
					m_topShootDrive->Set(TOP_SHOOT_DRIVE_KEY);
					m_botShootDrive->Set(BOT_SHOOT_DRIVE_KEY);
				}
				// run the riser for five seconds to feed balls into shooter
				// continue to run shooting motors
				for (int i = 1; i < 251; i++)
				{
					g_victors[4]->Set(-1.0);
					m_topShootDrive->Set(TOP_SHOOT_DRIVE_KEY);
					m_botShootDrive->Set(BOT_SHOOT_DRIVE_KEY);
					Wait (0.01);
				}
				// push out the ball plate
				g_solenoid[0]->Set(false);
				g_solenoid[1]->Set(true);
				for (int i = 1; i < 100; i++)
				{
					g_victors[4]->Set(0);
					m_topShootDrive->Set(TOP_SHOOT_DRIVE_KEY);
					m_botShootDrive->Set(BOT_SHOOT_DRIVE_KEY);
					Wait (0.01);
				}
				// push in the ball plate
				g_solenoid[0]->Set(false);
				g_solenoid[1]->Set(true);
				// run shooters one-half second more to make sure plate is in place
				for (int i = 1; i < 50; i++)
				{
					m_topShootDrive->Set(TOP_SHOOT_DRIVE_KEY);
					m_botShootDrive->Set(BOT_SHOOT_DRIVE_KEY);
				}
				for (int i = 1; i < 251; i++)
				{
					g_victors[4]->Set(-1.0);
					m_topShootDrive->Set(TOP_SHOOT_DRIVE_KEY);
					m_botShootDrive->Set(BOT_SHOOT_DRIVE_KEY);
					Wait (0.01);
				}
			}
			if (IsAutonomous())
			{
				// release the ball plate
				g_solenoid[0]->Set(true);
				g_solenoid[1]->Set(false);
				// turn off riser drive and shooter drives
				g_victors[4]->Set(0.0);
				m_topShootDrive->Set(0);
				m_botShootDrive->Set(0);
			}
			break;
		}
		default:
			// display that we hit an unused mode
			m_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line5, "AUTO MODE UNKNOWN");
			m_driverstationlcd->UpdateLCD();
			break;
		}
	}
	///////// end Autonomous /////////////


	/*********************************************
	 * Range - returns a floating point distance
	 * Gets the ultrasonic feedback and adjusts it to a range in inches
	 *********************************************/
	float Range()
	{
		float range = m_ultrasonic->GetVoltage() / 0.0098;
		return range;		
	}
	
	/*********************************************
	 * DriveToPoint
	 * Drives robot to a fixed distance from the shooting face,
	 * using ultrasonic to tell distance
	 * INPUT: Distance at which to stop as floating point
	 * RETURN: success code as integer
	 *********************************************/
	int DriveToPoint (float stopPoint)
	{
		float currDistance = 0.0;   // current distance from sonar on robot to next object (hopefully a wall)
		float speedFactor  = 0.0;   // what speed to drive the motors at
		bool  atStopPoint  = false; // are we at the point where we are supposed to stop yet
		int   loopCounter  = 0;     // added to debug whether it was going through loop or not
		
		while (!atStopPoint)   // if we are not yet where we are supposed to stop
		{
			loopCounter++;            // counter used to keep loop from running forever
			currDistance = Range();   // get current distance
			m_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line4, "DISTANCE: %f", currDistance);
			g_driverstationlcd->UpdateLCD();
			if (currDistance > (stopPoint + 1.0))  // use a 1.0 inch buffer because the measurements are not precise - vary about .2 inch constantly
			{
				speedFactor = 0.2 * (currDistance/stopPoint);	// adjust speed in increments of 0.2 to a minimum of 0.2
				if (speedFactor > 0.8)
				{
					speedFactor = 0.8;
				}
				m_wheelDrive->Drive(speedFactor,0.0);
			}
			else if (currDistance < (stopPoint - 1.0))
			{
				speedFactor = -0.2 * (stopPoint/currDistance);	
				if (speedFactor < -0.8)
				{
					speedFactor = -0.8;
				}
				m_wheelDrive->Drive(speedFactor,0);
			}
			else
			{
				atStopPoint = true;
			}
			// print out the distance info to driver station
			m_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line5, "StopPoint: %f", stopPoint);
			m_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line4, "DISTANCE: %f", currDistance);
			m_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line6, "LC-SF: %i - %f", loopCounter, speedFactor);
			m_driverstationlcd->UpdateLCD();
			dashboardDataFormat.SendIOPortData();
			// Check to see if we are still in Autonomous mode or gone too many loop counts
			if ( !IsAutonomous() || (loopCounter > 2000))
			{
				speedFactor = 0.0;
				m_wheelDrive->Drive(0,0);  // stop the wheels
				return 0;   // leave the routine if time is up
			}
			Wait(0.005);
		}
		return 1;
	}
	/////// end DriveToPoint /////////
	
};
//////////////////////////////////////////////////////////////////
////////////// END CLASS DASHBOARDDATAEXAMPLE ////////////////////
//////////////////////////////////////////////////////////////////

START_ROBOT_CLASS(DashboardDataExample);

/*****************************************
 * ButtonCheckTask
 * Checks for a button pressed down on the joystick
 * NOTE:::: Currently set to read the DRIVER JOYSTICK 
 * - set to joystick[1] to read the button stick
 * NOTE:::: If we want to check buttons on BOTH sticks 
 * - add a second round of button checks to this routine
 *****************************************/
void ButtonCheckTask(void)
{
//	double encoderDistance;
//	bool   encoderDirection = true;
	while (g_keepRunning)
	{
		static int lastButtonPress = 0;
		int buttonPress = 0;
		
		static int lastDriveButtonPress = 0;
		int driveButtonPress = 0;
		
		////// CHECK BUTTONS ON BUTTON STICK
		for (int i = 0; i <= 12;  i ++)
		{
			if (g_joysticks[1]->GetRawButton(i))  // check if the button is pressed
			{
				buttonPress = i;  // assign the button to a variable
				break;
			}
		}
		////// CHECK BUTTONS ON DRIVE STICK
		for (int i = 0; i <= 12;  i ++)
		{
			if (g_joysticks[0]->GetRawButton(i))  // check if the button is pressed
			{
				driveButtonPress = i;  // assign the button to a variable
				break;
			}
		}
		
		////////////////////////////////////////////////
		//////////// BUTTON STICK BUTTONS //////////////
		////////////////////////////////////////////////
		if (lastButtonPress == 0) // if the button is being held
		{
			switch (buttonPress)  // choose actions for buttons
			{
			case 0:
				g_runRiserDrive   = 0;       // turn off riser motor
				g_runSweeperDrive = 0;       // turn off sweeper motor
				if (g_solenoidsOff == false) // turn off the pneumatic arm for ball plate
				{
					g_solenoid[0]->Set(true);
					g_solenoid[1]->Set(false);
					g_solenoidsOff = true;
				}
				break;
			case 1: // shoot the balls - hold trigger down until all balls are through
				if (g_solenoidsOff) // change solenoid setting only once to release plate forward
				{
					g_solenoid[0]->Set(false);  // move ball plate forward
					g_solenoid[1]->Set(true);
					g_solenoidsOff = false;
				}
				g_runRiserDrive = 1;  // start moving the balls up to shoot
				buttonPress = 0;      // fake this so it will let us hold the trigger down
				break;
			case 2: // turn on shooting wheels
				g_shooterWheelsOn = true;
				break;
			case 3: // turn off shooting wheels
				g_shooterWheelsOn = false;
				break;
			case 4: // run the sweeper (in) and the riser (up) while button is held
				g_runRiserDrive   = 1; // run the riser upward
				g_runSweeperDrive = 1; // run the sweeper inward
				buttonPress = 0;  // fake this so it will let us hold the button down
				break;
			case 5: // run the riser (down) while button is held
				g_runRiserDrive   = -1;  // run down
				buttonPress       = 0;   // fake this so it will let us hold the button down
				break;
			case 6:
				break;
			case 7:
				break;
			case 8: //alternate move houser up
				g_moveHouser = 2;
				break;
			case 9: // assign far shooting speeds
				g_topShootDriveSpeed = TOP_SHOOT_DRIVE_FAR;
				g_botShootDriveSpeed = BOT_SHOOT_DRIVE_FAR;
				if (g_turnPrintOn)
				{
					g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line5, "TOP: %1.3f", g_topShootDriveSpeed);
					g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line6, "BOTTOM: %1.3f", g_botShootDriveSpeed); 
					//g_driverstationlcd->UpdateLCD();
				}
				g_shootState = 3;
				break;
			case 10: // assign medium shooting speeds
				g_topShootDriveSpeed = TOP_SHOOT_DRIVE_KEY;
				g_botShootDriveSpeed = BOT_SHOOT_DRIVE_KEY;
				if (g_turnPrintOn)
				{
					g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line5, "TOP: %1.3f", g_topShootDriveSpeed);
					g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line6, "BOTTOM: %1.3f", g_botShootDriveSpeed);
					//g_driverstationlcd->UpdateLCD();
				}
				g_shootState = 2;
				break;
			case 11: //assign close shooting speeds
				g_topShootDriveSpeed = TOP_SHOOT_DRIVE_CLOSE;
				g_botShootDriveSpeed = BOT_SHOOT_DRIVE_CLOSE;
				if (g_turnPrintOn)
				{
					g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line5, "TOP: %1.3f", g_topShootDriveSpeed);
					g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line6, "BOTTOM: %1.3f", g_botShootDriveSpeed);
					//g_driverstationlcd->UpdateLCD();
				}
				g_shootState = 1;
				break;
			}
		}
		////////////////////////////////////////////////
		///////////// DRIVE STICK BUTTONS //////////////
		////////////////////////////////////////////////
		if (lastDriveButtonPress == 0) // if the button is being held
		{
			switch (driveButtonPress)  // choose actions for buttons
			{
			case 0:
				g_moveHouser = 0;     // set to off
				g_moveCaster = 0;     // set to off
				g_brake      = false;
				break;
			case 1: // goose the motors - drive one inch at a time for bridge fine tuning
				g_goose = true;
				break;
			case 2: // casters down
				g_moveCaster = 1;
				g_moveHouser = 3;
				g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line1, "CASTERS: DOWN");
				break;
			case 3: // casters up
				g_moveCaster = 2;
				g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line1, "CASTERS: UP");
				break;
			case 4: // go back four inches - auto distance from bridge for Houser arm deployment
				g_back = true;
				break;
			case 5:  // move robot back from bumper to shoot
				g_shootBack = true;
				break;
			case 6: // move houser arm up
				g_moveHouser = 2;
				break;
			case 7: // move houser arm down
				g_moveHouser = 1;
				break;
			case 8:
				if (g_joysticks[0]->GetZ() > 0.5) // if z-axis is up
				{
					g_botShootDriveSpeed += .02;
					if (g_turnPrintOn)
					{
						g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line5, "TOP: %1.3f", g_topShootDriveSpeed);
						g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line6, "BOTTOM: %1.3f", g_botShootDriveSpeed);
						//g_driverstationlcd->UpdateLCD();
					}
				}
				else if (g_joysticks[0]->GetZ() < -0.5) // if z-axis is down
				{
					g_botShootDriveSpeed -= .02;
					if (g_turnPrintOn)
					{
						g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line5, "TOP: %1.3f", g_topShootDriveSpeed);
						g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line6, "BOTTOM: %1.3f", g_botShootDriveSpeed);
						//g_driverstationlcd->UpdateLCD();
					}
				}
				else          // if in middle - decrement both motors
				{
					g_topShootDriveSpeed -= .02;
					g_botShootDriveSpeed -= .02;
					if (g_turnPrintOn)
					{
						g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line5, "TOP: %1.3f", g_topShootDriveSpeed);
						g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line6, "BOTTOM: %1.3f", g_botShootDriveSpeed);						//g_driverstationlcd->UpdateLCD();
					}
				}
				if (g_turnPrintOn)
				{
					g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line5, "TOP: %1.3f", g_topShootDriveSpeed);
					g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line6, "BOTTOM: %1.3f", g_botShootDriveSpeed);				}	
				break;
			case 9: 
				if (g_joysticks[0]->GetZ() > 0.5) // if z-axis is up
					{
						g_topShootDriveSpeed += .02;
						if (g_turnPrintOn)
						{
							g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line5, "TOP: %1.3f", g_topShootDriveSpeed);
							g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line6, "BOTTOM: %1.3f", g_botShootDriveSpeed);							//g_driverstationlcd->UpdateLCD();
						}
					}
					else if (g_joysticks[0]->GetZ() < -0.5) // if z-axis is down
					{
						g_topShootDriveSpeed -= .02;
						if (g_turnPrintOn)
						{
							g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line5, "TOP: %1.3f", g_topShootDriveSpeed);
							g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line6, "BOTTOM: %1.3f", g_botShootDriveSpeed);							//g_driverstationlcd->UpdateLCD();
						}
					}
					else          // if in middle - increments both motors
					{
						g_topShootDriveSpeed += .02;
						g_botShootDriveSpeed += .02;
						if (g_turnPrintOn)
						{
							g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line5, "TOP: %1.3f", g_topShootDriveSpeed);
							g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line6, "BOTTOM: %1.3f", g_botShootDriveSpeed);							//g_driverstationlcd->UpdateLCD();
						}
					}
					if (g_turnPrintOn)
					{
						g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line5, "TOP: %1.3f", g_topShootDriveSpeed);
						g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line6, "BOTTOM: %1.3f", g_botShootDriveSpeed);						//g_driverstationlcd->UpdateLCD();
					}			
				break;
			case 10: // Change speed of bottom drive
				break;
			case 11: // drive forward with ultrasonic		
				break;
			}
		}
		if (g_turnPrintOn)
		{
			g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line3, "BUTTONS: %i - %i", buttonPress, driveButtonPress);
			//g_driverstationlcd->UpdateLCD();
		}
		lastButtonPress      = buttonPress;
		lastDriveButtonPress = driveButtonPress;
		Wait(WAIT_TIME);
	}
}
/////////// end ButtonCheckTask //////////

/************************************
 * MoveHouserTask
 * Moves Houser arm (bridge arm).
 * Sets speed once for a button press because the Victor 
 * controller will keep pushing until a different speed is set.
 ************************************/
void MoveHouserTask(void)
{
	while (g_keepRunning)
	{
		switch (g_moveHouser)
		{
		case 0: // stop
			g_victors[0]->Set(0.0);
			break;
		case 1:  // push down onto ramp
			g_victors[0]->Set(1.0);
			break;
		case 2: // retract slowly
			g_victors[0]->Set(-0.7);
			break;
		case 3:
			g_victors[0]->Set(-1.0);
			break;
		default:
			break;
		}
		Wait(WAIT_TIME);
	}
}

/////// end MoveHouserTask ///////////////////

/************************************
 * MoveCasterTask
 * Moves Houser arm (bridge arm).
 * Sets speed once for a button press because the Victor 
 * controller will keep pushing until a different speed is set.
 ************************************/
void MoveCasterTask(void)
{
	while (g_keepRunning)
		{
			switch (g_moveCaster)
			{
			case 0: // stop
				g_victors[1]->Set(0.0);
				break;
			case 1:  // push down casters
				g_victors[1]->Set(1.0);
				break;
			case 2: // pull casters up
				g_victors[1]->Set(-1.0);
				break;
			default:
				break;
			}
			Wait(WAIT_TIME);
		}
}
//////// end MoveCasterTask //////////

/***************************************
 * RunSweeperTask
 * turns the sweeper on/off and sets direction in/out
 ***************************************/
void RunSweeperTask()
{
	static bool sweeperIsOff = true;
	
	while (g_keepRunning)
	{
		if (g_runSweeperDrive == 1)  // pull balls in
		{
			g_victors[3]->Set(1.0); // run sweeper drive full speed
			sweeperIsOff = false;	
		}
		else if (g_runSweeperDrive == -1)  // push balls out
		{
			g_victors[3]->Set(-0.8);
			sweeperIsOff = false;
		}
		else  // stop running sweeper drive
		{
			if (!sweeperIsOff)
			{
				g_victors[3]->Set(0.0);  //turn off sweeper drive
				sweeperIsOff = true;
			}
		}
	Wait (WAIT_TIME);
	}
}
//////// end RunSweeperTask ///////////
 
/******************************************
 * RunRiserTask
 * turns the riser up, down, or off
 ******************************************/
void RunRiserTask()
{
	bool riserIsOff = true;
	while (g_keepRunning)
	{
		if (g_runRiserDrive == 1)  // roll things up
		{
			g_victors[4]->Set(-0.8); // drive sweeper motor
			riserIsOff = false;
		}
		else if (g_runRiserDrive == -1)  // roll things down
		{
			g_victors[4]->Set(0.8);  // drive motor downward
			riserIsOff = false;
		}
		else    // stop
		{
			if (riserIsOff == false)
			{
				g_victors[4]->Set(0.0); // stop the riser motor
				riserIsOff = true;
			}
		}
	Wait (WAIT_TIME);
	}
}
//////// end RunRiserTask //////////


/***************************************
 * ManageCompressorTask
 * Runs the compressor as necessary to maintain pressure
 ***************************************/
void ManageCompressorTask()
{
	DigitalModule *digitalModule1 = DigitalModule::GetInstance(1);
	while (g_keepRunning)
	{
		int port = digitalModule1->GetDIO();
		//g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line5, "Port: %x", port);
		//g_driverstationlcd->UpdateLCD();
		if (port & 0x0800)
			// DigitalModule::GetInstance(2)->SetRelayForward(1,false);
			g_cmprsrRelay->Set(Relay::kOff);
		else
			// DigitalModule::GetInstance(2)->SetRelayForward(1,true);
			g_cmprsrRelay->Set(Relay::kReverse);
		Wait(1);
	}	
}
///// end ManageCompressorTask //////////


/***********************************
 * RunShootersTask
 * Turns the shooter motors on or off
 */
void RunShootersTask()
{
	bool notShooting = 1;  // are we shooting or not
	
	while (g_keepRunning)
	{
		if (g_shooterWheelsOn)
		{
			g_jaguars[0]->Set(g_topShootDriveSpeed);
			g_jaguars[1]->Set(g_botShootDriveSpeed);
			notShooting = 0;
			if (g_shootState == 1)
				g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line2, "SHOOT: ON - NEAR");
			else if (g_shootState == 2)
				g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line2, "SHOOT: ON - KEY");
			else
				g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line2, "SHOOT: ON - FAR");
			g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line5, "TOP: %1.3f", g_topShootDriveSpeed);
			g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line6, "BOTTOM: %1.3f", g_botShootDriveSpeed);

		}
		else   // we are not shooting
		{
			if (!notShooting)  // if our last state was shooting
			{
				g_jaguars[0]->Set(0.0);
				g_jaguars[1]->Set(0.0);
				notShooting = 1;
				g_driverstationlcd->PrintfLine(DriverStationLCD::kUser_Line2, "SHOOT: OFF");
			}
		}
	g_driverstationlcd->UpdateLCD();
	Wait (WAIT_TIME);
	}
}

void BrakeTask()
{
	if (g_brake)
	{
		// drive wheels briefly forward and back a small amount to hold position
		g_wheelDrive->Drive(0.2, 0);
		g_wheelDrive->Drive(-0.2, 0);
		g_wheelDrive->Drive(0.2, 0);
		g_wheelDrive->Drive(-0.2, 0);
		g_wheelDrive->Drive(0.0, 0);  // last command to stop wheels
	}
}
