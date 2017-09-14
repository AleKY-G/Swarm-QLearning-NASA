#include <ros/ros.h>

//ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>

//ROS messages
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <algorithm>
//Custom messages
#include <shared_messages/TagsImage.h>
//Todo
#include <mobility/target.h>
// To handle shutdown signals so the node quits properly in response to "rosnode kill"
#include <ros/ros.h>
#include <signal.h>

using namespace std;

//Random number generator
random_numbers::RandomNumberGenerator* rng;	

//Mobility Logic Functions
void setVelocity(double linearVel, double angularVel);

//Numeric Variables
geometry_msgs::Pose2D currentLocation;
geometry_msgs::Pose2D goalLocation;
int currentMode = 0;
std_msgs::String IDmsg;
float mobilityLoopTimeStep = 0.1; //time between the mobility loop calls
float status_publish_interval = 5;
float killSwitchTimeout = 10;
std_msgs::Int16 targetDetected; //ID of the detected target
bool targetsCollected [256] = {0}; //array of booleans indicating whether each target ID has been found
//Todo
bool targetsDetected [256] = {0};
float targetsDetected_x[256] ={0};
float targetsDetected_y[256]={0};
int myID = -1;
int IDcnt= 0;
int IDinit = -1;
int dropcnt =0;
int cluster_cnt = 0;
std::vector<std::string> hostID;
//Map Exploration
int ToScatter = 1;
int Look4Bdr = 1;
std_msgs::Float32MultiArray mapMsg;
float map_sidelength = 0;
int mapPioneer = 0;  
float rect_a = 0;
float rect_b = 0;
float rect_c = 0;
int ObstCnt = 0;
int grid_init = 1;


int grid1024p [1024] = {1};
int grid256p [256] = {1};
int grid64p [64] ={1};
int grid16p [16] = {1};
int grid4p [4] = {1};
float fixgx;
float fixgy;
float grid_resol = 0.75;
int grid_num = 256;

int IDX=1;
int TagColCnt = 0;
int SearchTimeCnt = 0;

// state machine states  
#define STATE_MACHINE_TRANSFORM	0
#define STATE_MACHINE_ROTATE	1
#define STATE_MACHINE_TRANSLATE	2
int stateMachineState = STATE_MACHINE_TRANSFORM;
#define CAMERA_DISTANCE 0.4
geometry_msgs::Twist velocity;
char host[128];
string publishedName;
char prev_state_machine[128];

//Publishers
ros::Publisher velocityPublish;
ros::Publisher stateMachinePublish;
ros::Publisher status_publisher;
ros::Publisher targetCollectedPublish;
ros::Publisher targetPickUpPublish;
ros::Publisher targetDropOffPublish;
//Todo
ros::Publisher hostIDPublisher;
ros::Publisher mapPublisher;
ros::Publisher targetDetectedPublish;

//Subscribers
//Todo
ros::Subscriber hostIDSubscriber;
ros::Subscriber mapSubscriber;
ros::Subscriber targetDetectedSubscriber;

ros::Subscriber joySubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber obstacleSubscriber;
ros::Subscriber odometrySubscriber;
ros::Subscriber targetsCollectedSubscriber;

//Timers
ros::Timer stateMachineTimer;
ros::Timer publish_status_timer;
ros::Timer killSwitchTimer;

// OS Signal Handler
void sigintEventHandler(int signal);

//Callback handlers
//Todo
void targetDetectedHandler(const mobility::target target_msg);
void hostIDHandler(const std_msgs::String::ConstPtr& message);
void mapHandler(const std_msgs::Float32MultiArray::ConstPtr& mapMsg);

void joyCmdHandler(const geometry_msgs::Twist::ConstPtr& message);
void modeHandler(const std_msgs::UInt8::ConstPtr& message);
void targetHandler(const shared_messages::TagsImage::ConstPtr& tagInfo);
void obstacleHandler(const std_msgs::UInt8::ConstPtr& message);
void odometryHandler(const nav_msgs::Odometry::ConstPtr& message);
void mobilityStateMachine(const ros::TimerEvent&);
void publishStatusTimerEventHandler(const ros::TimerEvent& event);
void targetsCollectedHandler(const std_msgs::Int16::ConstPtr& message);
void killSwitchTimerEventHandler(const ros::TimerEvent& event);

int main(int argc, char **argv) {

    gethostname(host, sizeof (host));
    string hostname(host);
//Define hostID vector 

	hostID.push_back("A");
	hostID.push_back("B");
	hostID.push_back("C");
	hostID.push_back("Zz1");
	hostID.push_back("Zz2");
	hostID.push_back("Zz3");

	mapMsg.data.push_back(0);
	mapMsg.data.push_back(0);
	mapMsg.data.push_back(0);
	mapMsg.data.push_back(0);
	
//Grid initialization ----->>

//Divide the whole region into 4 primary partitions (4p), each region has weight of 64, 
	for(int ii=0;ii<4;++ii)
		{
		grid4p [ii] = 64;
		}

//Divide each of the 4 primary partitions into 4 subpartitions (16p), each subpartition has weight of 16
	for (int ii=0;ii<16;++ii)
		{
		grid16p [ii] = 16;
		}
	
//Divide each of the 16 subpartitions into 4 subsubpartitions (64p), each subsubpartition has weight of 4
	for (int ii=0;ii<64;++ii)
		{
		grid64p [ii] = 4;
		}

//Finally the whole area is divided into 256 grids, with each grid weight of 1;

	// <<----- end of Grid initialization
	
	rng = new random_numbers::RandomNumberGenerator(); //instantiate random number generator
	goalLocation.theta = rng->uniformReal(0, 2 * M_PI); //set initial random heading
    
	targetDetected.data = -1; //initialize target detected
    
    //select initial search position 50 cm from center (0,0)
	goalLocation.x = 0.5 * cos(goalLocation.theta);
	goalLocation.y = 0.5 * sin(goalLocation.theta);

    if (argc >= 2) {
        publishedName = argv[1];
        cout << "Welcome to the world of tomorrow " << publishedName << "!  Mobility module started." << endl;
    } else {
        publishedName = hostname;
        cout << "No Name Selected. Default is: " << publishedName << endl;
    }

    // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
    ros::init(argc, argv, (publishedName + "_MOBILITY"), ros::init_options::NoSigintHandler);
    ros::NodeHandle mNH;

    signal(SIGINT, sigintEventHandler); // Register the SIGINT event handler so the node can shutdown properly

    joySubscriber = mNH.subscribe((publishedName + "/joystick"), 10, joyCmdHandler);
    modeSubscriber = mNH.subscribe((publishedName + "/mode"), 1, modeHandler);
    targetSubscriber = mNH.subscribe((publishedName + "/targets"), 10, targetHandler);
    obstacleSubscriber = mNH.subscribe((publishedName + "/obstacle"), 10, obstacleHandler);
    odometrySubscriber = mNH.subscribe((publishedName + "/odom/ekf"), 10, odometryHandler);
    targetsCollectedSubscriber = mNH.subscribe(("targetsCollected"), 10, targetsCollectedHandler);
   // Todo
    hostIDSubscriber = mNH.subscribe("hostID", 50, hostIDHandler);
	mapSubscriber = mNH.subscribe("mapSize", 100, mapHandler); 
    targetDetectedSubscriber = mNH.subscribe(("targetDetected"),10,targetDetectedHandler);
    
    targetDetectedPublish = mNH.advertise<mobility::target>(("targetDetected"),1,true);
    hostIDPublisher = mNH.advertise<std_msgs::String>(("hostID"), 50, true);
	mapPublisher = mNH.advertise<std_msgs::Float32MultiArray>(("mapSize"), 100, true);
    status_publisher = mNH.advertise<std_msgs::String>((publishedName + "/status"), 1, true);
    velocityPublish = mNH.advertise<geometry_msgs::Twist>((publishedName + "/velocity"), 10);
    stateMachinePublish = mNH.advertise<std_msgs::String>((publishedName + "/state_machine"), 1, true);
    targetCollectedPublish = mNH.advertise<std_msgs::Int16>(("targetsCollected"), 1, true);
    targetPickUpPublish = mNH.advertise<sensor_msgs::Image>((publishedName + "/targetPickUpImage"), 1, true);
    targetDropOffPublish = mNH.advertise<sensor_msgs::Image>((publishedName + "/targetDropOffImage"), 1, true);

    publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
    killSwitchTimer = mNH.createTimer(ros::Duration(killSwitchTimeout), killSwitchTimerEventHandler);
    stateMachineTimer = mNH.createTimer(ros::Duration(mobilityLoopTimeStep), mobilityStateMachine);
    ros::spin();
    
    return EXIT_SUCCESS;
}

void mobilityStateMachine(const ros::TimerEvent&) {
    std_msgs::String stateMachineMsg;


	// If never published the hostID, then publish once
    if(IDinit<0){
    IDmsg.data = publishedName;
    hostIDPublisher.publish(IDmsg); 
    IDinit = 1;
    }
	



    if (currentMode == 2 || currentMode == 3) { //Robot is in automode
		//If my ID has not been assigned, sort published hostID and assign a number
		if(myID < 0)
		{
			sort(hostID.begin(), hostID.begin()+IDcnt);			// IDcnt is the number of swarmies
			for (int i = 0; i<IDcnt; ++i)
			{
				if( publishedName.compare(hostID[i]) ==0 ) myID = i+1;
				
			}
		}

		//If a map exploration is done, then map_sidelength > 0, 
		//and hence regenerate the map grid according to the real size 		
		if ((map_sidelength > 0) && (grid_init == 1))
		{
			if(IDcnt ==3)		//3 Swarmies
			{
				grid_resol = 2*round(map_sidelength-1)/16; //grid resolution
				grid_num = 256;		//256 grids in total
			}else if(IDcnt ==6)		//6 Swarmies
			{	
				grid_resol = 2*round(map_sidelength-1)/32;
				grid_num = 1024;		//1024 grids in total
				
				//Grid initialization ----->>

				for (int ii=0;ii<4;++ii) {grid4p [ii] = 256;} 

				for (int ii=0;ii<16;++ii) {grid16p [ii] = 64;}

				for (int ii=0;ii<64;++ii) {grid64p [ii] = 16;}

				for (int ii=0;ii<256;++ii) {grid256p [ii] = 4;}	
			
				// <<end of Grid initialization

			}
			grid_init = 0;
		}



		switch(stateMachineState) {
			
			//Select rotation or translation based on required adjustment
			//If no adjustment needed, select new goal
			case STATE_MACHINE_TRANSFORM: {
				stateMachineMsg.data = "TRANSFORMING";
				//If angle between current and goal is significant
				if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > 0.1) {
					stateMachineState = STATE_MACHINE_ROTATE; //rotate
				}
				//If goal has not yet been reached
				else if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2) {

				
					stateMachineState = STATE_MACHINE_TRANSLATE; //translate
				}
				//If returning with a target
				else if (targetDetected.data != -1) {
					//If goal has not yet been reached
					if (hypot(0.0 - currentLocation.x, 0.0 - currentLocation.y) > 0.5) {
				        //set angle to center as goal heading
						goalLocation.theta = M_PI + atan2(currentLocation.y, currentLocation.x);
						
						//set center as goal position
						goalLocation.x = 0.0;
						goalLocation.y = 0.0;
					}
					//Otherwise, reset target and select new random uniform heading
					else {
						targetDetected.data = -1;
						
						goalLocation.theta = rng->uniformReal(0, 2 * M_PI);
						
					}
				}
				//Otherwise, assign a new goal
				else {  
					 //select new heading from Gaussian distribution around current heading

					// Start map exploration ----->>
	
					// If a swarmie is looking for map boundary, and its ID =1, 2 or 3
						if ((Look4Bdr ==1) && (myID <= 3 )) 
						{	
							if (myID == 1)
							{					
							  //Assign ID=1 to explore x>0 on x axis;
							
								
								
								if (ToScatter == 1)		//Scatter swarmies away from the origin at the beginning
								{
									goalLocation.theta = 0;
									goalLocation.x = 6.5*cos(goalLocation.theta);
									goalLocation.y = 6.5*sin(goalLocation.theta);
									if ( sqrt(pow(goalLocation.x - currentLocation.x,2)+pow(goalLocation.y - currentLocation.y,2))<2 ) ToScatter = 0;
								}else

								{	// If swarmies already scattered, assign the swarmie to different boundaries.
									goalLocation.x = ceil(currentLocation.x) + 1;
									goalLocation.y = 0;
									goalLocation.theta = atan2(goalLocation.y-currentLocation.y,goalLocation.x-currentLocation.x);
								}

							}else if (myID ==2)
							{
							
							  //Assign ID=2 to explore 45 degree direction;
	
								if (ToScatter == 1)
								{
									goalLocation.theta = M_PI/4;
									goalLocation.x = 6.5*cos(goalLocation.theta);
									goalLocation.y = 6.5*sin(goalLocation.theta);
									if ( sqrt(pow(goalLocation.x - currentLocation.x,2)+pow(goalLocation.y - currentLocation.y,2))<2 ) ToScatter = 0;
								}else
								{
									if(currentLocation.x > currentLocation.y)
									{
										goalLocation.x = ceil(currentLocation.x) +1;
										goalLocation.y = goalLocation.x;
										
									}else
									{
										goalLocation.y = ceil(currentLocation.y) +1;
										goalLocation.x = goalLocation.y;
									}
									

								}
									goalLocation.theta = atan2(goalLocation.y-currentLocation.y,goalLocation.x-currentLocation.x);

							}else
							{
								
							  //Assign ID=3 to explore y >0 on y axis
							

								if (ToScatter == 1)
								{
									goalLocation.theta = M_PI/2;
									goalLocation.x = 6.5*cos(goalLocation.theta);
									goalLocation.y = 6.5*sin(goalLocation.theta);
									if ( sqrt(pow(goalLocation.x - currentLocation.x,2)+pow(goalLocation.y - currentLocation.y,2))<2 ) ToScatter = 0;
								}else
								{
									goalLocation.x = 0;
									goalLocation.y = ceil(currentLocation.y)+1;
									goalLocation.theta = atan2(goalLocation.y-currentLocation.y,goalLocation.x-currentLocation.x);
								}
							}
						} // <<-----End of map exploration



						else{ //if myID = 1, 2, 3 and it is done with map exploration, or myID = 4, 5, 6 
						 
							int idx1024p = 0;
							int idx256p = 0;					
							int idx64p = 0;
							int idx16p = 0;
							int idx4p = 0;	
							int temp1024p [4];
							int temp256p [4];
							int temp64p [4];
							int temp16p [4];
							int temp4p [4];
							int tempID =0;
			
							// Assign all robots into three groups with a temporary ID
							
							tempID = 3-(myID % 3);

							// temp#p, # stands for a number (4, 16, 256, 1024), is used as a temporary grid ID for grid weight sorting
							
							for(int ii=0; ii<4;++ii) temp4p [ii] = grid4p [ii]; 
							
						
							// sort the grid weight in increasing order
							sort(temp4p,temp4p+4);

							// if there's no weight difference b/w the bottom two ranked partitions of 4p, 
							// move robots w/ tempID = 1 to top ranked 4p, (4p stands for the grid/partion whose area is 1/4 of the whole region)
							if((temp4p[1]-temp4p[0]<1) && (tempID ==1)){tempID = 3;}
						
							// Assign swarmie w/ tempID to 4p w/ the ranking number "tempID",
							// the larger the number is, the more weight a partition has.
							for(int ii=0; ii<4;++ii)
							{
						 		
								if(grid4p [ii] == temp4p[tempID])
									{
										idx4p = ii;		//Specify the index of the 4p, where swarmie w/ tempID goes to 
										break;					
									} 
							}
						
							if (temp4p[3]-temp4p[2]<1)	// if top 2 ranked 4p regions have no weight difference, i.e. no targets have been found
														// then apply random search
							{
								goalLocation.theta = rng->gaussian(currentLocation.theta, 0.5);
						
							//select new position 50 cm from current location
								goalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta));
								goalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta));
							}else if ((temp4p[2]-temp4p[1]<1) && (tempID==2))	// At least 1 target has been detected
								
							{
								goalLocation.theta = rng->gaussian(currentLocation.theta, 0.5);
						
							//select new position 50 cm from current location
								goalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta));
								goalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta));
							}
							else{
								// Try to locate the subpartition where the target is.								

								for(int ii=0; ii<4;++ii)
								{
									temp16p [ii] = grid16p [idx4p*4+ii]; 
								}
								
								sort(temp16p,temp16p+4);
								
								// Generally, there are two swarmies (1st: myID <= 3, 2nd: myID >3) 		
								// in charge of a 4p region with the same index idx4p. To void frequent collision,
								// send these two swarmies into two different subpartitions (16p) 
		
								if (myID <=3 )
								{
								
									for(int ii=0; ii<4;++ii)
									{
										if(grid16p [idx4p*4+ii] == temp16p[3])
										{
											idx16p = ii;	//index of 16p subpartition for the swarmie w/ myID <=3
											break;					
										} 
									}
								}else
								{
									for(int ii=0; ii<4;++ii)
									{
										if(grid16p [idx4p*4+ii] == temp16p[2])
										{
											idx16p = ii;	//index of 16p subpartition for the swarmie w/ myID >3
											break;					
										} 
									}
								}

								// If the swarmie w/ myID > 3, and there's no target found in its 16p region,
								// then apply a random search

								if((temp16p[2]-temp16p[1]<1) && (myID>3))
								{
									goalLocation.theta = rng->gaussian(currentLocation.theta, 0.5);
	
									//select new position 50 cm from current location
  			
									goalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta));
									goalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta));
								}else 
								{
									// Specifity more accurate subsubpartion (64p) where a swarmie should go to
									for(int ii=0; ii<4;++ii)
									{
										temp64p [ii] = grid64p [16*idx4p+4*idx16p+ii]; 
									}
						
									sort(temp64p,temp64p+4);
	
									// Send the swarmie to the top weight subsubpartition (64p)
									for(int ii=0; ii<4;++ii)
									{
						 				if(grid64p [16*idx4p+4*idx16p+ii] == temp64p[3])
										{
											idx64p = ii;	//index of 64p subsubpartition for the swarmie
											break;					
										} 
									}
						
									// Specifity more accurate subsubpartion (256p) where a swarmie should go to
									for(int ii=0; ii<4;++ii)
									{
										temp256p [ii] = grid256p [64*idx4p+16*idx16p+4*idx64p+ii]; 
									}
						
									sort(temp256p,temp256p+4);
	
									for(int ii=0; ii<4;++ii)
									{
						 				if(grid256p [64*idx4p+16*idx16p+4*idx64p+ii] == temp256p[3])
										{
											idx256p = ii;	//index of 256p subsubsubpartition for the swarmie
											break;					
										} 
									}

									//if grid number = 1024, i.e. final competition size
									if (grid_num == 1024){
										// Specifity more accurate subsubpartion (1024p) where a swarmie should go to							
										for(int ii=0; ii<4;++ii)
										{
											temp1024p [ii] = grid1024p [256*idx4p+64*idx16p+16*idx64p+4*idx256p+ii]; 
										}
						
										sort(temp1024p,temp1024p+4);
	
										for(int ii=0; ii<4;++ii)
										{
							 				if(grid1024p [256*idx4p+64*idx16p+16*idx64p+4*idx256p+ii] == temp1024p[3])
											{
												idx1024p = ii;	//index of 1024p subsubsubsubpartition for the swarmie
												break;					
											} 
										}	
									}
	

									// Start to locate the target from grid index ----->>
									float tempa = rng->gaussian(currentLocation.theta, 0.25);
									float gx = 0;
									float gy = 0;
									// if preliminary competition 

									if(grid_num ==256)
									{		      

										// For example, given an index number idx64p, this 64p region contains 4 subpartitions of 256p, 
										//similar to 4 quadrants of a coordinate plane.
										// if idx64p = ii, it refers to quadrant ii+1, and so on, with ii = {0,1,2,3}.
										// (M_PI/4 + idx64p*M_PI/2) locates the center of the 64p from the center of the upper level partion 16p;
										// (M_PI/4 + idx256p*M_PI/2) locates the center of the 256p from the center of the 64p with index idx64p
		  
										gx = 4*grid_resol*sqrt(2)*cos(M_PI/4+idx4p*M_PI_2)+2*grid_resol*sqrt(2)*cos(M_PI/4+idx16p*M_PI_2)+grid_resol*sqrt(2)*cos(M_PI/4+idx64p*M_PI_2)+grid_resol/2*sqrt(2)*cos(M_PI/4+idx256p*M_PI_2)+0.5*cos(tempa); 
										// tempa is an angle for random walk = 0.5 m, starting from the target location. 
										// Steps: 1st. go back to the grid where most targets have been detected; 2nd. apply a random walk of 0.5 m from the grid,
										// since grid resolution cannot specify the exact location of a target. And this strategy can help to find more targets. 
										// If the random walk w/ the angle tempa removed, a robot sometimes has trouble locating the target and gets stuck there. 
										gy = 4*grid_resol*sqrt(2)*sin(M_PI/4+idx4p*M_PI_2)+2*grid_resol*sqrt(2)*sin(M_PI/4+idx16p*M_PI_2)+grid_resol*sqrt(2)*sin(M_PI/4+idx64p*M_PI_2)+grid_resol/2*sqrt(2)*sin(M_PI/4+idx256p*M_PI_2)+0.5*sin(tempa);

									}else if(grid_num ==1024) // if final competition
									{
										gx = 8*grid_resol*sqrt(2)*cos(M_PI/4+idx4p*M_PI_2)+4*grid_resol*sqrt(2)*cos(M_PI/4+idx16p*M_PI_2)+2*grid_resol*sqrt(2)*cos(M_PI/4+idx64p*M_PI_2)+grid_resol*sqrt(2)*cos(M_PI/4+idx256p*M_PI_2)+0.5*grid_resol*sqrt(2)*cos(M_PI/4+idx1024p*M_PI_2)+0.5*cos(tempa);
										gy = 8*grid_resol*sqrt(2)*sin(M_PI/4+idx4p*M_PI_2)+4*grid_resol*sqrt(2)*sin(M_PI/4+idx16p*M_PI_2)+2*grid_resol*sqrt(2)*sin(M_PI/4+idx64p*M_PI_2)+grid_resol*sqrt(2)*sin(M_PI/4+idx256p*M_PI_2)+0.5*grid_resol*sqrt(2)*sin(M_PI/4+idx1024p*M_PI_2)+0.5*sin(tempa);
									}

									// <<----- End of locating the target from grid index
									goalLocation.theta = atan2(gy - currentLocation.y, gx - currentLocation.x);
									// Heading to the top weight grid step by step, so it is able to detect tags on its way.
									goalLocation.x = currentLocation.x + (0.75 * cos(goalLocation.theta));
									goalLocation.y = currentLocation.y + (0.75 * sin(goalLocation.theta));		
								}
	
							}
						}
		
					}
				
				//Purposefully fall through to next case without breaking
			}
			
			//Calculate angle between currentLocation.theta and goalLocation.theta
			//Rotate left or right depending on sign of angle
			//Stay in this state until angle is minimized
			case STATE_MACHINE_ROTATE: {
				stateMachineMsg.data = "ROTATING";
			    if (angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta) > 0.1) {
					setVelocity(0.0, 0.2); //rotate left //0.2
			    }
			    else if (angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta) < -0.1) {
					setVelocity(0.0, -0.2); //rotate right //0.2
				}
				else {
					setVelocity(0.0, 0.0); //stop
					stateMachineState = STATE_MACHINE_TRANSLATE; //move to translate step
				}
			    break;
			}
			
			//Calculate angle between currentLocation.x/y and goalLocation.x/y
			//Drive forward
			//Stay in this state until angle is at least PI/2
			case STATE_MACHINE_TRANSLATE: {
				stateMachineMsg.data = "TRANSLATING";
				
				if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2) {

					
					setVelocity(0.3, 0.0);			
				}
				else {
					setVelocity(0.0, 0.0); //stop
				
			
					stateMachineState = STATE_MACHINE_TRANSFORM; //move back to transform step
				}
			    break;
			}
		
			default: {
			    break;
			}
		}
	}

    else { // mode is NOT auto

        // publish current state for the operator to see
        stateMachineMsg.data = "WAITING";
    }

    // publish state machine string for user, only if it has changed, though
    if (strcmp(stateMachineMsg.data.c_str(), prev_state_machine) != 0) {
        stateMachinePublish.publish(stateMachineMsg);
        sprintf(prev_state_machine, "%s", stateMachineMsg.data.c_str());
    }
}

void setVelocity(double linearVel, double angularVel) 
{
  // Stopping and starting the timer causes it to start counting from 0 again.
  // As long as this is called before the kill swith timer reaches killSwitchTimeout seconds
  // the rover's kill switch wont be called.
  //killSwitchTimer.stop();
  //killSwitchTimer.start();


  velocity.linear.x = linearVel * 1.5;
  velocity.angular.z = angularVel * 8; //scaling factor for sim; removed by aBridge node

  velocityPublish.publish(velocity);

}

/***********************
 * ROS CALLBACK HANDLERS
 ************************/

void targetHandler(const shared_messages::TagsImage::ConstPtr& message) {
	mobility::target target_detail;


	//if this is the goal target
	if (message->tags.data[0] == 256) {
		//if we were returning with a target
	    if (targetDetected.data != -1) {
			//publish to scoring code
			targetDropOffPublish.publish(message->image);
			targetDetected.data = -1;
	    }
	}

	//if target has not previously been detected (if target not carry a target)

	//	&& if (myID<=3 and done with map exploration) or (myID >3) 
	else if (targetDetected.data == -1 && ( ( (Look4Bdr == 0) && (myID<=3)) || (myID>3))) {  //free hand
   
					
			//if (targetsCollected[message->tags.data[0]]) {// no more target in this grid!!
		if (targetsCollected[message->tags.data[0]])
		{
			dropcnt+=1;		
			// If targets in this grid have all been collected, dropcnt +=1 and later this coefficient will decrease the weight of this grid 
			// And dropcnt will be cleared once the robot collects a target

			cluster_cnt =0;	

			int idx4p = 0;
			int idx16p = 0;
			int idx64p = 0;
			int idx256p =0;
			int idx1024p = 0;
			

			float tempx;
			float tempy;

			float tempa4p;
			float tempa16p;
			float tempa64p;	
			float tempa256p;
			float tempa1024p;
			tempx = goalLocation.x;
			tempy = goalLocation.y;
			
			// Start to identify the grid index from the target location (256)----->>
			// This is the inverse procedure of locating the target from grid index 		
			if (grid_num == 256)
			{

				tempa4p = atan2(tempy,tempx);
				// atan2 returns an angle in [-pi, pi], so map it to [0, 2pi]
				if (tempa4p < 0) {tempa4p = 2*M_PI+tempa4p;}
				idx4p = floor(tempa4p/(M_PI_2));

				
				tempa16p = atan2(tempy-4*grid_resol*sqrt(2)*sin(M_PI/4+idx4p*M_PI_2),tempx-4*grid_resol*sqrt(2)*cos(M_PI/4+idx4p*M_PI_2));
				if (tempa16p < 0) {tempa16p = 2*M_PI+tempa16p;}		
				idx16p = floor(tempa16p/(M_PI_2));


				tempa64p = atan2(tempy-(4*grid_resol*sqrt(2)*sin(M_PI/4+idx4p*M_PI_2)+2*grid_resol*sqrt(2)*sin(M_PI/4+idx16p*M_PI_2)),tempx-(4*grid_resol*sqrt(2)*cos(M_PI/4+idx4p*M_PI_2)+2*grid_resol*sqrt(2)*cos(M_PI/4+idx16p*M_PI_2)));	
				if (tempa64p < 0) {tempa64p = 2*M_PI+tempa64p;}		
				idx64p = floor(tempa64p/(M_PI_2));


				tempa256p = atan2(tempy-(4*grid_resol*sqrt(2)*sin(M_PI/4+idx4p*M_PI_2)+2*grid_resol*sqrt(2)*sin(M_PI/4+idx16p*M_PI_2)+grid_resol*sqrt(2)*sin(M_PI/4+idx64p*M_PI_2)),tempx-(4*grid_resol*sqrt(2)*cos(M_PI/4+idx4p*M_PI_2)+2*grid_resol*sqrt(2)*cos(M_PI/4+idx16p*M_PI_2)+grid_resol*sqrt(2)*cos(M_PI/4+idx64p*M_PI_2)));			
				if (tempa256p < 0) {tempa256p = 2*M_PI+tempa256p;}
				idx256p = floor(tempa256p/(M_PI_2));


				//Use subst1 and subst2 to record the grid weight change
				int subst1 = grid256p [64*idx4p+16*idx16p+4*idx64p+idx256p];
				int subst2 = 0;

				grid256p [64*idx4p+16*idx16p+4*idx64p+idx256p] -=3*dropcnt;		
				// failing to collect a target in the desired region will lead to grid weight drop

				if (grid256p [64*idx4p+16*idx16p+4*idx64p+idx256p]<0) grid256p [64*idx4p+16*idx16p+4*idx64p+idx256p]=0;
			
				subst1 -= grid256p [64*idx4p+16*idx16p+4*idx64p+idx256p];	
		
				for (int ii=0;ii<4;++ii)
				{
					subst2 += grid256p [64*idx4p+16*idx16p+4*idx64p+ii];
					grid256p [64*idx4p+16*idx16p+4*idx64p+ii] -= dropcnt;	
					// failing to collect a target in the desired region will lead to grid weight drop in the neighborhood
		
					if(grid256p [64*idx4p+16*idx16p+4*idx64p+ii]<0)	grid256p [64*idx4p+16*idx16p+4*idx64p+ii]=0;		

			 		subst2 -= grid256p [64*idx4p+16*idx16p+4*idx64p+ii];
				}
	

				grid64p [16*idx4p+4*idx16p+idx64p] -=subst1+subst2; 
		
			  	if(grid64p [16*idx4p+4*idx16p+idx64p]<0)	grid64p [16*idx4p+4*idx16p+idx64p] =0;
	
				grid16p [4*idx4p+idx16p] -= subst1+subst2;

				if(grid16p [4*idx4p+idx16p]<0)	grid16p [4*idx4p+idx16p]=0;
		
				grid4p [idx4p] -= subst1+subst2;
	
				if(grid4p [idx4p]<0)	grid4p [idx4p] =0;
			// <<----- End of identifying the grid index from the target location (256)

			}else if(grid_num == 1024)
			{
			// Start to identify the grid index from the target location (1024)----->>	

				tempa4p = atan2(tempy,tempx);
				if (tempa4p < 0) {tempa4p = 2*M_PI+tempa4p;}
				idx4p = floor(tempa4p/(M_PI_2));

				tempa16p = atan2(tempy-8*grid_resol*sqrt(2)*sin(M_PI/4+idx4p*M_PI_2),tempx-8*grid_resol*sqrt(2)*cos(M_PI/4+idx4p*M_PI_2));
				if (tempa16p < 0) {tempa16p = 2*M_PI+tempa16p;}		
				idx16p = floor(tempa16p/(M_PI_2));

				tempa64p = atan2(tempy-(8*grid_resol*sqrt(2)*sin(M_PI/4+idx4p*M_PI_2)+4*grid_resol*sqrt(2)*sin(M_PI/4+idx16p*M_PI_2)),tempx-(8*grid_resol*sqrt(2)*cos(M_PI/4+idx4p*M_PI_2)+4*grid_resol*sqrt(2)*cos(M_PI/4+idx16p*M_PI_2)));
				if (tempa64p < 0) {tempa64p = 2*M_PI+tempa64p;}		
				idx64p = floor(tempa64p/(M_PI_2));
		
				tempa256p = atan2(tempy-(8*grid_resol*sqrt(2)*sin(M_PI/4+idx4p*M_PI_2)+4*grid_resol*sqrt(2)*sin(M_PI/4+idx16p*M_PI_2)+2*grid_resol*sqrt(2)*sin(M_PI/4+idx64p*M_PI_2)),tempx-(8*grid_resol*sqrt(2)*cos(M_PI/4+idx4p*M_PI_2)+4*grid_resol*sqrt(2)*cos(M_PI/4+idx16p*M_PI_2)+2*grid_resol*sqrt(2)*cos(M_PI/4+idx64p*M_PI_2)));
				if (tempa256p < 0) {tempa256p = 2*M_PI+tempa256p;}			
				idx256p = floor(tempa256p/(M_PI_2));

				tempa1024p = atan2(tempy-(8*grid_resol*sqrt(2)*sin(M_PI/4+idx4p*M_PI_2)+4*grid_resol*sqrt(2)*sin(M_PI/4+idx16p*M_PI_2)+2*grid_resol*sqrt(2)*sin(M_PI/4+idx64p*M_PI_2)+grid_resol*sqrt(2)*sin(M_PI/4+idx256p*M_PI_2)),tempx-(8*grid_resol*sqrt(2)*cos(M_PI/4+idx4p*M_PI_2)+4*grid_resol*sqrt(2)*cos(M_PI/4+idx16p*M_PI_2)+2*grid_resol*sqrt(2)*cos(M_PI/4+idx64p*M_PI_2)+grid_resol*sqrt(2)*cos(M_PI/4+idx256p*M_PI_2) ) );
				if (tempa1024p < 0) {tempa1024p = 2*M_PI+tempa1024p;}
				idx1024p = floor(tempa1024p/(M_PI_2));

				int subst1 = grid1024p [256*idx4p+64*idx16p+16*idx64p+4*idx256p+idx1024p];
				int subst2 = 0;

				grid1024p [256*idx4p+64*idx16p+16*idx64p+4*idx256p+idx1024p] -=3*dropcnt;
				if (grid1024p [256*idx4p+64*idx16p+16*idx64p+4*idx256p+idx1024p] <0) grid1024p [256*idx4p+64*idx16p+16*idx64p+4*idx256p+idx1024p] = 0;
							

				subst1 -= grid1024p [256*idx4p+64*idx16p+16*idx64p+4*idx256p+idx1024p];	
		
				for (int ii=0;ii<4;++ii)
				{
					subst2 += grid1024p [256*idx4p+64*idx16p+16*idx64p+4*idx256p+ii];
					grid1024p [256*idx4p+64*idx16p+16*idx64p+4*idx256p+ii] -= dropcnt;		
		
					if(grid1024p [256*idx4p+64*idx16p+16*idx64p+4*idx256p+ii]<0) grid1024p [256*idx4p+64*idx16p+16*idx64p+4*idx256p+ii]=0;		

			 		subst2 -= grid1024p [256*idx4p+64*idx16p+16*idx64p+4*idx256p+ii];
				}
				

				grid256p [64*idx4p+16*idx16p+4*idx64p+idx256p] -=subst1+subst2;
				if(grid256p [64*idx4p+16*idx16p+4*idx64p+idx256p]<0) grid256p [64*idx4p+16*idx16p+4*idx64p+idx256p] =0;
				
				grid64p [16*idx4p+4*idx16p+idx64p] -=subst1+subst2;  		
			  	if(grid64p [16*idx4p+4*idx16p+idx64p]<0) grid64p [16*idx4p+4*idx16p+idx64p] =0;
	
				grid16p [4*idx4p+idx16p] -= subst1+subst2;
				if(grid16p [4*idx4p+idx16p]<0)	grid16p [4*idx4p+idx16p]=0;
		
				grid4p [idx4p] -= subst1+subst2;	
				if(grid4p [idx4p]<0) grid4p [idx4p] =0;
			// <<----- End of identifying the grid index from the target location (1024)
			}
		}else if (!targetsDetected[message->tags.data[0]]) 
		{
			
			//publish target details to other robot
			//target location use robot's current location
			target_detail.ID = message->tags.data[0];
			
			target_detail.x = currentLocation.x + CAMERA_DISTANCE*cos(currentLocation.theta);
			target_detail.y = currentLocation.y+ CAMERA_DISTANCE*sin(currentLocation.theta);
	
			targetDetectedPublish.publish(target_detail);
			
			goalLocation.theta = rng->gaussian(currentLocation.theta, 0.5); //if detect a target, look around for more
			
		}	
	
        //check if target has not yet been collected
		else
		{            
			dropcnt = 0;
			//copy target ID to class variable
			targetDetected.data = message->tags.data[0];
			
	        //set angle to center as goal heading
			goalLocation.theta = M_PI + atan2(currentLocation.y, currentLocation.x);
			
			//set center as goal position
			goalLocation.x = 0.0;
			goalLocation.y = 0.0;
			
			//publish detected target
			targetCollectedPublish.publish(targetDetected);

			//publish to scoring code
			targetPickUpPublish.publish(message->image);

			
			//switch to transform state to trigger return to center
			stateMachineState = STATE_MACHINE_TRANSFORM;
			SearchTimeCnt =0;
		}
	 
		
	}    

}

void modeHandler(const std_msgs::UInt8::ConstPtr& message) {
	currentMode = message->data;
	setVelocity(0.0, 0.0);
}

void obstacleHandler(const std_msgs::UInt8::ConstPtr& message) {
	if (message->data > 0) {
		
		// If robot is identifying the map boundary
		if ((ToScatter == 0) && (Look4Bdr == 1))
		{
					
			ObstCnt += 1;
			if(ObstCnt >15)	// If a map-exploerer swarmie is scattered, and its obstacle handler has been called>15, then claim that it has detected the map boundary 
			{
				Look4Bdr = 0;
				mapMsg.data [0] = myID;
				mapMsg.data [1] = currentLocation.x;
				mapMsg.data [2] = currentLocation.y;				
				mapMsg.data [3] = IDcnt;	
				// Done with the boundary search
				mapPublisher.publish(mapMsg);
			}
				
		}
			
		

		//obstacle on right side
		if (message->data == 1) {
		
			//select new heading 0.2 radians to the left
			goalLocation.theta = currentLocation.theta + 0.2;

		}
		
		//obstacle in front or on left side
		else if (message->data == 2) {
	
			//select new heading 0.2 radians to the right
			goalLocation.theta = currentLocation.theta - 0.2;
		}
							
		//select new position 50 cm from current location
		
		goalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta));
		goalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta));
		
		//switch to transform state to trigger collision avoidance
		stateMachineState = STATE_MACHINE_TRANSFORM;
	}
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& message) {
	//Get (x,y) location directly from pose
	currentLocation.x = message->pose.pose.position.x;
	currentLocation.y = message->pose.pose.position.y;
	
	//Get theta rotation by converting quaternion orientation to pitch/roll/yaw
	tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	currentLocation.theta = yaw;
}

void joyCmdHandler(const geometry_msgs::Twist::ConstPtr& message) {
    if (currentMode == 0 || currentMode == 1) 
      {
	setVelocity(message->linear.x, message->angular.z);
      } 
}


void publishStatusTimerEventHandler(const ros::TimerEvent&)
{
  std_msgs::String msg;
  msg.data = "online";
  status_publisher.publish(msg);
}

// Safety precaution. No movement commands - might have lost contact with ROS. Stop the rover.
// Also might no longer be receiving manual movement commands so stop the rover.
void killSwitchTimerEventHandler(const ros::TimerEvent& t)
{
  // No movement commands for killSwitchTime seconds so stop the rover 
  setVelocity(0,0);
  double current_time = ros::Time::now().toSec();
  ROS_INFO("In mobility.cpp:: killSwitchTimerEventHander(): Movement input timeout. Stopping the rover at %6.4f.", current_time);
}

void targetsCollectedHandler(const std_msgs::Int16::ConstPtr& message) {
	targetsCollected[message->data] = 1;
}
//Todo




void targetDetectedHandler(const mobility::target target_msg) {
	targetsDetected[target_msg.ID] = 1;
	targetsDetected_x[target_msg.ID] = target_msg.x;
	targetsDetected_y[target_msg.ID] = target_msg.y;
	int idx4p = 0;
	int idx16p = 0;
	int idx64p = 0;
	int idx256p = 0;
	int idx1024p = 0;	


	float tempx;
	float tempy;

	float tempa4p;
	float tempa16p;
	float tempa64p;	
	float tempa256p;
	float tempa1024p;
	tempx = target_msg.x;
	tempy = target_msg.y;
	// Start to identify grid index from target location (256)----->>
	if (grid_num == 256)
	{
		tempa4p = atan2(tempy,tempx);		
		if (tempa4p < 0) {tempa4p = 2*M_PI+tempa4p;}
		idx4p = floor(tempa4p/(M_PI_2));

		tempa16p = atan2(tempy-4*grid_resol*sqrt(2)*sin(M_PI/4+idx4p*M_PI_2),tempx-4*grid_resol*sqrt(2)*cos(M_PI/4+idx4p*M_PI_2));			
		if (tempa16p < 0) {tempa16p = 2*M_PI+tempa16p;}		
		idx16p = floor(tempa16p/(M_PI_2));

		tempa64p = atan2(tempy-(4*grid_resol*sqrt(2)*sin(M_PI/4+idx4p*M_PI_2)+2*grid_resol*sqrt(2)*sin(M_PI/4+idx16p*M_PI_2)),tempx-(4*grid_resol*sqrt(2)*cos(M_PI/4+idx4p*M_PI_2)+2*grid_resol*sqrt(2)*cos(M_PI/4+idx16p*M_PI_2)));	
		if (tempa64p < 0) {tempa64p = 2*M_PI+tempa64p;}
		idx64p = floor(tempa64p/(M_PI_2));		

		tempa256p = atan2(tempy-(4*grid_resol*sqrt(2)*sin(M_PI/4+idx4p*M_PI_2)+2*grid_resol*sqrt(2)*sin(M_PI/4+idx16p*M_PI_2)+grid_resol*sqrt(2)*sin(M_PI/4+idx64p*M_PI_2)),tempx-(4*grid_resol*sqrt(2)*cos(M_PI/4+idx4p*M_PI_2)+2*grid_resol*sqrt(2)*cos(M_PI/4+idx16p*M_PI_2)+grid_resol*sqrt(2)*cos(M_PI/4+idx64p*M_PI_2)));
		if (tempa256p < 0) {tempa256p = 2*M_PI+tempa256p;}			
		idx256p = floor(tempa256p/(M_PI_2));
		// <<----- End of indentifying the grid index (256)
		cluster_cnt += 1;	// Successive discovery of targets in one region will lead to larger cluster_cnt,
							// and later this coefficient will add weight to grid weight. cluster_cnt will be 
							// cleared once the swarmie collects nothing in this region.

		grid256p [64*idx4p+16*idx16p+4*idx64p+idx256p] +=6+3*cluster_cnt;	
		// Succesfully collecting a target in the desired region will lead to grid weight increasing
	
		for (int ii=0;ii<4;++ii)
		{		 
			 grid256p [64*idx4p+16*idx16p+4*idx64p+ii] += cluster_cnt;
			// Succesfully collecting a target in the desired region will lead to grid weight increasing in the neighborhood		
		}
	
		grid64p [16*idx4p+4*idx16p+idx64p] +=6+7*cluster_cnt; 
	  	grid16p [4*idx4p+idx16p] += 6+7*cluster_cnt;
		grid4p [idx4p] += 6+7*cluster_cnt;


	}else if (grid_num == 1024)
	// Start to identify grid index from target location (1024)----->>
	{
		tempa4p = atan2(tempy,tempx);			
		if (tempa4p < 0) {tempa4p = 2*M_PI+tempa4p;}
		idx4p = floor(tempa4p/(M_PI_2));

		tempa16p = atan2(tempy-8*grid_resol*sqrt(2)*sin(M_PI/4+idx4p*M_PI_2),tempx-8*grid_resol*sqrt(2)*cos(M_PI/4+idx4p*M_PI_2));			
		if (tempa16p < 0) {tempa16p = 2*M_PI+tempa16p;}		
		idx16p = floor(tempa16p/(M_PI_2));

		tempa64p = atan2(tempy-(8*grid_resol*sqrt(2)*sin(M_PI/4+idx4p*M_PI_2)+4*grid_resol*sqrt(2)*sin(M_PI/4+idx16p*M_PI_2)),tempx-(8*grid_resol*sqrt(2)*cos(M_PI/4+idx4p*M_PI_2)+4*grid_resol*sqrt(2)*cos(M_PI/4+idx16p*M_PI_2)));
		if (tempa64p < 0) {tempa64p = 2*M_PI+tempa64p;}		
		idx64p = floor(tempa64p/(M_PI_2));		

		tempa256p = atan2(tempy-(8*grid_resol*sqrt(2)*sin(M_PI/4+idx4p*M_PI_2)+4*grid_resol*sqrt(2)*sin(M_PI/4+idx16p*M_PI_2)+2*grid_resol*sqrt(2)*sin(M_PI/4+idx64p*M_PI_2)),tempx-(8*grid_resol*sqrt(2)*cos(M_PI/4+idx4p*M_PI_2)+4*grid_resol*sqrt(2)*cos(M_PI/4+idx16p*M_PI_2)+2*grid_resol*sqrt(2)*cos(M_PI/4+idx64p*M_PI_2)));
		if (tempa256p < 0) {tempa256p = 2*M_PI+tempa256p;}	
		idx256p = floor(tempa256p/(M_PI_2));

		tempa1024p = atan2(tempy-(8*grid_resol*sqrt(2)*sin(M_PI/4+idx4p*M_PI_2)+4*grid_resol*sqrt(2)*sin(M_PI/4+idx16p*M_PI_2)+2*grid_resol*sqrt(2)*sin(M_PI/4+idx64p*M_PI_2)+grid_resol*sqrt(2)*sin(M_PI/4+idx256p*M_PI_2)),tempx-(8*grid_resol*sqrt(2)*cos(M_PI/4+idx4p*M_PI_2)+4*grid_resol*sqrt(2)*cos(M_PI/4+idx16p*M_PI_2)+2*grid_resol*sqrt(2)*cos(M_PI/4+idx64p*M_PI_2)+grid_resol*sqrt(2)*cos(M_PI/4+idx256p*M_PI_2) ) );			
		if (tempa1024p < 0) {tempa1024p = 2*M_PI+tempa1024p;}
		idx1024p = floor(tempa1024p/(M_PI_2));
		// <<----- End of indentifying the grid index (1024)
		cluster_cnt += 1;


		grid1024p [256*idx4p+64*idx16p+16*idx64p+4*idx256p+idx1024p] +=6+3*cluster_cnt;
	
		for (int ii=0;ii<4;++ii)
		{		 
			 grid1024p [256*idx4p+64*idx16p+16*idx64p+4*idx256p+ii] += cluster_cnt;
		}
		
		grid256p [64*idx4p+16*idx16p+4*idx64p+idx256p] +=6+7*cluster_cnt;
		grid64p [16*idx4p+4*idx16p+idx64p] +=6+7*cluster_cnt; 
	  	grid16p [4*idx4p+idx16p] += 6+7*cluster_cnt;
		grid4p [idx4p] += 6+7*cluster_cnt;		

	}


}
		

void mapHandler(const std_msgs::Float32MultiArray::ConstPtr& mapMsg)
{
	mapPioneer += 1; // If mapPionner = 3, then robots are done with map exploration
	int temp_x;
	int temp_y;	
	temp_x = mapMsg->data [1];
	temp_y = mapMsg->data [2];
	
	if (fabs(mapMsg->data [0] -1) < 0.2) //myID = 1 		
	{	
		//Computing half length of the side 'a' of the rectangular map area (close to a square)	
		rect_a = sqrt(pow(temp_x,2)+pow(temp_y,2));
	}else if((fabs(mapMsg->data [0] -2) < 0.2)) //myId = 2
	{
			
		rect_c = sqrt(pow(temp_x,2)+pow(temp_y,2)); 
	}else if((fabs(mapMsg->data [0] -3) < 0.2)) //myId = 3
	{
		//Computing half length of the side 'b' of the rectangular map area (close to a square)	
		rect_b = sqrt(pow(temp_x,2)+pow(temp_y,2)); 
	}
	
	if(mapPioneer ==3)
	{
		// Geometry transformation
		float temp45side;
		temp45side = sqrt(pow(rect_b,2)+pow(rect_c,2)-2*rect_b*rect_c*cos(M_PI/4));			
		map_sidelength = rect_b*rect_c*sin(M_PI/4)/temp45side;
		// map_sidelength stands for half length of the map side, using the short side of the rectangle to make a square map area
		if(rect_a<rect_b) map_sidelength *=rect_a/rect_b;
	}
}


void hostIDHandler(const std_msgs::String::ConstPtr& message){
  	
	
	hostID [IDcnt] = message->data;
	IDcnt += 1;
}


void sigintEventHandler(int sig)
{
     // All the default sigint handler does is call shutdown()
     ros::shutdown();
}
