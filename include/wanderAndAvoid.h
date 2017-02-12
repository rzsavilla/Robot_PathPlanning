// Signatures

class FSM: public ArAction // Class action inherits from ArAction
{
	enum State
	{
		forward,turnLeft,turnRight,randomTurn,turnAround
	};

private: 
	int getRand(int min, int max);

 public:
   FSM(); // Constructor
   virtual ~FSM() {}  // Destructor
   virtual ArActionDesired * fire(ArActionDesired d); // Body of the action
   ArActionDesired desiredState; // Holds state of the robot that we wish to action

   State state;

   bool bTurning;
   float fSpeed;
   float fTurnSpeed;
   float fDesiredAngle;

   int iTick;
   int iRandTick;
};
