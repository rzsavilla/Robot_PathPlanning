/*
MobileRobots Advanced Robotics Interface for Applications (ARIA)
Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006, 2007, 2008, 2009 MobileRobots Inc.
Copyright (C) 2010, 2011 Adept Technology, Inc.

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
Adept MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
Adept MobileRobots, 10 Columbia Drive, Amherst, NH 03031; 800-639-9481
*/
#include "ArExport.h"
#include "ariaOSDef.h"
#include "ArLMS1XX.h"
#include "ArRobot.h"
#include "ArSerialConnection.h"
#include "ariaInternal.h"
#include <time.h>

AREXPORT ArLMS1XXPacket::ArLMS1XXPacket() : 
  ArBasePacket(10000, 1, NULL, 1)
{
  myFirstAdd = true;
  myCommandType[0] = '\0';
  myCommandName[0] = '\0';
}

AREXPORT ArLMS1XXPacket::~ArLMS1XXPacket()
{

}

AREXPORT const char *ArLMS1XXPacket::getCommandType(void)
{
  return myCommandType;
}

AREXPORT const char *ArLMS1XXPacket::getCommandName(void)
{
  return myCommandName;
}


AREXPORT void ArLMS1XXPacket::finalizePacket(void)
{
  myBuf[0] = '\002';
  rawCharToBuf('\003');
  myBuf[myLength] = '\0';
}
 
AREXPORT void ArLMS1XXPacket::resetRead(void)
{
  myReadLength = 1;

  myCommandType[0] = '\0';
  myCommandName[0] = '\0';

  bufToStr(myCommandType, sizeof(myCommandType));
  bufToStr(myCommandName, sizeof(myCommandName));
}
  
AREXPORT ArTime ArLMS1XXPacket::getTimeReceived(void)
{
  return myTimeReceived;
}

AREXPORT void ArLMS1XXPacket::setTimeReceived(ArTime timeReceived)
{
  myTimeReceived = timeReceived;
}

AREXPORT void ArLMS1XXPacket::duplicatePacket(ArLMS1XXPacket *packet)
{
  myLength = packet->getLength();
  myReadLength = packet->getReadLength();
  myTimeReceived = packet->getTimeReceived();
  myFirstAdd = packet->myFirstAdd;
  strcpy(myCommandType, packet->myCommandType);
  strcpy(myCommandName, packet->myCommandName);
  memcpy(myBuf, packet->getBuf(), myLength);
}

AREXPORT void ArLMS1XXPacket::empty(void)
{
  myLength = 0;
  myReadLength = 0;
  myFirstAdd = false;
  myCommandType[0] = '\0';
  myCommandName[0] = '\0';
}


AREXPORT void ArLMS1XXPacket::byteToBuf(ArTypes::Byte val)
{
  char buf[1024];
  if (val > 0)
    sprintf(buf, "+%d", val);
  else
    sprintf(buf, "%d", val);
  strToBuf(buf);
}

AREXPORT void ArLMS1XXPacket::byte2ToBuf(ArTypes::Byte2 val)
{
  char buf[1024];
  if (val > 0)
    sprintf(buf, "+%d", val);
  else
    sprintf(buf, "%d", val);
  strToBuf(buf);
}

AREXPORT void ArLMS1XXPacket::byte4ToBuf(ArTypes::Byte4 val)
{
  char buf[1024];
  if (val > 0)
    sprintf(buf, "+%d", val);
  else
    sprintf(buf, "%d", val);
  strToBuf(buf);
}

AREXPORT void ArLMS1XXPacket::uByteToBuf(ArTypes::UByte val)
{
  char buf[1024];
  sprintf(buf, "%u", val);
  strToBuf(buf);
}

AREXPORT void ArLMS1XXPacket::uByte2ToBuf(ArTypes::UByte2 val)
{
  uByteToBuf(val & 0xff);
  uByteToBuf((val >> 8) & 0xff);
}

AREXPORT void ArLMS1XXPacket::uByte4ToBuf(ArTypes::UByte4 val)
{
  char buf[1024];
  sprintf(buf, "%u", val);
  strToBuf(buf);
}

AREXPORT void ArLMS1XXPacket::strToBuf(const char *str)
{
  if (str == NULL) {
    str = "";
  }

  if (!myFirstAdd && hasWriteCapacity(1)) 
  {
    myBuf[myLength] = ' ';
    myLength++;
  }

  myFirstAdd = false;

  ArTypes::UByte2 tempLen = strlen(str);

  if (!hasWriteCapacity(tempLen)) {
    return;
  }

  memcpy(myBuf+myLength, str, tempLen);
  myLength += tempLen;
}

AREXPORT ArTypes::Byte ArLMS1XXPacket::bufToByte(void)
{
  ArTypes::Byte ret=0;


  if (!isNextGood(1))
    return 0;
  
  if (myBuf[myReadLength] == ' ')
    myReadLength++;

  if (!isNextGood(4))
    return 0;

  unsigned char n1, n2;
  n2 = deascii(myBuf[myReadLength+6]);
  n1 = deascii(myBuf[myReadLength+7]);
  ret = n2 << 4 | n1;

  myReadLength += 4;

  return ret;
}

AREXPORT ArTypes::Byte2 ArLMS1XXPacket::bufToByte2(void)
{
  ArTypes::Byte2 ret=0;

  if (!isNextGood(1))
    return 0;
  
  if (myBuf[myReadLength] == ' ')
    myReadLength++;

  if (!isNextGood(4))
    return 0;

  unsigned char n1, n2, n3, n4;
  n4 = deascii(myBuf[myReadLength+4]);
  n3 = deascii(myBuf[myReadLength+5]);
  n2 = deascii(myBuf[myReadLength+6]);
  n1 = deascii(myBuf[myReadLength+7]);
  ret = n4 << 12 | n3 << 8 | n2 << 4 | n1;

  myReadLength += 4;

  return ret;
}

AREXPORT ArTypes::Byte4 ArLMS1XXPacket::bufToByte4(void)
{
  ArTypes::Byte4 ret=0;

  if (!isNextGood(1))
    return 0;
  
  if (myBuf[myReadLength] == ' ')
    myReadLength++;

  if (!isNextGood(8))
    return 0;

  unsigned char n1, n2, n3, n4, n5, n6, n7, n8;
  n8 = deascii(myBuf[myReadLength]);
  n7 = deascii(myBuf[myReadLength+1]);
  n6 = deascii(myBuf[myReadLength+2]);
  n5 = deascii(myBuf[myReadLength+3]);
  n4 = deascii(myBuf[myReadLength+4]);
  n3 = deascii(myBuf[myReadLength+5]);
  n2 = deascii(myBuf[myReadLength+6]);
  n1 = deascii(myBuf[myReadLength+7]);
  ret = n8 << 28 | n7 << 24 | n6 << 20 | n5 << 16 | n4 << 12 | n3 << 8 | n2 << 4 | n1;

  myReadLength += 8;

  return ret;
}

AREXPORT ArTypes::UByte ArLMS1XXPacket::bufToUByte(void)
{
  ArTypes::UByte ret=0;
  if (!isNextGood(1))
    return 0;
  
  if (myBuf[myReadLength] == ' ')
    myReadLength++;

  std::string str;
  while (isNextGood(1) && myBuf[myReadLength] != ' ' && 
	 myBuf[myReadLength] != '\003')
  {
    str += myBuf[myReadLength];
    myReadLength += 1;
  }
  
  ret = strtol(str.c_str(), NULL, 16);

  return ret;
}

AREXPORT ArTypes::UByte2 ArLMS1XXPacket::bufToUByte2(void)
{
  //printf("@ 1\n");

  ArTypes::UByte2 ret=0;

  if (!isNextGood(1))
    return 0;
  
  if (myBuf[myReadLength] == ' ')
    myReadLength++;

  //printf("@ 2 '%c' %d %d %d %d\n", myBuf[myReadLength], isNextGood(1),
  // myReadLength, myLength, myFooterLength);
  std::string str;
  while (isNextGood(1) && myBuf[myReadLength] != ' ' && 
	 myBuf[myReadLength] != '\003')
  {
    //printf("%c\n", myBuf[myReadLength]);
    str += myBuf[myReadLength];
    myReadLength += 1;
  }


  ret = strtol(str.c_str(), NULL, 16);

  //printf("@ 3 %d\n", ret);
  return ret;
}

AREXPORT ArTypes::UByte4 ArLMS1XXPacket::bufToUByte4(void)
{
  ArTypes::Byte4 ret=0;

  if (!isNextGood(1))
    return 0;
  
  if (myBuf[myReadLength] == ' ')
    myReadLength++;

  std::string str;
  while (isNextGood(1) && myBuf[myReadLength] != ' ' && 
	 myBuf[myReadLength] != '\003')
  {
    str += myBuf[myReadLength];
    myReadLength += 1;
  }


  ret = strtol(str.c_str(), NULL, 16);

  return ret;
}

/** 
Copy a string from the packet buffer into the given buffer, stopping when
the end of the packet buffer is reached, the given length is reached,
or a NUL character ('\\0') is reached.  If the given length is not large
enough, then the remainder of the string is flushed from the packet.
A NUL character ('\\0') is appended to @a buf if there is sufficient room
after copying the sting from the packet, otherwise no NUL is added (i.e.
if @a len bytes are copied).
@param buf Destination buffer
@param len Maximum number of characters to copy into the destination buffer
*/
AREXPORT void ArLMS1XXPacket::bufToStr(char *buf, int len)
{
  if (buf == NULL) 
  {
    ArLog::log(ArLog::Normal, "ArLMS1XXPacket::bufToStr(NULL, %d) cannot write to null address",
               len);
    return;
  }
  int i;

  buf[0] = '\0';

  if (!isNextGood(1))
    return;
  
  if (myBuf[myReadLength] == ' ')
    myReadLength++;

  // see if we can read
  if (isNextGood(1))
  {
    // while we can read copy over those bytes
    for (i = 0; 
         isNextGood(1) && i < (len - 1) && myBuf[myReadLength] != ' ' && 
	 myBuf[myReadLength] != '\003';
         ++myReadLength, ++i) 
    {
      buf[i] = myBuf[myReadLength];
      buf[i+1] = '\0';
      //printf("%d %c %p\n", i);
    }
    if (i >= (len - 1)) 
    { 
      // Otherwise, if we stopped reading because the output buffer was full,
      // then attempt to flush the rest of the string from the packet

      // This is a bit redundant with the code below, but wanted to log the  
      // string for debugging
      myBuf[len - 1] = '\0';

      ArLog::log(ArLog::Normal, "ArLMS1XXPacket::bufToStr(buf, %d) output buf is not large enough for packet string %s",
                 len, myBuf);

      while (isNextGood(1) && myBuf[myReadLength] != ' ' && 
	     myBuf[myReadLength] != '\003') {
        myReadLength++;
      }
    } // end else if output buffer filled before null-terminator
  } // end if something to read

  // Make absolutely sure that the string is null-terminated...
  buf[len - 1] = '\0';
}

AREXPORT void ArLMS1XXPacket::rawCharToBuf(unsigned char c)
{
  if (!hasWriteCapacity(1)) {
    return;
  }
  myBuf[myLength] = c;
  //memcpy(myBuf+myLength, &c, 1);
  myLength += 1;
}

int ArLMS1XXPacket::deascii(char c)
{
  if (c >= '0' && c <= '9')
    return c - '0';
  else if (c >= 'a' && c <= 'f')
    return 10 + c - 'a';
  else if (c >= 'A' && c <= 'F')
    return 10 + c - 'a';
  else
    return 0;
}

AREXPORT ArLMS1XXPacketReceiver::ArLMS1XXPacketReceiver()
{
  myState = STARTING;
}

AREXPORT ArLMS1XXPacketReceiver::~ArLMS1XXPacketReceiver()
{

}

AREXPORT void ArLMS1XXPacketReceiver::setDeviceConnection(ArDeviceConnection *conn)
{
  myConn = conn;
}

AREXPORT ArDeviceConnection *ArLMS1XXPacketReceiver::getDeviceConnection(void)
{
  return myConn;
}
  

ArLMS1XXPacket *ArLMS1XXPacketReceiver::receivePacket(unsigned int msWait,
						      bool scandataShortcut)
{
  ArLMS1XXPacket *packet;
  unsigned char c;
  long timeToRunFor;
  ArTime timeDone;
  ArTime lastDataRead;
  ArTime packetReceived;
  int numRead;
  int i;

  if (myConn == NULL || 
      myConn->getStatus() != ArDeviceConnection::STATUS_OPEN)
  {
    return NULL;
  }

  timeDone.setToNow();
  if (!timeDone.addMSec(msWait)) {
    ArLog::log(ArLog::Normal,
               "ArLMS1XXPacketReceiver::receivePacket() error adding msecs (%i)",
               msWait);
  }

  do
  {
    timeToRunFor = timeDone.mSecTo();
    if (timeToRunFor < 0)
      timeToRunFor = 0;

    //printf("%x\n", c);
    if (myState == STARTING)
    {
      if ((numRead = myConn->read((char *)&c, 1, timeToRunFor)) <= 0) 
	return NULL;
      if (c == '\002')
      {
	myState = DATA;
	myPacket.empty();
	myPacket.setLength(0);
	myPacket.rawCharToBuf(c);
	myReadCount = 0;
	packetReceived = myConn->getTimeRead(0);
	myPacket.setTimeReceived(packetReceived);
      }
      else
      {
	ArLog::log(ArLog::Normal, 
		   "ArLMS1XXPacketReceiver: Failed read (%d)", 
		   numRead);
      }
    }
    else if (myState == DATA)
    {
      numRead = myConn->read(&myReadBuf[myReadCount],
			     sizeof(myReadBuf) - myReadCount,
			     5);
      // trap if we failed the read
      if (numRead < 0)
      {
	ArLog::log(ArLog::Normal, 
		   "ArLMS1XXPacketReceiver: Failed read (%d)", 
		   numRead);
	myState = STARTING;
	return NULL;
      }
      // see if we found the end of the packet 
      for (i = myReadCount; i < myReadCount + numRead; i++)
      {
	if (myReadBuf[i] == '\002')
	{
	  ArLog::log(ArLog::Verbose, "ArLMS1XXPacketReceiver: Data found start of new packet...", 
		     myReadCount);
	  myPacket.empty();
	  myPacket.setLength(0);
	  memmove(myReadBuf, &myReadBuf[i], myReadCount + numRead - i);
	  numRead -= (i - myReadCount);
	  myReadCount -= i;
	  i = 0;
	  continue;
	}
	if (myReadBuf[i] == '\003')
	{
	  myPacket.dataToBuf(myReadBuf, i + 1);
	  myPacket.resetRead();
	  packet = new ArLMS1XXPacket;
	  packet->duplicatePacket(&myPacket);
	  myPacket.empty();
	  myPacket.setLength(0);

	  // if it's the end of the data just go back to the beginning
	  if (i == myReadCount + numRead - 1)
	  {
	    //ArLog::log(ArLog::Verbose, "ArLMS1XXPacketReceiver: Starting again");
	    myState = STARTING;
	  }
	  // if it isn't move the data up and start again	  
	  else
	  {
	    memmove(myReadBuf, &myReadBuf[i+1], myReadCount + numRead - i - 1);
	    myReadCount = myReadCount + numRead - i - 1;
	    myState = REMAINDER;
	    ArLog::log(ArLog::Verbose, "ArLMS1XXPacketReceiver: Got remainder, %d bytes beyond one packet ...", 
		       myReadCount);
	  }	    
	  return packet;
	}
      }
      myReadCount += numRead;
      ArLog::log(ArLog::Verbose, "ArLMS1XXPacketReceiver: Got %d bytes (but not end char), up to %d", 
		 numRead, myReadCount);
    }
    else if (myState == REMAINDER)
    {
      //printf("In remainder ('%c' %d) \n", myReadBuf[0], myReadBuf[0]);
      if (myReadBuf[0] != '\002')
      {
	ArLog::log(ArLog::Verbose, 
		   "ArLMS1XXPacketReceiver: Remainder didn't start with \\002, starting over...");
	myState = STARTING;
	continue;
      }
      // see if we found the end of the packet 
      for (i = 0; i < myReadCount - 1; i++)
      {
	//printf("%03d '%c' %d\n", i, myReadBuf[i], myReadBuf[i]);
	if (myReadBuf[i] == '\002' && i != 0)
	{
	  ArLog::log(ArLog::Verbose, "ArLMS1XXPacketReceiver: Remainder found start of new packet...", 
		     myReadCount);
	  myPacket.empty();
	  myPacket.setLength(0);
	  memmove(myReadBuf, &myReadBuf[i], myReadCount + i);
	  myReadCount -= i;
	  i = 0;
	  continue;
	}
	if (myReadBuf[i] == '\003')
	{
	  myPacket.dataToBuf(myReadBuf, i + 1);
	  myPacket.resetRead();
	  packet = new ArLMS1XXPacket;
	  packet->duplicatePacket(&myPacket);
	  myPacket.empty();
	  myPacket.setLength(0);

	  // if it's the end of the data just go back to the beginning
	  if (i == myReadCount - 1)
	  {
	    myState = STARTING;
	    ArLog::log(ArLog::Verbose, 
		       "ArLMS1XXPacketReceiver: Remainder was one packet...");
	  }
	  // if it isn't move the data up and start again	  
	  else
	  {
	    if (myReadCount - i < 50)
	      printf("read buf (%d %d) %s\n", myReadCount, i, myReadBuf);
	    memmove(myReadBuf, &myReadBuf[i+1], myReadCount - i);
	    myReadCount = myReadCount - i - 1;
	    myState = REMAINDER;
	    ArLog::log(ArLog::Verbose, 
		       "ArLMS1XXPacketReceiver: Remainder was more than one packet... (%d %d)", myReadCount, i);
	  }	    
	  return packet;
	}
      }
      // if we didn't find the end of the packet, then get the rest of the data
      myState = DATA;
      ArLog::log(ArLog::Verbose, 
	 "ArLMS1XXPacketReceiver: Remainder didn't contain a whole packet...");

      continue;
    }
    else
    {
      ArLog::log(ArLog::Normal, "ArLMS1XXPacketReceiver: Bad state (%d)", 
		 myState);
      myState = STARTING;
    }
  } while (timeDone.mSecTo() >= 0); // || !myStarting)

  return NULL;
}

AREXPORT ArLMS1XX::ArLMS1XX(int laserNumber,
			    const char *name) : 
  ArLaser(laserNumber, name, 20000),
  mySensorInterpTask(this, &ArLMS1XX::sensorInterp),
  myAriaExitCB(this, &ArLMS1XX::disconnect)
{
  clear();
  myRawReadings = new std::list<ArSensorReading *>;

  Aria::addExitCallback(&myAriaExitCB, -10);

  setInfoLogLevel(ArLog::Normal);

  laserSetName(getName());

  laserAllowSetPowerControlled(false);
  laserAllowSetDegrees(-135, -135, 135, // start degrees
		  135, -135, 135); // end degrees

  std::map<std::string, double> incrementChoices;
  incrementChoices["half"] = .5;
  incrementChoices["quarter"] = .25;
  laserAllowIncrementChoices("half", incrementChoices);

  laserSetDefaultTcpPort(2111);
  laserSetDefaultPortType("tcp");

  myLogLevel = ArLog::Verbose;

  setMinDistBetweenCurrent(0);
  setMaxDistToKeepCumulative(4000);
  setMinDistBetweenCumulative(200);
  setMaxSecondsToKeepCumulative(30);
  setMaxInsertDistCumulative(3000);

  setCumulativeCleanDist(75);
  setCumulativeCleanInterval(1000);
  setCumulativeCleanOffset(600);

  resetLastCumulativeCleanTime();

  setCurrentDrawingData(
	  new ArDrawingData("polyDots", 
			    ArColor(0, 0, 255), 
			    80,  // mm diameter of dots
			    75), // layer above sonar 
	  true);

  setCumulativeDrawingData(
	  new ArDrawingData("polyDots", 
			    ArColor(125, 125, 125), 
			    100, // mm diameter of dots
			    60), // layer below current range devices  
	  true);

}

AREXPORT ArLMS1XX::~ArLMS1XX()
{
  Aria::remExitCallback(&myAriaExitCB);
  if (myRobot != NULL)
  {
    myRobot->remRangeDevice(this);
    myRobot->remLaser(this);
    myRobot->remSensorInterpTask(&mySensorInterpTask);
  }
  if (myRawReadings != NULL)
  {
    ArUtil::deleteSet(myRawReadings->begin(), myRawReadings->end());
    myRawReadings->clear(); 
    delete myRawReadings;
    myRawReadings = NULL;
  }
  lockDevice();
  if (isConnected())
    disconnect();
  unlockDevice();
}

void ArLMS1XX::clear(void)
{
  myIsConnected = false;
  myTryingToConnect = false;
  myStartConnect = false;

  myVersionNumber = 0;
  myDeviceNumber = 0;
  mySerialNumber = 0;
  myDeviceStatus1 = 0;
  myDeviceStatus2 = 0;
  myMessageCounter = 0;
  myScanCounter = 0;
  myPowerUpDuration = 0;
  myTransmissionDuration = 0;
  myInputStatus1 = 0;
  myInputStatus2 = 0;
  myOutputStatus1 = 0;
  myOutputStatus2 = 0;
  myReserved = 0;
  myScanningFreq = 0;
  myMeasurementFreq = 0;
  myNumberEncoders = 0;
  myNumChans = 0;
}

AREXPORT void ArLMS1XX::laserSetName(const char *name)
{
  myName = name;
  
  myConnMutex.setLogNameVar("%s::myConnMutex", getName());
  myPacketsMutex.setLogNameVar("%s::myPacketsMutex", getName());
  myDataMutex.setLogNameVar("%s::myDataMutex", getName());
  myAriaExitCB.setNameVar("%s::exitCallback", getName());
  
  ArLaser::laserSetName(getName());
}

AREXPORT void ArLMS1XX::setRobot(ArRobot *robot)
{
  myRobot = robot;

  if (myRobot != NULL)
  {
    myRobot->remSensorInterpTask(&mySensorInterpTask);
    myRobot->addSensorInterpTask("lms1XX", 90, &mySensorInterpTask);
  }
  ArLaser::setRobot(robot);
}



AREXPORT bool ArLMS1XX::asyncConnect(void)
{
  myStartConnect = true;
  if (!getRunning())
    runAsync();
  return true;
}

AREXPORT bool ArLMS1XX::disconnect(void)
{
  if (!isConnected())
    return true;

  ArLog::log(ArLog::Normal, "%s: Disconnecting", getName());

  laserDisconnectNormally();
  return true;
}

void ArLMS1XX::failedToConnect(void)
{
  lockDevice();
  myTryingToConnect = true;
  unlockDevice();
  laserFailedConnect();
}

void ArLMS1XX::sensorInterp(void)
{
  ArLMS1XXPacket *packet;
  
  while (1)
  {
    myPacketsMutex.lock();
    if (myPackets.empty())
    {
      myPacketsMutex.unlock();
      return;
    }
    packet = myPackets.front();
    myPackets.pop_front();
    myPacketsMutex.unlock();
	   
    // if its not a reading packet just skip it 
    if (strcasecmp(packet->getCommandName(), "LMDscandata") != 0)
    {
      delete packet;
      continue;
    }

    //set up the times and poses

    ArTime time = packet->getTimeReceived();
    ArPose pose;
    int ret;
    int retEncoder;
    ArPose encoderPose;
    
    // this value should be found more empirically... but we used 1/75
    // hz for the lms2xx and it was fine, so here we'll use 1/50 hz for now
    if (!time.addMSec(-20)) {
      ArLog::log(ArLog::Normal,
                 "ArLMS1XX::sensorInterp() error adding msecs (-20)");
    }

    if (myRobot == NULL || !myRobot->isConnected())
    {
      pose.setPose(0, 0, 0);
      encoderPose.setPose(0, 0, 0);
    } 
    else if ((ret = myRobot->getPoseInterpPosition(time, &pose)) < 0 ||
	     (retEncoder = 
	      myRobot->getEncoderPoseInterpPosition(time, &encoderPose)) < 0)
    {
      ArLog::log(ArLog::Normal, "%s: reading too old to process", getName());
      delete packet;
      continue;
    }
    
    ArTransform transform;
    transform.setTransform(pose);
    
    unsigned int counter = 0; 
    if (myRobot != NULL)
      counter = myRobot->getCounter();
    
    lockDevice();
    myDataMutex.lock();
    
    int i;
    int dist;
    //int onStep;
    
    std::list<ArSensorReading *>::reverse_iterator it;
    ArSensorReading *reading;

    // read the extra stuff
    myVersionNumber = packet->bufToUByte2();
    myDeviceNumber = packet->bufToUByte2();
    mySerialNumber = packet->bufToUByte4();
    myDeviceStatus1 = packet->bufToUByte();
    myDeviceStatus2 = packet->bufToUByte();
    myMessageCounter = packet->bufToUByte2();
    myScanCounter = packet->bufToUByte2();
    myPowerUpDuration = packet->bufToUByte4();
    myTransmissionDuration = packet->bufToUByte4();
    myInputStatus1 = packet->bufToUByte();
    myInputStatus2 = packet->bufToUByte();
    myOutputStatus1 = packet->bufToUByte();
    
    myOutputStatus2 = packet->bufToUByte();
    myReserved = packet->bufToUByte2();
    myScanningFreq = packet->bufToUByte4();
    myMeasurementFreq = packet->bufToUByte4();

    if (myDeviceStatus1 != 0 || myDeviceStatus2 != 0)
      ArLog::log(myLogLevel, "%s: DeviceStatus %d %d", 
		 myDeviceStatus1, myDeviceStatus2); 

    /*
      printf("Received: %s %s ver %d devNum %d serNum %d scan %d sf %d mf %d\n", 
	   packet->getCommandType(), packet->getCommandName(), 
	   myVersionNumber, myDeviceNumber, 
	   mySerialNumber, myScanCounter, myScanningFreq, myMeasurementFreq);
    */
    myNumberEncoders = packet->bufToUByte2();
    //printf("\tencoders %d\n", myNumberEncoders);
    if (myNumberEncoders > 0)
      ArLog::log(myLogLevel, "%s: Encoders %d", getName(), myNumberEncoders);

    for (i = 0; i < myNumberEncoders; i++)
    {
      packet->bufToUByte4();
      packet->bufToUByte2();
      //printf("\t\t%d\t%d %d\n", i, eachEncoderPosition, eachEncoderSpeed);
    }

    myNumChans = packet->bufToUByte2();
    if (myNumChans > 1)
      ArLog::log(myLogLevel, "%s: NumChans %d", getName(), myNumChans);
    //printf("\tnumchans %d\n", myNumChans);

    char eachChanMeasured[1024];
    int eachScalingFactor;
    int eachScalingOffset;
    double eachStartingAngle;
    double eachAngularStepWidth;
    int eachNumberData;


    for (i = 0; i < myNumChans; i++)
    {
      eachChanMeasured[0] = '\0';
      packet->bufToStr(eachChanMeasured, sizeof(eachChanMeasured));
      
      // if this isn't the data we want then skip it
      if (strcasecmp(eachChanMeasured, "DIST1") != 0 &&
	  strcasecmp(eachChanMeasured, "DIST2") != 0)
	continue;

      eachScalingFactor = packet->bufToUByte4(); // FIX should be real
      eachScalingOffset = packet->bufToUByte4(); // FIX should be real
      eachStartingAngle = packet->bufToByte4() / 10000.0;
      eachAngularStepWidth = packet->bufToUByte2() / 10000.0;
      eachNumberData = packet->bufToUByte2();

      /*
      ArLog::log(myLogLevel, "%s: %s start %.1f step %.2f numReadings %d", 
		 getName(), eachChanMeasured,
		 eachStartingAngle, eachAngularStepWidth, eachNumberData);
      */

      /*
      printf("\t\t%s\tscl %d %d ang %g %g num %d\n", 
	     eachChanMeasured, 
	     eachScalingFactor, eachScalingOffset, 
	     eachStartingAngle, eachAngularStepWidth, 
	     eachNumberData);
      */
      // If we don't have any sensor readings created at all, make 'em all 
      if (myRawReadings->size() == 0)
	for (i = 0; i < eachNumberData; i++)
	  myRawReadings->push_back(new ArSensorReading);

      if (eachNumberData > myRawReadings->size())
      {
	ArLog::log(ArLog::Terse, "%s: Bad data, in theory have %d readings but can only have 541... skipping this packet\n", getName(), eachNumberData);
	printf("%s\n", packet->getBuf());
	continue;
      }
      
      std::list<ArSensorReading *>::iterator it;
      double atDeg;
      int onReading;

      double start;
      double increment;
      
      if (myFlipped)
      {
	start = mySensorPose.getTh() + eachStartingAngle - 90.0 + eachAngularStepWidth * eachNumberData;
	increment = -eachAngularStepWidth;
      }
      else
      {
	start = mySensorPose.getTh() + eachStartingAngle - 90.0;
	increment = eachAngularStepWidth;
      }
	
      bool ignore;
      for (//atDeg = mySensorPose.getTh() + eachStartingAngle - 90.0,
	   //atDeg = mySensorPose.getTh() + eachStartingAngle - 90.0 + eachAngularStepWidth * eachNumberData,
	   atDeg = start,
	   it = myRawReadings->begin(),
	   onReading = 0; 
	   
	   onReading < eachNumberData; 
	   
	   //atDeg += eachAngularStepWidth,
	   //atDeg -= eachAngularStepWidth,
	   atDeg += increment,
	   it++,
	   onReading++)
      {
	ignore = false;

	if (atDeg < getStartDegrees() || atDeg > getEndDegrees())
	  ignore = true;

	reading = (*it);
	dist = packet->bufToUByte2();

	// this was the original code, that just ignored 0s as a
	// reading... however sometimes the sensor reports very close
	// distances for rays it gets no return on... Sick wasn't very
	// helpful figuring out what values it will report for
	// those... so this is changing to a check that is basically
	// if it reports as well within the sensor itself it gets
	// ignored (the sensor head is 90mm across, but part of that
	// slopes in, so this check should be those readings well
	// within the sensor).  Further note that shiny/black things
	// within the minimum range will sometimes report closer than
	// they are... 9/21/2010 MPL

	//if (dist == 0)
	if (dist < 30)
	{
	  ignore = true;
	}
	
	/*
	else if (!ignore && dist < 150)
	{
	  //ignore = true;

	  ArLog::log(ArLog::Normal, "%s: Reading at %.1f %s is %d (not ignoring, just warning)", 
		     getName(), atDeg, 
		     eachChanMeasured, dist);
	}
	*/

	reading->resetSensorPosition(ArMath::roundInt(mySensorPose.getX()),
				     ArMath::roundInt(mySensorPose.getY()),
				     atDeg); 
	reading->newData(dist, pose, encoderPose, transform, counter, 
			 time, ignore, 0); // no reflector yet
      }
      /*
      ArLog::log(ArLog::Normal, 
		 "Received: %s %s scan %d numReadings %d", 
		 packet->getCommandType(), packet->getCommandName(), 
		 myScanCounter, onReading);
      */
    }
    
    myDataMutex.unlock(); 

    
    laserProcessReadings();
    unlockDevice();
    delete packet;
  }
}

AREXPORT ArLMS1XXPacket *ArLMS1XX::sendAndRecv(
	ArTime timeout, ArLMS1XXPacket *sendPacket, const char *recvName)
{
  ArLMS1XXPacket *packet;

  while (timeout.mSecTo() > 0)
  {
    // just ask for data
    if (!myConn->write(sendPacket->getBuf(), sendPacket->getLength()))
    {
      ArLog::log(ArLog::Terse, 
		 "%s: Could not send %s to laser", getName(), recvName);
      return NULL;
    }

    while (timeout.mSecTo() > 0 && 
	   (packet = myReceiver.receivePacket(1000)) != NULL)
    {
      if (strcasecmp(packet->getCommandName(), recvName) == 0)
	return packet;
      else
      {
	ArLog::log(myLogLevel, "%s: Got %s %s", getName(),
		   packet->getCommandType(), 
		   packet->getCommandName());

	delete packet;
	packet = NULL;
      }

    }
  }

  ArLog::log(ArLog::Normal, 
	     "%s::blockingConnect: Did not get %s ack", 
	     getName(), recvName);
  
  return NULL; 
}

AREXPORT bool ArLMS1XX::blockingConnect(void)
{
  char buf[1024];

  if (!getRunning())
    runAsync();

  myConnMutex.lock();
  if (myConn == NULL)
  {
    ArLog::log(ArLog::Terse, 
	       "%s: Could not connect because there is no connection defined", 
	       getName());
    myConnMutex.unlock();
    failedToConnect();
    return false;
  }

  if (myConn->getStatus() != ArDeviceConnection::STATUS_OPEN && 
      !myConn->openSimple())
  {
    ArLog::log(ArLog::Terse, 
	       "%s: Could not connect because the connection was not open and could not open it", getName());
    myConnMutex.unlock();
    failedToConnect();
    return false;
  }
  myReceiver.setDeviceConnection(myConn);
  myConnMutex.unlock();

  lockDevice();
  myTryingToConnect = true;
  unlockDevice();

  laserPullUnsetParamsFromRobot();
  laserCheckParams();
  
  int size = (270 / .25 + 1);
  ArLog::log(myInfoLogLevel, "%s: Setting current buffer size to %d", 
	     getName(), size);
  setCurrentBufferSize(size);
  

  ArTime timeDone;
  if (myPowerControlled) {
    if (!timeDone.addMSec(60 * 1000)) {
      ArLog::log(ArLog::Normal,
                 "ArLMS1XX::blockingConnect() error adding msecs (60 * 1000)");
    }
  }  
  else {
    if (!timeDone.addMSec(30 * 1000)) {
      ArLog::log(ArLog::Normal,
                 "ArLMS1XX::blockingConnect() error adding msecs (30 * 1000)");
    }
  }
  ArLMS1XXPacket *packet;

  ArLMS1XXPacket sendPacket;

  sendPacket.empty();
  sendPacket.strToBuf("sMN");
  sendPacket.strToBuf("SetAccessMode");
  sendPacket.uByteToBuf(0x3); // level
  sendPacket.strToBuf("F4724744"); // hashed password
  sendPacket.finalizePacket();

  if ((packet = sendAndRecv(timeDone, &sendPacket, "SetAccessMode")) != NULL)
  {
    int val;
    val = packet->bufToUByte();

    delete packet;
    packet = NULL;

    if (val == 1)
    {
      ArLog::log(myLogLevel, "%s: Changed access mode (%d)", 
		 getName(), val);
    }
    else
    {
      ArLog::log(ArLog::Terse, 
		 "%s: Could not change access mode (%d)", getName(), val);
      failedToConnect();
      return false;
    }
  }
  else
  {    
    failedToConnect();
    return false;
  }

  sendPacket.empty();
  sendPacket.strToBuf("sMN");
  sendPacket.strToBuf("mLMPsetscancfg");
  sendPacket.byte4ToBuf(5000); // scanning freq
  sendPacket.byte2ToBuf(1); // number segments
  sendPacket.byte4ToBuf(getIncrementChoiceDouble() * 10000); // angle resolution
  //sendPacket.byte4ToBuf((getStartDegrees() + 90) * 10000); // starting angle
  sendPacket.byte4ToBuf(-45 * 10000); // can't change starting angle
  //sendPacket.byte4ToBuf((getEndDegrees() + 90) * 10000); // ending angle
  sendPacket.byte4ToBuf(225 * 10000); // can't change ending angle
  sendPacket.finalizePacket();

  ArLog::log(myLogLevel, "%s: setscancfg: %s", getName(), 
	     sendPacket.getBuf());

  if ((packet = sendAndRecv(timeDone, &sendPacket, "mLMPsetscancfg")) != NULL)
  {
    int val;
    val = packet->bufToUByte();

    delete packet;
    packet = NULL;

    if (val == 0)
    {
      ArLog::log(myLogLevel, "%s: setscancfg succeeded (%d)", 
		 getName(), val);
    }
    else
    {
      ArLog::log(ArLog::Terse, 
		 "%s: Setscancfg failed (%d)", getName(), val);
      failedToConnect();
      return false;
    }

  }
  else
  {
    failedToConnect();
    return false;
  }

  sendPacket.empty();
  sendPacket.strToBuf("sWN");
  sendPacket.strToBuf("LMDscandatacfg");
  sendPacket.uByte2ToBuf(0x1); // output channel
  sendPacket.uByteToBuf(0x0); // remission
  sendPacket.uByteToBuf(0x0); // remission resolution
  sendPacket.uByteToBuf(0x0); // unit
  sendPacket.uByte2ToBuf(0x0); // encoder
  sendPacket.uByteToBuf(0x0); // position
  sendPacket.uByteToBuf(0x0); // device name
  sendPacket.uByteToBuf(0x0); // comment
  sendPacket.uByteToBuf(0x0); // time 
  sendPacket.byteToBuf(5); // which scan
  //sendPacket.byteToBuf(1); // which scan
  sendPacket.finalizePacket();

  ArLog::log(myLogLevel, "%s: scandatacfg: %s", getName(), sendPacket.getBuf());

  if ((packet = sendAndRecv(timeDone, &sendPacket, "LMDscandatacfg")) != NULL)
  {
    ArLog::log(myLogLevel, "%s: scandatacfg succeeded", getName());

    delete packet;
    packet = NULL;
  }
  else
  {
    failedToConnect();
    return false;
  }

  sendPacket.empty();
  sendPacket.strToBuf("sMN");
  sendPacket.strToBuf("Run");
  sendPacket.finalizePacket();

  if ((packet = sendAndRecv(timeDone, &sendPacket, "Run")) != NULL)
  {
    int val;
    val = packet->bufToUByte();
    delete packet;
    packet = NULL;
    if (val == 1)
    {
      ArLog::log(myLogLevel, "%s: Run succeeded (%d)", 
		 getName(), val);
    }
    else
    {
      ArLog::log(ArLog::Terse, 
		 "%s: Could not run (%d)", getName(), val);
      failedToConnect();
      return false;
    }
  }
  else
  {    
    failedToConnect();
    return false;
  }

  /* when asking one at a time
  sendPacket.empty();
  sendPacket.strToBuf("sRN");
  sendPacket.strToBuf("LMDscandata");
  sendPacket.finalizePacket();

  if ((packet = sendAndRecv(timeDone, &sendPacket, "LMDscandata")) != NULL)
  {
    ArLog::log(myLogLevel, "%s: Got %s scan data %d", getName(), 
	       packet->getCommandType(), packet->getLength());
    myPacketsMutex.lock();
    myPackets.push_back(packet);
    myPacketsMutex.unlock();	
    sensorInterp();

    ArLog::log(myLogLevel, "%s: Processed scan data", getName());

  }
  else
  {
    failedToConnect();
    return false;
  }
  */

  sendPacket.empty();
  sendPacket.strToBuf("sEN");
  sendPacket.strToBuf("LMDscandata");
  sendPacket.uByteToBuf(1);
  sendPacket.finalizePacket();

  //printf("(%s)\n", sendPacket.getBuf());
  // just ask for continuous data
  if (!myConn->write(sendPacket.getBuf(), sendPacket.getLength()))
  {
    ArLog::log(ArLog::Terse, 
	       "%s: Could not send %s to laser", getName(), "LMDscandata");
    failedToConnect();
    return false;
  }

  while (timeDone.mSecTo())
  {
    packet = myReceiver.receivePacket(1000);
    if (packet != NULL && 
	strcasecmp(packet->getCommandType(), "sSN") == 0 && 
	strcasecmp(packet->getCommandName(), "LMDscandata") == 0)
    {
      delete packet;
      packet = NULL;

      lockDevice();
      myIsConnected = true;
      myTryingToConnect = false;
      unlockDevice();
      ArLog::log(ArLog::Normal, "%s: Connected to laser", getName());
      laserConnect();
      return true;
    }
    else if (packet != NULL)
    {
      ArLog::log(myLogLevel, "%s: Got %s %s (%d long)", getName(), 
		 packet->getCommandType(), packet->getCommandName(), 
		 packet->getLength());
      delete packet;
      packet = NULL;
    }
  }

  ArLog::log(ArLog::Terse, 
	     "%s: Did not get scandata back from laser", getName());
  failedToConnect();
  return false;

}

AREXPORT void * ArLMS1XX::runThread(void *arg)
{
  char buf[1024];
  ArLMS1XXPacket *packet;

  /*
  ArTime dataRequested;

  ArLMS1XXPacket requestPacket;
  requestPacket.strToBuf("sRN");
  requestPacket.strToBuf("LMDscandata");
  requestPacket.finalizePacket();
  */

  while (getRunning())
  {
    lockDevice();
    if (myStartConnect)
    {
      myStartConnect = false;
      myTryingToConnect = true;
      unlockDevice();

      blockingConnect();

      lockDevice();
      myTryingToConnect = false;
      unlockDevice();
      continue;
    }
    unlockDevice();
    
    if (!myIsConnected)
    {
      ArUtil::sleep(100);
      continue;
    }


    /*
    dataRequested.setToNow();

    if (myConn == NULL || !myConn->write(requestPacket.getBuf(), 
					 requestPacket.getLength()))
    {
      ArLog::log(ArLog::Terse, "Could not send packets request to lms1XX");
      continue;
    }
    */

    while (getRunning() && myIsConnected && 
	   (packet = myReceiver.receivePacket(50, true)) != NULL)
    {
      myPacketsMutex.lock();
      myPackets.push_back(packet);
      myPacketsMutex.unlock();	

      if (myRobot == NULL)
	sensorInterp();

      // if we have a robot but it isn't running yet then don't have a
      // connection failure
      if (laserCheckLostConnection())
      {
	ArLog::log(ArLog::Terse, 
		   "%s:  Lost connection to the laser because of error.  Nothing received for %g seconds (greater than the timeout of %g).", getName(), 
		   myLastReading.mSecSince()/1000.0, 
		   getConnectionTimeoutSeconds());
	myIsConnected = false;
	laserDisconnectOnError();
	continue;
      }
    }

    ArUtil::sleep(1);
    //ArUtil::sleep(2000);
    //ArUtil::sleep(500);
  }
  return NULL;
}

