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
#ifndef ARLMS1XX_H
#define ARLMS1XX_H

#include "ariaTypedefs.h"
#include "ArRobotPacket.h"
#include "ArLaser.h"   
#include "ArFunctor.h"

/** @internal */
class ArLMS1XXPacket : public ArBasePacket
{
public:
  /// Constructor
  AREXPORT ArLMS1XXPacket();
  /// Destructor
  AREXPORT virtual ~ArLMS1XXPacket();

  /// Gets the command type 
  AREXPORT const char *getCommandType(void);
  /// Gets the command name
  AREXPORT const char *getCommandName(void);
  
  // only call finalizePacket before a send
  AREXPORT virtual void finalizePacket(void);
  AREXPORT virtual void resetRead(void);
  
  /// Gets the time the packet was received at
  AREXPORT ArTime getTimeReceived(void);
  /// Sets the time the packet was received at
  AREXPORT void setTimeReceived(ArTime timeReceived);

  AREXPORT virtual void duplicatePacket(ArLMS1XXPacket *packet);
  AREXPORT virtual void empty(void);
  
  AREXPORT virtual void byteToBuf(ArTypes::Byte val);
  AREXPORT virtual void byte2ToBuf(ArTypes::Byte2 val);
  AREXPORT virtual void byte4ToBuf(ArTypes::Byte4 val);
  AREXPORT virtual void uByteToBuf(ArTypes::UByte val);
  AREXPORT virtual void uByte2ToBuf(ArTypes::UByte2 val);
  AREXPORT virtual void uByte4ToBuf(ArTypes::UByte4 val);
  AREXPORT virtual void strToBuf(const char *str);

  AREXPORT virtual ArTypes::Byte bufToByte(void);
  AREXPORT virtual ArTypes::Byte2 bufToByte2(void);
  AREXPORT virtual ArTypes::Byte4 bufToByte4(void);
  AREXPORT virtual ArTypes::UByte bufToUByte(void);
  AREXPORT virtual ArTypes::UByte2 bufToUByte2(void);
  AREXPORT virtual ArTypes::UByte4 bufToUByte4(void);
  AREXPORT virtual void bufToStr(char *buf, int len);

  // adds a raw char to the buf
  AREXPORT virtual void rawCharToBuf(unsigned char c);
protected:
  int deascii(char c);

  ArTime myTimeReceived;
  bool myFirstAdd;

  char myCommandType[1024]; 
  char myCommandName[1024]; 
};


/// Given a device connection it receives packets from the sick through it
/// @internal
class ArLMS1XXPacketReceiver
{
public:
  /// Constructor with assignment of a device connection
  AREXPORT ArLMS1XXPacketReceiver();
  /// Destructor
  AREXPORT virtual ~ArLMS1XXPacketReceiver();
  
  /// Receives a packet from the robot if there is one available
  AREXPORT ArLMS1XXPacket *receivePacket(unsigned int msWait = 0,
					 bool shortcut = false);

  /// Sets the device this instance receives packets from
  AREXPORT void setDeviceConnection(ArDeviceConnection *conn);
  /// Gets the device this instance receives packets from
  AREXPORT ArDeviceConnection *getDeviceConnection(void);
  
protected:
  ArDeviceConnection *myConn;
  ArLMS1XXPacket myPacket;
  
  enum State 
  {
    STARTING, ///< Looking for beginning
    DATA, ///< Read the data in a big whack
    REMAINDER ///< Have extra data from reading in data
  };
  State myState;
  char myName[1024];
  unsigned int myNameLength;
  char myReadBuf[100000];
  int myReadCount;
};

/**
  @since Aria 2.7.2
  @see ArLaserConnector
  Use ArLaserConnector to connect to a laser, determining type based on robot and program configuration  parameters.
*/
class ArLMS1XX : public ArLaser
{
public:
  /// Constructor
  AREXPORT ArLMS1XX(int laserNumber,
		 const char *name = "lms1XX");
  /// Destructor
  AREXPORT ~ArLMS1XX();
  AREXPORT virtual bool blockingConnect(void);
  AREXPORT virtual bool asyncConnect(void);
  AREXPORT virtual bool disconnect(void);
  AREXPORT virtual bool isConnected(void) { return myIsConnected; }
  AREXPORT virtual bool isTryingToConnect(void) 
    { 
      if (myStartConnect)
	return true;
      else if (myTryingToConnect)
	return true; 
      else
	return false;
    }  

  /// Logs the information about the sensor
  AREXPORT void log(void);
protected:
  AREXPORT virtual void laserSetName(const char *name);
  AREXPORT virtual void * runThread(void *arg);
  AREXPORT virtual void setRobot(ArRobot *robot);
  AREXPORT ArLMS1XXPacket *sendAndRecv(
	  ArTime timeout, ArLMS1XXPacket *sendPacket, const char *recvName);
  void sensorInterp(void);
  void failedToConnect(void);
  void clear(void);
  bool myIsConnected;
  bool myTryingToConnect;
  bool myStartConnect;

  int myVersionNumber;
  int myDeviceNumber;
  int mySerialNumber;
  int myDeviceStatus1;
  int myDeviceStatus2;
  int myMessageCounter;
  int myScanCounter;
  int myPowerUpDuration;
  int myTransmissionDuration;
  int myInputStatus1;
  int myInputStatus2;
  int myOutputStatus1;
  int myOutputStatus2;
  int myReserved;
  int myScanningFreq;
  int myMeasurementFreq;
  int myNumberEncoders;
  int myNumChans;

  ArLog::LogLevel myLogLevel;

  ArLMS1XXPacketReceiver myReceiver;

  ArMutex myPacketsMutex;
  ArMutex myDataMutex;

  std::list<ArLMS1XXPacket *> myPackets;

  ArFunctorC<ArLMS1XX> mySensorInterpTask;
  ArRetFunctorC<bool, ArLMS1XX> myAriaExitCB;
};

#endif 
