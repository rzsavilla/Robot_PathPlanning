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
#include "ArRobotConnector.h"
#include "ArRobot.h"
#include "ArSick.h"
#include "ariaInternal.h"


/** @warning do not delete @a parser during the lifetime of this
 * ArRobotConnector, which may need to access its contents later.
 */
AREXPORT ArRobotConnector::ArRobotConnector(
	ArArgumentParser *parser, ArRobot *robot, bool autoParseArgs) :
  myParseArgsCB(this, &ArRobotConnector::parseArgs),
  myLogOptionsCB(this, &ArRobotConnector::logOptions)
{
  myParser = parser;
  myOwnParser = false;
  myRobot = robot;
  myAutoParseArgs = autoParseArgs;
  myHaveParsedArgs = false;

  myParseArgsCB.setName("ArRobotConnector");
  Aria::addParseArgsCB(&myParseArgsCB, 75);
  myLogOptionsCB.setName("ArRobotConnector");
  Aria::addLogOptionsCB(&myLogOptionsCB, 90);

  myRemoteHost = NULL;
  myRobotPort = NULL;
  myRemoteRobotTcpPort = 8101;
  myRobotBaud = 9600;
  myRemoteIsSim = false;
  myRemoteIsNotSim = false;
  myRobotLogPacketsReceived = false;
  myRobotLogPacketsSent = false;
  myRobotLogMovementReceived = false;
  myRobotLogMovementSent = false;
  myRobotLogVelocitiesReceived = false;
  myRobotLogActions = false;

}

AREXPORT ArRobotConnector::~ArRobotConnector(void)
{

}

/**
 * Parse command line arguments using the ArArgumentParser given in the ArRobotConnector constructor.
 *
 * See parseArgs(ArArgumentParser*) for details about argument parsing.
 * 
  @return true if the arguments were parsed successfully false if not
 **/

AREXPORT bool ArRobotConnector::parseArgs(void)
{
  return parseArgs(myParser);
}

/**
 * Parse command line arguments held by the given ArArgumentParser.
 *
  @return true if the arguments were parsed successfully false if not

   The following arguments are used for the robot connection:

   <dl>
    <dt><code>-robotPort</code> <i>port</i></dt>
    <dt><code>-rp</code> <i>port</i></dt>
    <dd>Use the given serial port device name for a serial port connection (e.g. <code>COM1</code>, or <code>/dev/ttyS0</code> if on Linux.)
    The default is the first serial port, or COM1, which is the typical Pioneer setup.
    </dd>

    <dt><code>-remoteHost</code> <i>hostname</i></dt>
    <dt><code>-rh</code> <i>hostname</i></dt>
    <dd>Use a TCP connection to a remote computer with the given network host name instead of a serial port connection</dd>

    <dt><code>-remoteRobotTcpPort</code> <i>port</i></dt>
    <dt><code>-rrtp</code> <i>port</i></dt>
    <dd>Use the given TCP port number if connecting to a remote robot using TCP due to <code>-remoteHost</code> having been given.</dd>

    <dt><code>-remoteIsSim</code></dt>
    <dt><code>-ris</code></dt>
    <dd>The remote TCP robot given by <code>-remoteHost</code> or <code>-rh</code> is actually a simulator. Use any alternative
     behavior intended for the simulator (e.g. tell the laser device object to request laser data from the simulator rather
     than trying to connect to a real laser device on the local computer)</dd>

    <dt><code>-remoteIsNotSim</code></dt>
    <dt><code>-rins</code></dt>
    <dd>The remote TCP robot given by <code>-remoteHost</code> or
     <code>-rh</code> (or their defaults) is actually a simulator but
     DO NOT treat it like one. Use any the default robot behavior, instead of the simulator behavior (ie open a real laser device for the laser type given and try and talk to it over it's actual local port or remote port if given)</dd>

    <dt><code>-robotBaud</code> <i>baudrate</i></dt>
    <dt><code>-rb</code> <i>baudrate</i></dt>
    <dd>Use the given baud rate when connecting over a serial port, instead of trying to use the normal rate.</dd>
  </dl>

 **/

AREXPORT bool ArRobotConnector::parseArgs(ArArgumentParser *parser)
{
  myHaveParsedArgs = true;

  bool wasReallySetOnlyTrue = parser->getWasReallySetOnlyTrue();
  parser->setWasReallySetOnlyTrue(true);

  if (parser->checkArgument("-remoteIsSim") ||
      parser->checkArgument("-ris"))      
    myRemoteIsSim = true;

  if (parser->checkArgument("-remoteIsNotSim") ||
      parser->checkArgument("-rins"))      
    myRemoteIsNotSim = true;

  if (parser->checkArgument("-robotLogPacketsReceived") || 
      parser->checkArgument("-rlpr")) 

    myRobotLogPacketsReceived = true;
  
  if (parser->checkArgument("-robotLogPacketsSent") || 
      parser->checkArgument("-rlps"))      
    myRobotLogPacketsSent = true;

  if (parser->checkArgument("-robotLogMovementReceived") || 
      parser->checkArgument("-rlmr"))      
    myRobotLogMovementReceived = true;

  if (parser->checkArgument("-robotLogMovementSent") || 
      parser->checkArgument("-rlms"))

    myRobotLogMovementSent = true;
  
  if (parser->checkArgument("-robotLogVelocitiesReceived") || 
      parser->checkArgument("-rlvr"))
      
    myRobotLogVelocitiesReceived = true;
  
  if (parser->checkArgument("-robotLogActions") || 
      parser->checkArgument("-rla"))
    myRobotLogActions = true;

  if (!parser->checkParameterArgumentString("-remoteHost", 
					    &myRemoteHost) ||
      !parser->checkParameterArgumentString("-rh", 
					    &myRemoteHost) ||

      !parser->checkParameterArgumentString("-robotPort",
					     &myRobotPort) ||
      !parser->checkParameterArgumentString("-rp",
					     &myRobotPort) ||

      !parser->checkParameterArgumentInteger("-remoteRobotTcpPort",
					      &myRemoteRobotTcpPort) ||
      !parser->checkParameterArgumentInteger("-rrtp",
					      &myRemoteRobotTcpPort) ||

      !parser->checkParameterArgumentInteger("-robotBaud",
					     &myRobotBaud) || 
      !parser->checkParameterArgumentInteger("-rb",
					     &myRobotBaud))      
  {
    parser->setWasReallySetOnlyTrue(wasReallySetOnlyTrue);
    return false;
  }

  parser->setWasReallySetOnlyTrue(wasReallySetOnlyTrue);
  return true;
}

AREXPORT void ArRobotConnector::logOptions(void) const
{
  ArLog::log(ArLog::Terse, "Options for ArRobotConnector (see docs for more details):");
  ArLog::log(ArLog::Terse, "");

  ArLog::log(ArLog::Terse, "Robot options:");
  ArLog::log(ArLog::Terse, "-remoteHost <remoteHostNameOrIP>");
  ArLog::log(ArLog::Terse, "-rh <remoteHostNameOrIP>");
  ArLog::log(ArLog::Terse, "-robotPort <robotSerialPort>");
  ArLog::log(ArLog::Terse, "-rp <robotSerialPort>");
  ArLog::log(ArLog::Terse, "-robotBaud <baud>");
  ArLog::log(ArLog::Terse, "-rb <baud>");
  ArLog::log(ArLog::Terse, "-remoteRobotTcpPort <remoteRobotTcpPort>");
  ArLog::log(ArLog::Terse, "-rrtp <remoteRobotTcpPort>");
  ArLog::log(ArLog::Terse, "-remoteIsSim");
  ArLog::log(ArLog::Terse, "-ris");
  ArLog::log(ArLog::Terse, "-remoteIsNotSim");
  ArLog::log(ArLog::Terse, "-rins");
  ArLog::log(ArLog::Terse, "-robotLogPacketsReceived");
  ArLog::log(ArLog::Terse, "-rlpr");
  ArLog::log(ArLog::Terse, "-robotLogPacketsSent");
  ArLog::log(ArLog::Terse, "-rlps");
  ArLog::log(ArLog::Terse, "-robotLogMovementReceived");
  ArLog::log(ArLog::Terse, "-rlmr");
  ArLog::log(ArLog::Terse, "-robotLogMovementSent");
  ArLog::log(ArLog::Terse, "-rlms");
  ArLog::log(ArLog::Terse, "-robotLogVelocitiesReceived");
  ArLog::log(ArLog::Terse, "-rlvr");
  ArLog::log(ArLog::Terse, "-robotLogActions");
  ArLog::log(ArLog::Terse, "-rla");
}

/**
 * This method is normally used internally by connectRobot(), but you may 
 * use it if you wish.
 *
 * If -remoteHost was given, then open that TCP port. If it was not given,
 * then try to open a TCP port to the simulator on localhost.
 * If that fails, then use a local serial port connection.
 * Sets the given ArRobot's device connection pointer to this object.
 * Sets up internal settings determined by command line arguments such
 * as serial port and baud rate, etc.
 *
 * After calling this function  (and it returns true), then you may connect
 * ArRobot to the robot using ArRobot::blockingConnect() (or similar).
 *
 * @return false if -remoteHost was given and there was an error connecting to
 * the remote host, true otherwise.
 **/
AREXPORT bool ArRobotConnector::setupRobot(void)
{
  return setupRobot(myRobot);
}

/**
 * @deprecated
 * 
 * This method is normally used internally by connectRobot(), but you may 
 * use it if you wish.
 *
 * If -remoteHost was given, then open that TCP port. If it was not given,
 * then try to open a TCP port to the simulator on localhost.
 * If that fails, then use a local serial port connection.
 * Sets the given ArRobot's device connection pointer to this object.
 * Sets up internal settings determined by command line arguments such
 * as serial port and baud rate, etc.
 *
 * After calling this function  (and it returns true), then you may connect
 * ArRobot to the robot using ArRobot::blockingConnect() (or similar).
 *
 * @return false if -remoteHost was given and there was an error connecting to
 * the remote host, true otherwise.
 **/
AREXPORT bool ArRobotConnector::setupRobot(ArRobot *robot)
{
  if (myRobot == NULL)
    myRobot = robot;

  if (myRobot == NULL)
  {
    ArLog::log(ArLog::Normal, 
	       "ArRobotConnector::setupRobot: NULL robot, cannot setup robot");
    return false;
  }

  if (myAutoParseArgs)
  {
    parseArgs();
  }
  else if (!myHaveParsedArgs)
  {
    ArLog::log(ArLog::Normal, "ArRobotConnector: Args not parsed and are not autoparsed, so connection may fail and command line arguments won't be used");
  }

  // set up all that logging
  if (myRobotLogPacketsReceived)
    robot->setPacketsReceivedTracking(true);
  if (myRobotLogPacketsSent)
    robot->setPacketsSentTracking(true);
  if (myRobotLogMovementReceived)
    robot->setLogMovementReceived(true);
  if (myRobotLogMovementSent)
    robot->setLogMovementSent(true);
  if (myRobotLogVelocitiesReceived)
    robot->setLogVelocitiesReceived(true);
  if (myRobotLogActions)
    robot->setLogActions(true);


  // We see if we can open the tcp connection, if we can we'll assume
  // we're connecting to the sim, and just go on...  if we can't open
  // the tcp it means the sim isn't there, so just try the robot

  // see if we're doing remote host or not
  if (myRemoteHost != NULL)
    myRobotTcpConn.setPort(myRemoteHost, myRemoteRobotTcpPort);
  else
    myRobotTcpConn.setPort("localhost", myRemoteRobotTcpPort);

  // see if we can get to the simulator  (true is success)
  if (myRobotTcpConn.openSimple())
  {
    robot->setDeviceConnection(&myRobotTcpConn);
    // we could get to the sim, so set the robots device connection to the sim
    if (myRemoteHost != NULL)
    {
      ArLog::log(ArLog::Normal, "Connected to remote host %s through tcp.\n", 
		 myRemoteHost);
      if (myRemoteIsSim)
	myUsingSim = true;
      else
	myUsingSim = false;
    }
    else
    {
      ArLog::log(ArLog::Normal, "Connecting to simulator through tcp.\n");
      myUsingSim = true;
    }
  }
  else
  {
    // if we were trying for a remote host and it failed, just exit
    if (myRemoteHost != NULL)
    {
      ArLog::log(ArLog::Terse, "Could not connect robot to remote host %s, port %d.\n", myRemoteHost, myRemoteRobotTcpPort);
      return false;
    }
    // we couldn't get to the sim, so set the port on the serial
    // connection and then set the serial connection as the robots
    // device

    myRobotSerConn.setPort(myRobotPort);
    myRobotSerConn.setBaud(myRobotBaud);
    ArLog::log(ArLog::Normal,
	       "Could not connect to simulator, connecting to robot through serial port %s.", 
	       myRobotSerConn.getPort());
    robot->setDeviceConnection(&myRobotSerConn);
    myUsingSim = false;
  }
  return true;
}

/** Prepares the given ArRobot object for connection, then begins
 * a blocking connection attempt.
 * If you wish to simply prepare the ArRobot object, but not begin
 * the connection, then use setupRobot().
 */
AREXPORT bool ArRobotConnector::connectRobot(void)
{
  return connectRobot(myRobot);
}

/** Prepares the given ArRobot object for connection, then begins
 * a blocking connection attempt.
 * If you wish to simply prepare the ArRobot object, but not begin
 * the connection, then use setupRobot().
 */
AREXPORT bool ArRobotConnector::connectRobot(ArRobot *robot)
{
  if (!setupRobot(robot))
    return false;
  else
    return robot->blockingConnect();
}

AREXPORT const char *ArRobotConnector::getRemoteHost(void) const
{
  return myRemoteHost;
}


AREXPORT bool ArRobotConnector::getRemoteIsSim(void) const
{
  if (myRemoteIsSim) 
    return true;
  else if (myRemoteIsNotSim)
    return false;
  else if (myRobot != NULL && 
	   strcasecmp(myRobot->getRobotName(), "MobileSim") == 0)
    return true;
  else
    return false;
}

AREXPORT ArRobot *ArRobotConnector::getRobot(void) 
{
  return myRobot;
}
