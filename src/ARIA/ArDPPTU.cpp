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
#include "ArDPPTU.h"
#include "ArCommands.h"

AREXPORT ArDPPTUPacket::ArDPPTUPacket(ArTypes::UByte2 bufferSize) :
  ArBasePacket(bufferSize, 0)
{
}

AREXPORT ArDPPTUPacket::~ArDPPTUPacket()
{

}

AREXPORT void ArDPPTUPacket::byte2ToBuf(int val)
{
  //ArLog::log(ArLog::Normal, "Putting %d in an DPPTU packet...", val);
  int i;
  char buf[8];
  if (myLength + 4 > myMaxLength)
  {
    ArLog::log(ArLog::Terse, "ArDPPTUPacket::byte2ToBuf: Trying to add beyond length of buffer.");
    return;
  }

  if(val > 9999999 || val < -999999) 
	  ArLog::log(ArLog::Terse, "ArDPPTUPacket::byte2ToBuf: Warning: truncating value %d to 7 digits!", val);

  snprintf(buf, 8, "%d", val);

  for (i=0;i<(int)strlen(buf);i++)
  {
      myBuf[myLength] = buf[i];
      ++myLength;
  }
}

AREXPORT void ArDPPTUPacket::finalizePacket(void)
{
    ArDPPTUPacket::uByteToBuf(ArDPPTUCommands::DELIM);
}

/** @a deviceType selects different parameters for different DPPTU models. 
 *    For most DPPTUs, the default is correct. For others, the type
 *    must be specified so the correct position values will be used.
 */
AREXPORT ArDPPTU::ArDPPTU(ArRobot *robot, DeviceType deviceType) :
  ArPTZ(robot)
{
  myRobot = robot;
  myDeviceType = deviceType;

  switch(myDeviceType) {
    case PANTILT_PTUD47:
      myMaxPan = 158;
      myMinPan = -158;
      myMaxTilt = 30;
      myMinTilt = -46;
      myMaxPanSlew = 149;
      myMinPanSlew = 2;
      myMaxTiltSlew = 149;
      myMinTiltSlew = 2;
      myMaxPanAccel = 102;
      myMinPanAccel = 2;
      myMaxTiltAccel = 102;
      myMinTiltAccel = 2;
      myPanConvert = 0.0514;
      myTiltConvert = 0.0129;
      myPanSlew = 40;
      myTiltSlew = 40;
      break;
    case PANTILT_DEFAULT:
      myMaxPan = 158;
      myMinPan = -158;
      myMaxTilt = 30;
      myMinTilt = -46;
      myMaxPanSlew = 149;
      myMinPanSlew = 2;
      myMaxTiltSlew = 149;
      myMinTiltSlew = 2;
      myMaxPanAccel = 102;
      myMinPanAccel = 2;
      myMaxTiltAccel = 102;
      myMinTiltAccel = 2;
      myPanConvert = 0.0514;
      myTiltConvert = 0.0514;
      myPanSlew = 40; //Default to 1000 positions/sec
      myTiltSlew = 40; //Defaults to 1000 positions/sec
    default:
      break;
  }
}

AREXPORT ArDPPTU::~ArDPPTU()
{
}

void ArDPPTU::preparePacket(void)
{
  myPacket.empty();
  myPacket.byteToBuf(ArDPPTUCommands::DELIM);
}

AREXPORT bool ArDPPTU::init(void)
{
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::INIT);

  if (!sendPacket(&myPacket))
    return false;

  myPan = -1;  //myPan and myTilt set to -1 for initial positioning
  myTilt = -1;

  if (!panTilt(0,0))
    return false;

  switch(myDeviceType) {
    case PANTILT_PTUD47:
      //Assuming default accel and slew rates
      myPanSlew = 40;
      myBasePanSlew = 40;
      myTiltSlew = 40;
      myBaseTiltSlew = 40;
      myPanAccel = 80;
      myTiltAccel = 80;
      break;
    case PANTILT_DEFAULT:
    default:
      //Assuming default accel and slew rates
      myPanSlew = 40; // 1000 positions/sec
      myBasePanSlew = 40; // 1000 positions/sec
      myTiltSlew = 40; // 1000 positions/sec
      myBaseTiltSlew = 40; // 1000 positions/sec
      myPanAccel = 80; // 2000 positions/sec^2
      myTiltAccel = 80; // 2000 positions/sec^2
      break;
  }

  return true;
}

/** A blank packet can be sent to exit monitor mode **/
AREXPORT bool ArDPPTU::blank(void)
{
  myPacket.empty();
  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::panTilt(double pdeg, double tdeg)
{
  //ArLog::log(ArLog::Normal, "ArDPPTU::panTilt(%f, %f)", pdeg, tdeg);
  if (pdeg > getMaxPosPan())
    pdeg = getMaxPosPan();
  if (pdeg < getMaxNegPan())
    pdeg = getMaxNegPan();

  if (tdeg > getMaxPosTilt())
    tdeg = getMaxPosTilt();
  if (tdeg < getMaxNegTilt())
    tdeg = getMaxNegTilt();

  if (pdeg != myPan)
  {
	//ArLog::log(ArLog::Normal, "ArDPPTU::panTilt: sending command to pan %d deg (maxPosPan=%f, minNegPan=%f)", pdeg, getMaxPosPan(), getMaxNegPan());
    preparePacket();
    myPacket.byteToBuf(ArDPPTUCommands::PAN);
    myPacket.byteToBuf(ArDPPTUCommands::PAN);
    myPacket.byte2ToBuf(ArMath::roundInt(pdeg/myPanConvert));

    myPan = pdeg;
    if (!sendPacket(&myPacket)) return false;
  }

  if (tdeg != myTilt)
  {
	//ArLog::log(ArLog::Normal, "ArDPPTU::panTilt: sending command to tilt %d deg (maxPosTilt=%f, minNegTilt=%f)", tdeg, getMaxPosTilt(), getMaxNegTilt());
    preparePacket();
    myPacket.byteToBuf(ArDPPTUCommands::TILT);
    myPacket.byteToBuf(ArDPPTUCommands::PAN);
    myPacket.byte2ToBuf(ArMath::roundInt(tdeg/myTiltConvert));

    myTilt = tdeg;
    if (!sendPacket(&myPacket)) return false;
  }

  return true;
}

AREXPORT bool ArDPPTU::panSlew(double deg)
{
  if (deg > getMaxPanSlew())
    deg = getMaxPanSlew();
  if (deg < getMinPanSlew())
    deg = getMinPanSlew();
  
  myPanSlew = deg;
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::PAN);
  myPacket.byteToBuf(ArDPPTUCommands::SPEED);

  myPacket.byte2ToBuf(ArMath::roundInt(deg/myPanConvert));

  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::tiltSlew(double deg)
{
  if (deg > getMaxTiltSlew())
    deg = getMaxTiltSlew();
  if (deg < getMinTiltSlew())
    deg = getMinTiltSlew();
  
  myTiltSlew = deg;
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::TILT);
  myPacket.byteToBuf(ArDPPTUCommands::SPEED);

  myPacket.byte2ToBuf(ArMath::roundInt(deg/myTiltConvert));

  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::resetCalib(void)
{
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::RESET);
  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::disableReset(void)
{
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::RESET);
  myPacket.byteToBuf(ArDPPTUCommands::DISABLE);

  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::resetTilt(void)
{
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::RESET);
  myPacket.byteToBuf(ArDPPTUCommands::TILT);

  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::resetPan(void)
{
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::RESET);
  myPacket.byteToBuf(ArDPPTUCommands::PAN);

  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::resetAll(void)
{
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::RESET);
  myPacket.byteToBuf(ArDPPTUCommands::ENABLE);

  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::saveSet(void)
{
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::DISABLE);
  myPacket.byteToBuf(ArDPPTUCommands::SPEED);

  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::restoreSet(void)
{
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::DISABLE);
  myPacket.byteToBuf(ArDPPTUCommands::RESET);

  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::factorySet(void)
{
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::DISABLE);
  myPacket.byteToBuf(ArDPPTUCommands::FACTORY);

  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::limitEnforce(bool val)
{
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::LIMIT);

  if (val)
    myPacket.byteToBuf(ArDPPTUCommands::ENABLE);
  else
    myPacket.byteToBuf(ArDPPTUCommands::DISABLE);

  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::immedExec(void)
{
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::IMMED);

  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::slaveExec(void)
{
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::SPEED);

  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::awaitExec(void)
{
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::ACCEL);

  return sendPacket(&myPacket);
}


AREXPORT bool ArDPPTU::haltAll(void)
{
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::HALT);

  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::haltPan(void)
{
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::HALT);
  myPacket.byteToBuf(ArDPPTUCommands::PAN);

  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::haltTilt(void)
{
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::HALT);
  myPacket.byteToBuf(ArDPPTUCommands::TILT);

  return sendPacket(&myPacket);
}


AREXPORT bool ArDPPTU::panAccel(double deg)
{
  if (deg > getMaxPanAccel())
    deg = getMaxPanAccel();
  if (deg < getMinPanAccel())
    deg = getMinPanAccel();

  if (myPanAccel != deg) {
    preparePacket();
    myPacket.byteToBuf(ArDPPTUCommands::PAN);
    myPacket.byteToBuf(ArDPPTUCommands::ACCEL);
    myPacket.byte2ToBuf(ArMath::roundInt(deg/myPanConvert));

    return sendPacket(&myPacket);
  }

  return true;
}

AREXPORT bool ArDPPTU::tiltAccel(double deg)
{
  if (deg > getMaxPanAccel())
    deg = getMaxPanAccel();
  if (deg < getMinPanAccel())
    deg = getMinPanAccel();

  if (myTiltAccel != deg) {
    preparePacket();
    myPacket.byteToBuf(ArDPPTUCommands::TILT);
    myPacket.byteToBuf(ArDPPTUCommands::ACCEL);
    myPacket.byte2ToBuf(ArMath::roundInt(deg/myTiltConvert));

    return sendPacket(&myPacket);
  }

  return true;
}

AREXPORT bool ArDPPTU::basePanSlew(double deg)
{
  myBasePanSlew = deg;

  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::PAN);
  myPacket.byteToBuf(ArDPPTUCommands::BASE);
  myPacket.byte2ToBuf(ArMath::roundInt(deg/myPanConvert));

  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::baseTiltSlew(double deg)
{
  myBaseTiltSlew = deg;

  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::TILT);
  myPacket.byteToBuf(ArDPPTUCommands::BASE);
  myPacket.byte2ToBuf(ArMath::roundInt(deg/myTiltConvert));

  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::upperPanSlew(double deg)
{
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::PAN);
  myPacket.byteToBuf(ArDPPTUCommands::UPPER);
  myPacket.byte2ToBuf(ArMath::roundInt(deg/myPanConvert));

  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::lowerPanSlew(double deg)
{
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::PAN);
  myPacket.byteToBuf(ArDPPTUCommands::LIMIT);
  myPacket.byte2ToBuf(ArMath::roundInt(deg/myPanConvert));

  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::upperTiltSlew(double deg)
{
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::TILT);
  myPacket.byteToBuf(ArDPPTUCommands::UPPER);
  myPacket.byte2ToBuf(ArMath::roundInt(deg/myTiltConvert));

  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::lowerTiltSlew(double deg)
{
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::TILT);
  myPacket.byteToBuf(ArDPPTUCommands::LIMIT);
  myPacket.byte2ToBuf(ArMath::roundInt(deg/myTiltConvert));

  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::indepMove(void)
{
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::CONTROL);
  myPacket.byteToBuf(ArDPPTUCommands::IMMED);

  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::velMove(void)
{
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::CONTROL);
  myPacket.byteToBuf(ArDPPTUCommands::VELOCITY);

  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::enMon(void)
{
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::MONITOR);
  myPacket.byteToBuf(ArDPPTUCommands::ENABLE);

  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::disMon(void)
{
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::MONITOR);
  myPacket.byteToBuf(ArDPPTUCommands::DISABLE);

  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::initMon(double deg1, double deg2, 
			       double deg3, double deg4)
{

  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::MONITOR);

  myPacket.byte2ToBuf(ArMath::roundInt(deg1/myPanConvert));
  myPacket.byteToBuf(',');
  myPacket.byte2ToBuf(ArMath::roundInt(deg2/myPanConvert));
  myPacket.byteToBuf(',');
  myPacket.byte2ToBuf(ArMath::roundInt(deg3/myTiltConvert));
  myPacket.byteToBuf(',');
  myPacket.byte2ToBuf(ArMath::roundInt(deg4/myTiltConvert));

  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::offStatPower(void)
{
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::PAN);
  myPacket.byteToBuf(ArDPPTUCommands::HALT);
  myPacket.byteToBuf(ArDPPTUCommands::OFFSET);

  if (!sendPacket(&myPacket))
    return false;

  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::TILT);
  myPacket.byteToBuf(ArDPPTUCommands::HALT);
  myPacket.byteToBuf(ArDPPTUCommands::OFFSET);

  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::regStatPower(void)
{
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::PAN);
  myPacket.byteToBuf(ArDPPTUCommands::HALT);
  myPacket.byteToBuf(ArDPPTUCommands::RESET);

  if (!sendPacket(&myPacket))
    return false;

  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::TILT);
  myPacket.byteToBuf(ArDPPTUCommands::HALT);
  myPacket.byteToBuf(ArDPPTUCommands::RESET);

  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::lowStatPower(void)
{
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::PAN);
  myPacket.byteToBuf(ArDPPTUCommands::HALT);
  myPacket.byteToBuf(ArDPPTUCommands::LIMIT);

  if (!sendPacket(&myPacket))
    return false;

  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::TILT);
  myPacket.byteToBuf(ArDPPTUCommands::HALT);
  myPacket.byteToBuf(ArDPPTUCommands::LIMIT);

  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::lowMotPower(void)
{
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::PAN);
  myPacket.byteToBuf(ArDPPTUCommands::MONITOR);
  myPacket.byteToBuf(ArDPPTUCommands::LIMIT);

  if (!sendPacket(&myPacket))
    return false;

  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::TILT);
  myPacket.byteToBuf(ArDPPTUCommands::MONITOR);
  myPacket.byteToBuf(ArDPPTUCommands::LIMIT);

  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::regMotPower(void)
{
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::PAN);
  myPacket.byteToBuf(ArDPPTUCommands::MONITOR);
  myPacket.byteToBuf(ArDPPTUCommands::RESET);

  if (!sendPacket(&myPacket))
    return false;

  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::TILT);
  myPacket.byteToBuf(ArDPPTUCommands::MONITOR);
  myPacket.byteToBuf(ArDPPTUCommands::RESET);

  return sendPacket(&myPacket);
}

AREXPORT bool ArDPPTU::highMotPower(void)
{
  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::PAN);
  myPacket.byteToBuf(ArDPPTUCommands::MONITOR);
  myPacket.byteToBuf(ArDPPTUCommands::HALT);

  if (!sendPacket(&myPacket))
    return false;

  preparePacket();
  myPacket.byteToBuf(ArDPPTUCommands::TILT);
  myPacket.byteToBuf(ArDPPTUCommands::MONITOR);
  myPacket.byteToBuf(ArDPPTUCommands::HALT);

  return sendPacket(&myPacket);
}
