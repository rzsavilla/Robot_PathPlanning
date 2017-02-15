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
#include "ArSocket.h"
#include "ArLog.h"
#include <stdio.h>
#include <string.h>
#include "ArFunctor.h"

/*
#include <sys/socket.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
*/

bool ArSocket::ourInitialized=false;


AREXPORT ArSocket::ArSocket() :
  myType(Unknown),
  myError(NoErr),
  myErrorStr(),
  myDoClose(true),
  myFD(-1),
  myNonBlocking(false),
  mySin()
{
  internalInit();
}

AREXPORT ArSocket::ArSocket(const char *host, int port, Type type) :
  myType(type),
  myError(NoErr),
  myErrorStr(),
  myDoClose(true),
  myFD(-1),
  myNonBlocking(false),
  mySin()
{
  internalInit();
  connect(host, port, type);
}

AREXPORT ArSocket::ArSocket(int port, bool doClose, Type type) :
  myType(type),
  myError(NoErr),
  myErrorStr(),
  myDoClose(doClose),
  myFD(-1),
  myNonBlocking(false),
  mySin()
{
  internalInit();
  open(port, type);
}

AREXPORT ArSocket::~ArSocket()
{
  close();
}

/** @return false failure. */
AREXPORT bool ArSocket::init()
{
  WORD wVersionRequested;
  WSADATA wsaData;

//  if (!ourInitialized)
  //{
  wVersionRequested=MAKEWORD( 2, 2 );
  
  if (WSAStartup(wVersionRequested, &wsaData) != 0)
  {
    ourInitialized=false;
    return(false);
  }
  
  ourInitialized=true;
  //}

  return(true);
}

AREXPORT void ArSocket::shutdown()
{
  if (ourInitialized)
  {
    WSACleanup();
    ourInitialized=false;
  }
}

/** @return false on failure */
AREXPORT bool ArSocket::hostAddr(const char *host, struct in_addr &addr)
{
  struct hostent *hp;

  if (!(hp=gethostbyname(host)))
  {
    perror("gethostbyname");
    memset(&addr, 0, sizeof(in_addr));
    return(false);
  }
  else
  {
    memcpy(&addr, hp->h_addr, hp->h_length);
    return(true);
  }
}

/** @return false on failure */
AREXPORT bool ArSocket::addrHost(struct in_addr &addr, char *host)
{
  struct hostent *hp;

  hp=gethostbyaddr((char*)&addr.s_addr, sizeof(addr.s_addr), AF_INET);
  if (hp)
    strcpy(host, hp->h_name);
  else
    strcpy(host, inet_ntoa(addr));

  return(true);
}

/** @return false and set error code and description string on failure */
AREXPORT bool ArSocket::connect(const char *host, int port, Type type,
				const char *openOnIP)
{
  char localhost[MAXGETHOSTSTRUCT];
  myError = NoErr;
  myErrorStr.clear();

  init();

  if (!host)
  {
    if (gethostname(localhost, sizeof(localhost)) == 1)
    {
      myError=ConBadHost;
      myErrorStr="Failure to locate host '";
      myErrorStr+=localhost;
      myErrorStr+="'";
      perror("gethostname");
      return(false);
    }
    host=localhost;
  }

  char useHost[1024];
  int usePort;
  separateHost(host, port, useHost, sizeof(useHost), &usePort);

  memset(&mySin, 0, sizeof(mySin));
  if ((mySin.sin_addr.s_addr = inet_addr(useHost)) == INADDR_NONE)
  {
    if (!hostAddr(host, mySin.sin_addr))
    {
      setRawIPString();
      myError = ConBadHost;
      myErrorStr = "Could not find the address of '";
      myErrorStr += host;
      myErrorStr += "'";
      return(false);
    }
  }

  mySin.sin_family=AF_INET;
  mySin.sin_port=hostToNetOrder(usePort);

  // WSA_FLAG_OVERLAPPED allows concurrent calls to select, read and send on the same socket,
  // which could happen occasionally. If OVERLAPPED is not enabled in this situation, calls can
  // hang mysteriously.
  // This flag is also required for all non-blocking sockets on Windows NT 4.0 (according to MS
  // Knowlege Base article Q179942)
  if ((type == TCP) && ((myFD=WSASocket(AF_INET, SOCK_STREAM, 0, NULL, 0, WSA_FLAG_OVERLAPPED)) < 0))
  {
    myError=NetFail;
    myErrorStr="Failure to make TCP socket";
    perror("socket");
    return(false);
  }
  else if ((type == UDP) && ((myFD=WSASocket(AF_INET, SOCK_DGRAM, 0, NULL, 0, WSA_FLAG_OVERLAPPED)) < 0))
  {
    myError=NetFail;
    myErrorStr="Failure to make UDP socket";
    perror("socket");
    return(0);
  }

  myType=type;

  if (::connect(myFD, (struct sockaddr *)&mySin,
		sizeof(struct sockaddr_in)) < 0)
  {
    char buff[10];
    int err=WSAGetLastError();
    sprintf(buff, "%d", err);
    myErrorStr="Failure to connect socket";
    myErrorStr+=buff;
    switch (err)
    {
    case WSAEADDRNOTAVAIL:
      myError=ConBadHost;
      break;
    case WSAECONNREFUSED:
      myError=ConRefused;
      break;
    case WSAENETUNREACH:
      myError=ConNoRoute;
      break;
    default:
      myError=NetFail;
      break;
    }
    //perror("connect");
    ::shutdown(myFD, SD_BOTH);
    closesocket(myFD);
    myFD = -1;
    return(0);
  }

  return(1);
}

/** @return false and set error code and description string on failure */
AREXPORT bool ArSocket::open(int port, Type type, const char *openOnIP)
{
  int ret;
  char localhost[MAXGETHOSTSTRUCT];
  myError = NoErr;
  myErrorStr.clear();

  if ((type == TCP) && ((myFD=socket(AF_INET, SOCK_STREAM, 0)) < 0))
  {
    //ret=WSAGetLastError();
    myError = NetFail;
    myErrorStr="Failure to make TCP socket";
    perror("socket");
    return(false);
  }
  else if ((type == UDP) && ((myFD=socket(AF_INET, SOCK_DGRAM, 0)) < 0))
  {
    myError = NetFail;
    myErrorStr="Failure to make UDP socket";
    perror("socket");
    return(false);
  }

  setLinger(0);
  setReuseAddress();

  /* MPL this is useless withw hat I Took out below
  if (gethostname(localhost, sizeof(localhost)) == 1)
  {
    myErrorStr="Failure to locate localhost";
    perror("gethostname");
    return(false);
  }
  */

  memset(&mySin, 0, sizeof(mySin));
  /* MPL took this out since it was just overriding it with the
     INADDR_ANY anyways and it could cause slowdowns if a machine wasn't
     configured so lookups are quick
  if (!hostAddr(localhost, mySin.sin_addr))
    return(false);
  */
  setRawIPString();
  if (openOnIP != NULL)
  {
    
    if (!hostAddr(openOnIP, mySin.sin_addr))
    {
      myError = NameLookup;
      myErrorStr = "Name lookup failed";
      ArLog::log(ArLog::Normal, "Couldn't find ip of %s to open on", openOnIP);
      return(false); 
    }
    else
    {
      //printf("Opening on %s\n", openOnIP);
    }
  }
  else
  {
    mySin.sin_addr.s_addr=htonl(INADDR_ANY);
  }
  mySin.sin_family=AF_INET;
  mySin.sin_port=hostToNetOrder(port);

  myType=type;

  if ((ret=bind(myFD, (struct sockaddr *)&mySin, sizeof(mySin))) < 0)
  {
    myErrorStr="Failure to bind socket to port ";
    sprintf(localhost, "%d", port);
    myErrorStr+=localhost;
    myError = NetFail;
    perror("socket");
    return(false);
  }

  if ((type == TCP) && (listen(myFD, 5) < 0))
  {
    myErrorStr="Failure to listen on socket";
    myError = NetFail;
    perror("listen");
    return(false);
  }

  return(true);
}

/** @return false and set error code and description string on failure */
AREXPORT bool ArSocket::create(Type type)
{
  myError = NoErr;
  myErrorStr.clear();
  if ((type == TCP) && ((myFD=socket(AF_INET, SOCK_STREAM, 0)) < 0))
  {
    myErrorStr="Failure to make TCP socket";
    myError = NetFail;
    perror("socket");
    return(false);
  }
  else if ((type == UDP) && ((myFD=socket(AF_INET, SOCK_DGRAM, 0)) < 0))
  {
    myErrorStr="Failure to make UDP socket";
    myError = NetFail;
    perror("socket");
    return(false);
  }

/*
  int zero = 0;
  if (setsockopt(myFD, SOL_SOCKET, SO_SNDBUF, (char *)&zero, sizeof(zero)) != 0)
  {
    perror("setsockopt");
    ArLog::log(ArLog::Normal, "Could not set SNDBUF %d", WSAGetLastError());
    return(false);
  }

  if (setsockopt(myFD, SOL_SOCKET, SO_RCVBUF, (char *)&zero, sizeof(zero)) != 0)
  {
    perror("setsockopt");
    ArLog::log(ArLog::Normal, "Could not set SNDBUF %d", WSAGetLastError());
    return(false);
  }

  myType=type;
*/
  /*if (getSockName())
    return(true);
  else
    return(false);*/
  return(true);
}

/** @return false on failure */
AREXPORT bool ArSocket::findValidPort(int startPort, const char *openOnIP)
{

  /*
  char localhost[MAXGETHOSTSTRUCT];
  if (gethostname(localhost, sizeof(localhost)) == 1)
  {
    myErrorStr="Failure to locate localhost";
    perror("gethostname");
    return(false);
  }
  */
  for (int i=0; i+startPort < 65000; ++i)
  {
    memset(&mySin, 0, sizeof(mySin));
    /*
    if (!hostAddr(localhost, mySin.sin_addr))
      return(false);
    */
    setRawIPString();
    if (openOnIP != NULL)
    {
      if (!hostAddr(openOnIP, mySin.sin_addr))
      {
        ArLog::log(ArLog::Normal, "Couldn't find ip of %s to open on", openOnIP);
        return(false); 
      }
      else
      {
	//printf("Opening on %s\n", openOnIP);
      }
    }
    else
    {
      mySin.sin_addr.s_addr=htonl(INADDR_ANY);
    }

    mySin.sin_family=AF_INET;
    //mySin.sin_addr.s_addr=htonl(INADDR_ANY);
    mySin.sin_port=hostToNetOrder(startPort+i);

    if (bind(myFD, (struct sockaddr *)&mySin, sizeof(mySin)) == 0)
      break;
  }

  return(true);
}

/** @return false and set error code and description string on failure */
AREXPORT bool ArSocket::connectTo(const char *host, int port)
{

  char localhost[MAXGETHOSTSTRUCT];
  myError = NoErr;
  myErrorStr.clear();

  if (myFD < 0)
    return(false);

  if (!host)
  {
    if (gethostname(localhost, sizeof(localhost)) == 1)
    {
      myErrorStr="Failure to locate host '";
      myErrorStr+=localhost;
      myErrorStr+="'";
      myError = ConBadHost;
      perror("gethostname");
      return(false);
    }
    host=localhost;
  }

  char useHost[1024];
  int usePort;
  separateHost(host, port, useHost, sizeof(useHost), &usePort);


  memset(&mySin, 0, sizeof(mySin));
  if (!hostAddr(useHost, mySin.sin_addr))
  {
    myError = ConBadHost;
    return(false);
  }
  setRawIPString();
  mySin.sin_family=AF_INET;
  mySin.sin_port=hostToNetOrder(usePort);

  return(connectTo(&mySin));
}

/** @return false and set error code and description string on failure */
AREXPORT bool ArSocket::connectTo(struct sockaddr_in *sin)
{
  myError = NoErr;
  myErrorStr.clear();
  if (::connect(myFD, (struct sockaddr *)sin,
		sizeof(struct sockaddr_in)) < 0)
  {
    myErrorStr="Failure to connect socket";
    myError = NetFail;
    //perror("connect");
    return(0);
  }

  return(1);
}

/** @return false and set error code and description string on failure */
AREXPORT bool ArSocket::close()
{
  myError = NoErr;
  myErrorStr.clear();
  if (myFD != -1)
    ArLog::log(ArLog::Verbose, "Closing socket");
  if (myCloseFunctor != NULL)
    myCloseFunctor->invoke();
  if (myDoClose && (myFD >= 0))
  {
    ::shutdown(myFD, SD_BOTH);
    closesocket(myFD);
    myFD = -1;
    return(true);
  }
  return(false);
}

/** @return false and set error code and description string on failure */
AREXPORT bool ArSocket::setLinger(int time)
{
  struct linger lin;
  myError = NoErr;
  myErrorStr.clear();

  if (time)
  {
    lin.l_onoff=1;
    lin.l_linger=time;
  }
  else
  {
    lin.l_onoff=0;
    lin.l_linger=time;
  }

  if (setsockopt(myFD, SOL_SOCKET, SO_LINGER, (char*)&lin, sizeof(lin)) != 0)
  {
    myErrorStr="Failure to setsockopt LINGER";
    myError = NetFail;
    perror("setsockopt");
    return(false);
  }
  else
    return(true);
}

/** @return false and set error code and description string on failure */
AREXPORT bool ArSocket::setBroadcast()
{
  myError = NoErr;
  myErrorStr.clear();
  if (setsockopt(myFD, SOL_SOCKET, SO_BROADCAST, NULL, 0) != 0)
  {
    myError = NetFail;
    myErrorStr="Failure to setsockopt BROADCAST";
    perror("setsockopt");
    return(false);
  }
  else
    return(true);
}

/** @return false and set error code and description string on failure 
    @internal
    @note ArSocket always sets the reuse-address option in open(), so calling this function is normally unneccesary.
     (This apparently needs to be done after the socket is created before
     the socket is bound.)
*/
AREXPORT bool ArSocket::setReuseAddress()
{
  int opt=1;
  myError = NoErr;
  myErrorStr.clear();
  if (setsockopt(myFD, SOL_SOCKET, SO_REUSEADDR,
		 (char*)&opt, sizeof(opt)) != 0)
  {
    myErrorStr="Failure to setsockopt REUSEADDR";
    myError = NetFail;
    perror("setsockopt");
    return(false);
  }
  else
    return(true);
}

/** @return false and set error code and description string on failure  */
AREXPORT bool ArSocket::setNonBlock()
{
  u_long arg=1;
  myError = NoErr;
  myErrorStr.clear();
  if (ioctlsocket(myFD, FIONBIO, &arg) != 0)
  {
    myErrorStr="Failure to fcntl O_NONBLOCK";
    myError = NetFail;
    perror("fcntl");
    return(false);
  }
  else
  {
    myNonBlocking = true;
    return(true);
  }
}

/** @return false and set error code and description string on failure  */
AREXPORT bool ArSocket::copy(int fd, bool doclose)
{
  int len;

  myFD=fd;
  myDoClose=doclose;
  myError = NoErr;
  myErrorStr.clear();

  len=sizeof(struct sockaddr_in);
  if (getsockname(myFD, (struct sockaddr*)&mySin, &len))
  {
    myErrorStr="Failed to getsockname on fd ";
    myError = NetFail;
    perror("getsockname");
    return(false);
  }
  else
    return(true);
}

/** @return false and set error code and description string on failure  */
AREXPORT bool ArSocket::accept(ArSocket *sock)
{
  int len;
  //unsigned char *bytes;
  myError = NoErr;
  myErrorStr.clear(); 
  len=sizeof(struct sockaddr_in);
  sock->myFD=::accept(myFD, (struct sockaddr*)&(sock->mySin), &len);
  sock->myType=myType;
  sock->setRawIPString();
  /*
  bytes = (unsigned char *)sock->inAddr();
  sprintf(sock->myIPString, "%d.%d.%d.%d", bytes[0], bytes[1], bytes[2], 
	  bytes[3]);
  */
  if ((sock->myFD < 0 && !myNonBlocking) || 
      (sock->myFD < 0 && WSAGetLastError() != WSAEWOULDBLOCK && myNonBlocking))
  {
    myErrorStr="Failed to accept on socket";
    myError = ConRefused;
    perror("accept");
    return(false);
  }

  return(true);
}

AREXPORT void ArSocket::inToA(struct in_addr *addr, char *buff)
{
  strcpy(buff, inet_ntoa(*addr));
}

/** @return false and set error code and description string on failure  */
AREXPORT bool ArSocket::getSockName()
{
  int size;

  myError = NoErr;
  myErrorStr.clear();
  if (myFD < 0)
  {
    myErrorStr="Trying to get socket name on an unopened socket";
    myError = NetFail;
    printf(myErrorStr.c_str());
    return(false);
  }

  size=sizeof(mySin);
  if (getsockname(myFD, (struct sockaddr *)&mySin, &size) != 0)
  {
    myErrorStr="Error getting socket name";
    switch (WSAGetLastError())
    {
    case WSAEINVAL:
      myErrorStr+=": inval";
      break;
    case WSANOTINITIALISED:
      myErrorStr+=": not init";
      break;
    }
    myError = NetFail;
    perror(myErrorStr.c_str());
    return(false);
  }

  return(true);
}

AREXPORT unsigned int ArSocket::hostToNetOrder(int i)
{
  return(htons(i));
}

AREXPORT unsigned int ArSocket::netToHostOrder(int i)
{
  return(ntohs(i));
}

AREXPORT std::string ArSocket::getHostName()
{
  char localhost[MAXGETHOSTSTRUCT];

  if (gethostname(localhost, sizeof(localhost)) == 1)
    return("");
  else
    return(localhost);
}

/** If this socket is a TCP socket, then set the TCP_NODELAY flag to 1,
 *  to disable the use of the Nagle algorithm (which waits until enough
 *  data is ready to send to fill a TCP frame, rather then sending the
 *  packet immediately).
 *  @return true of the flag was successfully set, false if there was an 
 *    error or this socket is not a TCP socket.
 */
AREXPORT bool ArSocket::setNoDelay(bool flag)
{
  if(myType != TCP) return false;
  int f = flag?1:0;
  int r = setsockopt(myFD, IPPROTO_TCP, TCP_NODELAY, (char*)&f, sizeof(f));
  return (r != -1);
}

