# w5100.cpp 
1. Added code to record last time each socket was used
2. Set TTL according to IANA recommendation
3. Moved delay in W5100Class::softReset() to comply with recommendation in W5500 datasheet, Section 5.5.1. Delay is now between the reset command and the first poll of MR.
4. Modified W5100Class::execCmdSn() as follows:
	a. Clear Sn_IR timeout bit before sending command
	b. Added 1 microsecond delay between polls of Sn_CR because I'd noticed that connect and disconnect were not reliable without it when running a W5500 with the Adafruit - Metro M4 Express.
5. Added W5100Class::waitForCmd(), which waits for Sn_SR to change to expected state after a command is sent. There are two variants, one with a single expected value, and the other with three. Each polls Sn_SR and Sn_IR and loops until either the expected value is seen in Sn_SR or Sn_IR indicates a timeout or disconnected socket. Both are instrumented to collect the number of times called and length of time spent in them; the average time in these functions can be retrieved by calling W5100Class::getAvgWait(). \f1\fs20 It may seem counterintuitive,but when I disabled these functions by returning "true" immediately upon entry my average Ethernet throughput dropped by 200 KBps.
#  w5100.h
1. Increased SPI clock speed from 14000000 to 33000000 per W5500 datasheet. Should stick with the former for W5100.
2. Added declarations to support changes to w5100.cpp
3. Modified register offsets to conform with those in W5500 datasheet. This probably broke the ability to set the W5100 timeout period (see RTR (Retry Time-value Register)), which didn't work properly for the W5500. If you're using the W5100, comment out the __GP_REGISTERxx definitions for the W5500 starting with RTR and ending with UPORT and uncomment the corresponding definitions for the W5100, which are still in the file.
4. Ditto for __SOCKET_REGISTER8(SnPROTO, which is at a reserved address in the W5500.
5. Added definitions for missing W5500 socket registers Sn_IMR, Sn_FRAG and Sn_KPALVTR.
#  Ethernet.cpp
1. Added variable declarations to store sockets' last time of use.
2. Added function uint8_t EthernetClass::linkRawStatus() which returns the status of the Ethernet link as indicated by the W5x00.
#  Ethernet.h
1. Hard-coded MAX_SOCK_NUM to 8 because it wasn't being set correctly for my W5500. You'll want to change this to 4 if you're using a W5100.
2. Defined the Time-To-Live value encoded in packets to conform to the recommendation at https://www.iana.org/assignments/ip-parameters/ip-parameters.xhtml#ip-parameters-2
3. Declared new functions linkRawStatus(), getWaitTime() and manageSockets().
4. Added declaration of EthernetClient::getSocketProtocol().
5. Added declaration of EthernetServer::begin2(), which is exactly the same as EthernetServer::begin() except it returns success or failure as a bool, whereas begin() returns nothing. I didn't want to break the Arduino interface and the IDE wouldn't let me overload begin().
5. Added declaration of EthernetServer::statusreport()
#  EthernetClient.cpp
1. EthernetClient::connect() [both versions. probably not needed in the first one, which calls the second]- added SnSR::FIN_WAIT, SnSR::CLOSING, SnSR::TIME_WAIT, SnSR::CLOSE_WAIT & SnSR::LAST_ACK to the socket statuses that make a socket eligible for being disconnected for reuse. These choices were made based on the table at {{\field{\*\fldinst{HYPERLINK http://tcpipguide.com/free/t_TCPOperationalOverviewandtheTCPFiniteStateMachineF-2.htm }}{\fldrslt{http://tcpipguide.com/free/t_TCPOperationalOverviewandtheTCPFiniteStateMachineF-2.htm\ul0\cf0}}}}\f1\fs20  and the W5500 datasheet section describing the Sn_SR register.
2. EthernetClient::connect(), IP version - replaced timeout code with check of Sn_IR timeout bit and reduced loop delay from 1ms to 5 microseconds
3. EthernetClient::connected() - added code to set socket's sockindex to MAX_SOCK_NUM if closed because other changes allowed sockets to be closed without this useful flag being set.
4. Added EthernetClient::getSocketProtocol(), which allows programs to determine whether a particular client is open in TCP or UDP mode.
#  EthernetServer.cpp
1. Added EthernetServer::begin2(), which is identical to EthernetServer::begin() except that it returns a bool value indicating whether a socket was placed in LISTEN mode or not.
2. EthernetServer::available() - removed if (!listening) begin(); from the end as this function is now managed by manageSockets(). Also removed check for listening socket.
3. EthernetServer::accept() - same changes as were made to EthernetServer::available().
4. Changed #if 0 to #if 1 so EthernetServer::statusreport() is built.
#  socket.cpp
1. Added macro definitions for DEBUG_PRINT variants.
2. EthernetClass::socketBegin() - removed socket management functionality, which is now consolidated in manageSockets(). Removed 250 microsecond delay which had no obvious function.
3. Added calls to W5100.waitForCmd() after each call to W5100.execCmdSn() to ensure command had completed before continuing per W5500 datasheet. This allowed other conditions such as Sn_IR::TIMEOUT and Sn_IR::DISCONNECT to be checked after each command, thus eliminating a cause of library infinite loops.
4. Added code to record each time a socket was "handled" in some way, whether sending it a command or reading or writing data.
5. EthernetClass::socketConnect - added a 90ms delay between setting destination IP and Port and sending Sock_CONNECT command because my testing with Wireshark reliably caught mishandled SYN/ACK exchanges without it. I started with a longer delay and reduced it incrementally until errors returned. YMMV depending on your MCU.
6. EthernetClass::socketSend() - added a timeout after observing this function hang up forever waiting for free space after remote client crashed, leaving W5500 unaware that the connection was no longer active.
7. Added EthernetClass::manageSockets(). This function should be called in every pass through your webserver. It takes 3 parameters:
   a. *server - your server object, which is used to start new listening sockets
   b. maxListeners - the maximum number of sockets you want to be in LISTEN mode at the same time. Modern browsers typically open up to 6 at a time, and shares them among. Why might you want to limit this, you ask? Well, if you need to get an IP address or run NTP to update your server's time, you'll want to have a spare socket or two available.
   c. doNotDisconnectSocket  - if you have a client that you want to remain constantly connected for some reason, put its socket number here. You can use client.getSocketNumber() for this purpose.
   EthernetClass::manageSockets() does the following:
   a. Counts the sockets that are disconnecting, closed (and thus available), established (connected to a client) and listening. It closes any that have been closed by the remote client and have no data waiting to be read.
   b. IF you pass a non-null pointer to a server object, it starts listening on all closed (available) sockets but one, leaving that one for other uses (see "b" above).
   c. Disconnects sockets in ESTABLISHED state with no data available to read that haven't been used lately. This is where recording the "last time the socket was used" is put to use. All such sockets idle for an arbitrarily chosen period of 30 seconds are gracefully -  closed by sending a \f3\fs19 Sock_DISCON\f4\lang1033  \f1\fs20\lang9 command. In the case where we're really hurting for -  sockets (one socket listening, none available (closed) and none disconnecting), an -  arbitrary period of 5 seconds is used instead to make sure we don't find ourselves unable -  to respond to a new connection attempt. AND... I arbitrarily chose to leave 2 otherwise -  idle sockets connected to improve performance if the same browser tries to reuse them.
	  Are these the optimal values? I don't know yet; still testing. They don't seem horrible though. Feel free to change them to whatever works best for you, or change them to user-passed parameters.
      I'd appreciate your feedback if you find other values work better for you. Please describe what your system does so others can understand why your changes work better for you. Oh, my system is a webserver :).
8. Added EthernetClass::getWaitTime() to give Arduino programs a way to retrieve the otherwise inaccessible average time spent waiting for W5100 commands to complete
