RoboDK Driver for ABB
----------------------

These files require the latest version of the ABBDriver, included with RoboDK v5.6.3 and later.

You should only load one of the 2 files to the controller depending on the type of the connection.

For RobotWare version 5 and 6, use:
 * Serial/RS232    -> RDK_DriverSerial.mod
 * Socket/Ethernet -> RDK_DriverSocket.mod
 
For RobotWare version 7 and up, use:
 * Serial/RS232    -> RDK_DriverSerial_RW7.mod
 * Socket/Ethernet -> RDK_DriverSocket_RW7.mod
 