#include <fstream>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <string>
#include <SerialStream.h>
#include "serial/serial.h"

using namespace std;

class SerialCom {
 public:

 	void Send(string data) {
 		data.append("\n");
		char *out_buf = (char*)data.c_str();
    ROS_INFO("Writing %s", out_buf);
    serial_port.write(out_buf, data.length());
    Listen();
 	}

 	void Open(const std::string &addr) {
    serial_port.Open(addr) ;
    if ( ! serial_port.IsOpen() ) {
      std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                   << "Error: Could not open serial port."
                   << std::endl ;
      exit(1) ;
    }
    // Set the number of data bits.
    serial_port.SetCharSize( LibSerial::SerialStreamBuf::CHAR_SIZE_8 ) ;
    if ( ! serial_port.IsOpen() ) {
      std::cerr << "Error: Could not set the character size." << std::endl ;
      exit(1) ;
    }
    // Disable parity.
    serial_port.SetParity( LibSerial::SerialStreamBuf::PARITY_NONE ) ;
    if ( ! serial_port.IsOpen() ) {
      std::cerr << "Error: Could not disable the parity." << std::endl ;
      exit(1) ;
    }
    // Set the number of stop bits.
    serial_port.SetNumOfStopBits( 1 ) ;
    if ( ! serial_port.IsOpen() ) {
      std::cerr << "Error: Could not set the number of stop bits." << std::endl ;
      exit(1) ;
    }
    // Turn off hardware flow control.
    serial_port.SetFlowControl( LibSerial::SerialStreamBuf::FLOW_CONTROL_NONE ) ;
    if ( ! serial_port.IsOpen() ) {
      std::cerr << "Error: Could not use hardware flow control." << std::endl ;
      exit(1) ;
    }
   /* ROS_INFO("baud rate is %d, should be %d", serial_port.BaudRate(), SerialStreamBuf::BAUD_115200);
    serial_port.SetBaudRate( SerialStreamBuf::BAUD_115200 ) ;
    if ( ! serial_port.IsOpen() ) {
      std::cerr << "Error: Could not set baud rate." << std::endl ;
      exit(1) ;
    }*/
    serial::Serial ser; //necessary because libserial fails to set correct baud rate
    ser.setPort(addr);
    ser.setBaudrate(115200);
    ser.open();
    ROS_INFO("baud rate enum is %d, should be %d", serial_port.BaudRate(), LibSerial::SerialStreamBuf::BAUD_115200);
  }

  void Close() {
    serial_port.Close();
  }

  SerialCom() {

  }

  SerialCom(const string &addr) {
    Open(addr);
  }

  ~SerialCom() {
 		Close();
 	}

 private:
 	LibSerial::SerialStream serial_port;
 	string address;

 	void Listen() {
    while(  serial_port.rdbuf()->in_avail() > 0  ){
      char next_byte;
      serial_port.get(next_byte);
      std::cerr << next_byte;
    }
	}
};