#include <fstream>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <string>
#include <SerialStream.h>

using namespace LibSerial;
using namespace std;

class SerialCom {
 public:

 	void Send(string data) {
 		data.append("\n");
		char *out_buf = (char*)data.c_str();
    serial_port.write(out_buf, data.length());
    Listen();
 	}

 	void Open(const std::string &addr) {
 		serial_port.Open(addr) ;
    if ( ! serial_port.good() ) {
      std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                   << "Error: Could not open serial port."
                   << std::endl ;
      exit(1) ;
    }
    // Set the number of data bits.
    serial_port.SetCharSize( SerialStreamBuf::CHAR_SIZE_8 ) ;
    if ( ! serial_port.good() ) {
      std::cerr << "Error: Could not set the character size." << std::endl ;
      exit(1) ;
    }
    // Disable parity.
    serial_port.SetParity( SerialStreamBuf::PARITY_NONE ) ;
    if ( ! serial_port.good() ) {
      std::cerr << "Error: Could not disable the parity." << std::endl ;
      exit(1) ;
    }
    // Set the number of stop bits.
    serial_port.SetNumOfStopBits( 1 ) ;
    if ( ! serial_port.good() ) {
    	std::cerr << "Error: Could not set the number of stop bits." << std::endl ;
    	exit(1) ;
    }
    // Turn off hardware flow control.
    serial_port.SetFlowControl( SerialStreamBuf::FLOW_CONTROL_NONE ) ;
    if ( ! serial_port.good() ) {
      std::cerr << "Error: Could not use hardware flow control." << std::endl ;
      exit(1) ;
    }
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
 	SerialStream serial_port;
 	string address;

 	void Listen() {
    while(  serial_port.rdbuf()->in_avail() > 0  ){
      char next_byte;
      serial_port.get(next_byte);
      std::cerr << next_byte;
    }
	}
};