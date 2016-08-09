#include "diff_pi_bot/leddar_one.h"

leddar_one::leddar_one (string port_path, int baud_rate): leddar_s (port_path, baud_rate, serial::Timeout::simpleTimeout(5)) {
	//std::cout << "Arduino port status:";
	leddar_s.setTimeout ( 1, 0, 0, 5, 1 );
	if(leddar_s.isOpen()) {
		//std::cout << " Success, leddar_port " << leddar_port << " open, baud " << leddar_baud << endl;
	}
	else {
		//std::cout << " ERROR: Failed to open leddar serial port " << leddar_port << endl;
		// TODO: end program or exception?
		exit(0);
	}
}

// *****************************************************************************
// Function: CRC16
//
/// \brief   Compute a CRC16 using the Modbus recipe.
///
/// \param   aBuffer  Array containing the data to use.
/// \param   aLength  Number of byte in aBuffer.
/// \param   aCheck   If true, the two bytes after aLength in aBuffer are
///                   supposed to contain the CRC and we verify that it is
///                   the same as what was just computed.
///
/// \return  If aCheck is false, always returns true, if aCheck is true,
///          return true if the CRC compares ok, false otherwise.
// *****************************************************************************
bool leddar_one::CRC16(uint8_t *aBuffer, uint8_t aLength, bool aCheck) {
	uint8_t lCRCHi = 0xFF; // high byte of CRC initialized
	uint8_t lCRCLo = 0xFF; // low byte of CRC initialized
	int i;

	for (i = 0; i<aLength; ++i) {
		int lIndex = lCRCLo ^ aBuffer[i]; // calculate the CRC
		lCRCLo = lCRCHi ^ CRC_HI[lIndex];
		lCRCHi = CRC_LO[lIndex];
	}

	if (aCheck) {
		return ( aBuffer[aLength] == lCRCLo ) && ( aBuffer[aLength+1] == lCRCHi );
	}
	else {
		aBuffer[aLength] = lCRCLo;
		aBuffer[aLength+1] = lCRCHi;
		return true;
	}
}

bool leddar_one::CRC16(std::vector< uint8_t > &aBuffer, uint8_t aLength, bool aCheck) {
	uint8_t lCRCHi = 0xFF; // high byte of CRC initialized
	uint8_t lCRCLo = 0xFF; // low byte of CRC initialized
	int i;

	for (i = 0; i<aLength; ++i) {
		int lIndex = lCRCLo ^ aBuffer[i]; // calculate the CRC
		lCRCLo = lCRCHi ^ CRC_HI[lIndex];
		lCRCHi = CRC_LO[lIndex];
	}

	if (aCheck) {
		return ( aBuffer[aLength] == lCRCLo ) && ( aBuffer[aLength+1] == lCRCHi );
	}
	else {
		aBuffer.push_back( lCRCLo );
		aBuffer.push_back( lCRCHi );
		return true;
	}
}

// Sends a read request fore intensity and distance registers
char leddar_one::request_reading() {
	std::vector <uint8_t> out_buf;
	out_buf.push_back(0x01);
	out_buf.push_back(0x04);
	out_buf.push_back(0x00);
	out_buf.push_back(0x14);
	out_buf.push_back(0x00);
	out_buf.push_back(0x0a);
	CRC16(out_buf, 6, false);

	return leddar_s.write(out_buf);
}

// Receives intensity and distance readings 
int leddar_one::get_reading() {
	unsigned int crc = 0xFFFF;
	//unsigned char dataBuffer[25] = {0};
	unsigned int i = 0;
	size_t len = 0;
	ros::Time startTime;
	unsigned char detcount = 0;
	
	if(leddar_s.available()) {
		startTime = ros::Time::now();
		buffer.clear();
		len = leddar_s.read(buffer, MODBUS_MAX_LEN);


		if (len == 25 && buffer[1] == 0x04 && buffer[0] == 1) {
			// Check CRC
			if (!CRC16(buffer, len-2, true)) {
				return ERR_LEDDAR_BAD_CRC; //invalid CRC
			}
		    
			NbDet = buffer[10];
		    
			if (NbDet > ARRAYSIZE(detections)) {
				
				return ERR_LEDDAR_NB_DETECTIONS;
			}
		    
		    //Timestamp
			TimeStamp = ((unsigned long)buffer[5]) << 24;
			TimeStamp += ((unsigned long)buffer[6]) << 16 ;
			TimeStamp += ((unsigned long)buffer[3])<<8; 
			TimeStamp += buffer[4];

		    // Internal Temperature
			//Temperature = buffer[7];
		 	//Temperature += ((float)buffer[8])/256;
			
			i = 11;
			for (detcount = 0; detcount < NbDet; detcount++) {
				// For each detection:
				// Bytes 0 and 1 = distance in cm
				// Bytes 2-3 are amplitude*256
				//if
				detections[detcount].distance = ((unsigned int)buffer[i])*256 + buffer[i+1];
				detections[detcount].amplitude = ((float)buffer[i+2])+ ((float)buffer[i+3])/256;
	
				i += 4;
			}
			
			return NbDet;
		}
		else 
		return ERR_LEDDAR_BAD_RESPONSE; // Invalid response

	}
	return ERR_LEDDAR_NO_RESPONSE;

}
