#include "imu_bno080_rvc.hpp"

using namespace sweetie_bot;
using namespace RTT;

namespace imu_rvc
{

//Convinence macro fo logging.
std::ostream& resetfmt(std::ostream& s) {
    s.copyfmt(std::ios(nullptr));
	return s;
}

IMURVCDriver::IMURVCDriver(std::string const& name) : 
    TaskContext(name, PreOperational),
    log(logger::categoryFromComponentName(name)),
    port_fd(-1),
    receivePacketDL("receivePacket")
{
	if (!log.ready()) {
		RTT::Logger::In in("IMURVCDriver");
        RTT::log(RTT::Error) << "Logger is not ready! " << name.c_str() << RTT::endlog();
		this->fatal();
		return;
	}

    // PORTS
    this->addPort("out_imu", imu_port).doc("Publish Imu data.");

	this->requires()->addOperationCaller(receivePacketDL);

    this->addProperty("frame_id", frame_id_prop)
        .doc("Frame id.");
    this->addProperty("port_name", port_name_prop)
        .doc("Serial port device.");
    this->addProperty("baudrate", baudrate_prop)
		.doc("Serial port baudrate.")
		.set(115200);

    this->setActivity(new extras::FileDescriptorActivity(60, nullptr, "IMURVCPortActivity"));
}

bool IMURVCDriver::configureHook()
{

	struct termios tty;

	port_fd = open(this->port_name_prop.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK );
	if (port_fd == -1) {
		log(ERROR) << "open() serial port \"" << this->port_name_prop << "\" failed: " << strerror(errno) << endlog(); 
		return false;
	}
	// configure serial port
	if (tcgetattr (this->port_fd, &tty) != 0) {
		log(ERROR) << "tcgetattr() failed: " << strerror(errno) << endlog(); 
		return false;
	}
	
	// 8-bits, 1 STOP bit, enable receiver, ignore modem lines
	//tty.c_cflag = CS8 | CREAD | CSTOPB | CLOCAL; 
	tty.c_cflag = CS8 | CREAD | CLOCAL; 
	// no signaling chars, no echo, no canonical processing
	tty.c_lflag = 0;
	// no special input processing
	tty.c_iflag = 0;
	// no special output processing
	tty.c_oflag = 0;
	// set speed
	int ret;
	switch (this->baudrate_prop) {
		case 9600:
			ret = cfsetspeed (&tty, B9600);
			break;
		case 19200:
			ret = cfsetspeed (&tty, B19200);
			break;
		case 38400:
			ret = cfsetspeed (&tty, B38400);
			break;
		case 57600:
			ret = cfsetspeed (&tty, B57600);
			break;
		case 115200:
			ret = cfsetspeed (&tty, B115200);
			break;
		case 230400:
			ret = cfsetspeed (&tty, B230400);
			break;
		default:
			log(ERROR) << "Incorrect baudrate property value: " << baudrate_prop << endlog(); 
			return false;
	}
	if (ret) {
		log(ERROR) << "cfsetspeed() failed: " << strerror(errno) << endlog(); 
		return false;
	}
	// special properties
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout, so read will not block forever
	// configure port
	if (tcsetattr (this->port_fd, TCSANOW, &tty) != 0) {
		log(ERROR) << "tcsetattr() failed: " << strerror(errno) << endlog(); 
		return false;
	}
	// set low_latency flag
	struct serial_struct serial;
	if (ioctl(this->port_fd, TIOCGSERIAL, &serial) == -1) {
		log(WARN) << "Unable to get serial_struct. ioctl() failed: " << strerror(errno) << endlog();
	}
	else {
		serial.flags |= ASYNC_LOW_LATENCY;
		if (ioctl(this->port_fd, TIOCSSERIAL, &serial) == -1) {
			log(WARN) << "Unable to set low_latency flag. ioctl() failed: " << strerror(errno) << endlog();
		}
	}
	// Setup FileDescriptorActivity
	extras::FileDescriptorActivity * activity = dynamic_cast<extras::FileDescriptorActivity *>(this->getActivity());
	if (! activity) {
		log(ERROR) << "Incompatible activity type."  << endlog(); 
		return false;
	}
	activity->watch(port_fd);
	
    imu_port.setDataSample(imu_data);

	// reserve memory
	//recv_pkt.data.reserve(packet_size);
	log(INFO) << "IMURVCDriver is configured!" << endlog(); 
	return true;
}

bool IMURVCDriver::startHook()
{
	// flush input buffer
	int retval = tcflush(port_fd, TCIFLUSH);
	if (retval == -1) {
		log(ERROR) << "tcflush() failed:" << strerror(errno) << endlog(); 
		return false;
	}
	// wait for header
	recv_state = HEADER1;
	log(INFO) << "IMURVCDriver is started!" << endlog(); 
	return true;
}

void IMURVCDriver::updateHook()
{
	extras::FileDescriptorActivity * activity = dynamic_cast<extras::FileDescriptorActivity *>(this->getActivity());
	if (! activity) {
		log(ERROR) << "Incompatible activity type."  << endlog(); 
		this->exception();
		return;
	}
	if (activity->hasError()) {
		log(ERROR) << "FileDescriptorActivity error."  << endlog(); 
		this->exception();
		return;
	}

	if (activity->isUpdated(port_fd)) {
		// read new data on port
        ulong buffer_index = 0;

        ssize_t retval = TEMP_FAILURE_RETRY(read(port_fd, packet.raw.buffer+buffer_size, (packet_size - ulong(buffer_size)) ));
		if (retval == 0) return;
		else if (retval == -1) {
			log(ERROR) << "Read serial port failed:" << strerror(errno) << endlog(); 
			this->exception();
		}

        buffer_size += ulong(retval);

        if (log(DEBUG)) {
            log() << "READ on serial port (" << buffer_size << " bytes):" << std::dec << std::setw(2) << std::setfill('0');
            for (ulong i = 0; i < buffer_size; i++) log() << uint32_t( packet.raw.buffer[i] ) << " ";
            log() << resetfmt << endlog();
        }

        if(buffer_size != packet_size){
            if(first_read){
               // we can delete incomplete data at first read
               buffer_size = 0;
               first_read = false;
            }
            return;
        }
        recv_state = HEADER1;


		// parse new data in buffer
        for(buffer_index = 0; buffer_index < buffer_size; buffer_index++) {
            unsigned char c = packet.raw.buffer[buffer_index];

			switch (recv_state) {
				case HEADER1:
                    if (c == 0xAA) recv_state = HEADER2;
                    break;
			   
				case HEADER2:
                    if(c == 0xAA)
                    {
                        recv_state = DATA;
                    }
                    else
                    {
                        if ( packet.raw.buffer[14] == 0x00 and
                             packet.raw.buffer[15] == 0x00 and
                             packet.raw.buffer[16] == 0x00 and
                             packet.raw.buffer[17] != 0x00 and
                             packet.raw.buffer[18] == 0xAA )
                        {
                            log(DEBUG) << "Buffer is not aligned!" << endlog();
                            buffer_size = 1;
                            return;
                        }
                        recv_state = HEADER1;
                    }
					break;

                case DATA:
                    if(buffer_index > 2)
                    {
                        log(WARN) << "Buffer shifted" << endlog();
                        ulong extra_bytest = ulong(buffer_index-2);
                        ulong shift_len = sizeof(packet.raw.buffer) - extra_bytest;
                        memmove(packet.raw.buffer, packet.raw.buffer+extra_bytest, shift_len );
                        // we need more data!
                        buffer_size = shift_len;
                        return;
                    }

                    // we reach the end of loop
                    buffer_size = 0;

                    unsigned char crc = 0;
                    // compute checksum
                    for(unsigned char i=2; i<18; i++)
                        crc +=packet.raw.buffer[i];

                    if (true) {
                       log(DEBUG)
                            << " h=" << uint16_t(packet.fields.header1)
                            << " h=" << uint16_t(packet.fields.header2)
                            << " I=" << uint16_t(packet.fields.index)
                            << " Y=" << (packet.fields.yaw.num / 100.0) / (180.0 / M_PI)
                            << " P=" << (packet.fields.pitch.num / 100.0) / (180.0 / M_PI)
                            << " R=" << (packet.fields.roll.num / 100.0) / (180.0 / M_PI)
                            /*<< " X=" << format("{:#06x}", packet.fields.accelx.num)
                            << " Y=" << format("{:#06x}", packet.fields.accely.num)
                            << " Z=" << format("{:#06x}", packet.fields.accelz.num)*/
                            << " X=" << (packet.fields.accelx.num / 100.0) * 9.8
                            << " Y=" << (packet.fields.accely.num / 100.0) * 9.8
                            << " Z=" << (packet.fields.accelz.num / 100.0) * 9.8
                            << " R=" << uint32_t(packet.fields.reserved1 + packet.fields.reserved2 + packet.fields.reserved3)
                            << " C=" << uint16_t(packet.fields.crc)
                            << " CRC=" << int(crc)
                            << endlog();
                    }

                    if(crc == packet.fields.crc){
                        if(first_read){
                          first_read = false;
                          packet_index = packet.fields.index;
                        }
                    }
                    else
                    {
                        log(ERROR) << "CRC did not match!" << endlog();
                        return;
                    }

                    if( packet.fields.reserved1 + packet.fields.reserved2 +packet.fields.reserved3 != 0)
                    {
                        log(ERROR) << "Reserved bytes are not zeros! " << endlog();
                        return;
                    }

                    if(!first_packet)
                    {
                        if((uint8_t(packet_index+1) != packet.fields.index))
                        {
                            log(WARN) << "index did not match! "
                                       << uint8_t(packet_index+1) << " != " <<
                                          uint8_t(packet.fields.index) << endlog();
                        }

                    }else {
                        first_packet = false;
                    }

                    packet_index = packet.fields.index;

                    // convert Euler to Quaternion
                    quaternion.setRPY((packet.fields.roll.num / 100.0) / (180.0 / M_PI),
                                      (packet.fields.pitch.num / 100.0) / (180.0 / M_PI),
                                      (packet.fields.yaw.num / 100.0) / (180.0 / M_PI));

                    // convert tf2 Quaternion to geometry_msgs::Quaternion
                    tf2::convert(quaternion, imu_data.orientation);

                    // convert
                    imu_data.linear_acceleration.x = (packet.fields.accelx.num / 100.0) * 9.8;
                    imu_data.linear_acceleration.y = (packet.fields.accely.num / 100.0) * 9.8;
                    imu_data.linear_acceleration.z = (packet.fields.accelz.num / 100.0) * 9.8;

                    imu_data.header.stamp = ros::Time::now();
                    imu_data.header.frame_id = frame_id_prop;
                    imu_port.write(imu_data);


                    return; // goto next packet
			}
		}
        buffer_size = 0; // if header is not found
	} // if (activity->isUpdated(port_fd))
}

void IMURVCDriver::stopHook() 
{
	log(INFO) << "IMURVCDriver is stopped!" << endlog(); 
}

void IMURVCDriver::cleanupHook() {
	extras::FileDescriptorActivity * activity = dynamic_cast<extras::FileDescriptorActivity *>(this->getActivity());
	if (! activity) {
		log(ERROR) << "Incompatible activity type."  << endlog(); 
	}
	activity->unwatch(port_fd);
	if (TEMP_FAILURE_RETRY(close(port_fd))) {
		log(ERROR) << "close() serial port failed: " << strerror(errno) << endlog(); 
	}
	log(INFO) << "IMURVCDriver is cleaned up!" << endlog(); 
}

}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(IMURVCDriver)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(imu_rvc::IMURVCDriver)
