#ifndef OROCOS_IMURVC_DRIVER_COMPONENT_HPP
#define OROCOS_IMURVC_DRIVER_COMPONENT_HPP

// Orocos
#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <rtt/extras/FileDescriptorActivity.hpp>
#include <rtt/os/TimeService.hpp>

// ROS
#include <sensor_msgs/typekit/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// Sweetie bot
#include <sweetie_bot_logger/logger.hpp>

// Other
#include <string>

extern "C" {
#include <termios.h>
#include <string.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
}

namespace imu_rvc
{

class IMURVCDriver : public RTT::TaskContext
{
	protected: 
		typedef imu_rvc::IMURVCDriver IMU_RVCDriver;

        enum ReceiverState {
            HEADER1, HEADER2, DATA
        };

        static const unsigned int packet_size = 19;

	protected:
#ifdef SWEETIEBOT_LOGGER
        sweetie_bot::logger::SWEETIEBOT_LOGGER log;
#else
		sweetie_bot::logger::LoggerRTT log;
#endif
		// Port file handler
		int port_fd;
		// Receiver state
        ReceiverState recv_state;
		// Receive buffers

	    // COMPONENT INTERFACE

    protected:
            union Register
            {
                std::int16_t num;

                struct __attribute__ ((packed)) Bytes
                {
                    // The order of these bytes matters
                    std::uint8_t msb;
                    std::uint8_t lsb;
                } bytest;
            };

            struct __attribute__ ((packed)) Packet
            {
                std::uint8_t padding;
                std::uint8_t header1;
                std::uint8_t header2;
                std::uint8_t index;
                Register yaw;
                Register pitch;
                Register roll;
                Register accelx;
                Register accely;
                Register accelz;
                std::uint8_t reserved1;
                std::uint8_t reserved2;
                std::uint8_t reserved3;
                std::uint8_t crc;
            };

            union __attribute__ ((packed)) PacketData
            {
                Packet fields;
                struct Raw {
                  std::uint8_t padding;
                  std::uint8_t buffer[packet_size];
                } raw;
            };

	protected:
		// Properties
        std::string frame_id_prop;
        std::string port_name_prop;
        unsigned int baudrate_prop;
        ulong buffer_size = 0;
        bool first_read = true;
        bool first_packet = true;

        unsigned char packet_index = 0;
        sensor_msgs::Imu imu_data;
        tf2::Quaternion quaternion;
        PacketData packet;

	protected:
		// Operations: required
		RTT::OperationCaller<void(const sensor_msgs::Imu& pkt)> receivePacketDL;
        // PORTS
        RTT::OutputPort<sensor_msgs::Imu> imu_port;

	public:
		IMURVCDriver(std::string const& name);
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
};

}
#endif
