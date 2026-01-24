/*
  * Author: Christian Llanes, Georgia Institute of Technology, USA
  */
#ifndef CRAZYFLIE_INTERFACE_HH_
#define CRAZYFLIE_INTERFACE_HH_

#include <iostream>
#include <mutex>
#include <atomic>
#include <math.h>
#include <deque>
#include <stdio.h>
#include <queue>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/math/Vector3.hh>

#include <gz/transport.hh>

#include <gz/msgs/imu.pb.h>
#include <gz/msgs/fluid_pressure.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/actuators.pb.h>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>

/* Fast library for a reader writer queue */
#include "blockingconcurrentqueue.h"
#include "CrtpUtils.h"

// Default topic names
static const std::string kDefaultMotorVelocityReferencePubTopic = "/command/motor_speed";
static const std::string kDefaultImuTopic = "/imu";
static const std::string kDefaultNamespace = "";
static const std::string kDefaultMagneticFieldTopic = "/mag";
static const std::string kDefaultFluidPressureTopic = "/baro";
static const std::string kDefaultOdomTopic = "/odom";
static const std::string kDefaultCfPrefix = "cf";
// Constants
static constexpr double kGasConstantNmPerKmolKelvin = 8314.32;
static constexpr double kMeanMolecularAirWeightKgPerKmol = 28.9644;
static constexpr double kGravityMagnitude = 9.80665;
static constexpr double kEarthRadiusMeters = 6356766.0;
static constexpr double kPressureOneAtmospherePascals = 101325.0;
static constexpr double kSeaLevelTempKelvin = 288.15;
static constexpr double kTempLapseKelvinPerMeter = 0.0065;
static constexpr double kAirConstantDimensionless = kGravityMagnitude *
    kMeanMolecularAirWeightKgPerKmol /
        (kGasConstantNmPerKmolKelvin * -kTempLapseKelvinPerMeter);
typedef struct _SensorsData {
	uint8_t data[sizeof(struct imu_s)];
} SensorsData;

namespace crazyflie_interface
{
	class GzCrazyflieInterface : 
		public gz::sim::System, 
		public gz::sim::ISystemConfigure,
		public gz::sim::ISystemPreUpdate,
		public gz::sim::ISystemPostUpdate
	{
		public: GzCrazyflieInterface();
		public:	~GzCrazyflieInterface() override;

		public:	void Configure(const gz::sim::Entity &_entity,
								const std::shared_ptr<const sdf::Element> &_sdf,
								gz::sim::EntityComponentManager &_ecm,
								gz::sim::EventManager &_eventMgr) override;
		public: void PreUpdate(const gz::sim::UpdateInfo &_info,
							    gz::sim::EntityComponentManager &_ecm) override;
		public: void PostUpdate(const gz::sim::UpdateInfo &_info,
								const gz::sim::EntityComponentManager &_ecm) override;

		private:
			int cf_id_;
			std::string namespace_;
			std::string motor_velocity_reference_pub_topic_;
			std::string imu_sub_topic_;
			std::string magnetic_field_sub_topic_;
			std::string barometer_sub_topic_;
			std::string odom_sub_topic_;
			std::string cffirm_addr;
			std::string cffirm_port;
			std::string cflib_addr;
			std::string cflib_port;
			// int cflib_latency_ms; // simulated radio delay


			// bool enable_logging;
			// bool enable_logging_imu;
			// bool enable_logging_magnetic_field;
			// bool enable_logging_temperature;
			// bool enable_parameters;
			// bool enable_logging_pressure;
			// bool enable_logging_battery;
			// bool enable_logging_packets;
			std::string cf_prefix;


			void ImuCallback(const gz::msgs::IMU& imu_msg);
			void BarometerCallback(const gz::msgs::FluidPressure& air_pressure_msg);
			void OdomCallback(const gz::msgs::Odometry& odom_msg);

			// send and recv functions
			bool sendCfFirmware(const uint8_t* data, uint32_t length);
			bool sendCfLib(const uint8_t* data, uint32_t length);
			void recvCfLib(uint8_t* data , int length);
			
			// Threads main function
			void handleMotorsMessage(const uint8_t* data);

			// server port and remote address for the crazyflies
			int port;
			int port_cfLib;
			int fd;
			int fd_cfLib;
			struct sockaddr_in myaddr;
			struct sockaddr_in myaddr_CfLib;
			struct sockaddr_in remaddr_rcv;
			struct sockaddr_in remaddr_rcv_cfLib;
			socklen_t addrlen_rcv;
			socklen_t addrlen_rcv_cfLib;
			struct sockaddr_in remaddr;
			socklen_t addrlen;
			bool socketInit;
			struct sockaddr_in remaddr_cfLib;
			socklen_t addrlen_cfLib;
			std::atomic<bool> cfLibAddrInitialized_;

			bool isInit;
			bool isPluginOn;

			// Queue for exchanging messages between recv and sender threads
			moodycamel::BlockingConcurrentQueue<crtpPacket_t> m_queueSendCfFirm;
			moodycamel::BlockingConcurrentQueue<crtpPacket_t> imu_queue;
			moodycamel::BlockingConcurrentQueue<crtpPacket_t> barometer_queue;
			moodycamel::BlockingConcurrentQueue<crtpPacket_t> odom_queue;
			moodycamel::BlockingConcurrentQueue<crtpPacket_t> cflib_to_firmware_queue;
			moodycamel::BlockingConcurrentQueue<crtpPacket_t> firmware_to_cflib_queue;

			// mutex and messages for motors command
			gz::msgs::Actuators m_motor_speed;
			std::mutex motors_mutex;
			gz::transport::Node node_;
			gz::transport::Node::Publisher motor_velocity_reference_pub_;

			struct MotorsCommand {
				float m1;
				float m2;
				float m3;
				float m4;
			} m_motor_command_;

			// Initialize publishers and subscribers
		    void initializeSubsAndPub();

			// recv thread
			std::thread receiverCfFirmwareThread;
			void recvCfFirmwareThread();

			// Crazyflie library receiver thread
			std::thread receiverCfLibThread;
			void recvCfLibThread();

			// Sensors data sender thread
			std::thread senderCfFirmwareThread;
			void sendCfFirmwareThread();

			// Crazyflie library sender thread
			std::thread senderCfLibThread;
			void sendCfLibThread();

			// Motor command publisher
			void writeMotors();

	};
}

#endif