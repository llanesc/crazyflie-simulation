/*
  * Author: Christian Llanes, Georgia Institute of Technology, USA
  */

#include "crazysim_plugin.h"
#include <future>

#include <gz/plugin/Register.hh>
GZ_ADD_PLUGIN(crazyflie_interface::GzCrazyflieInterface,
			  gz::sim::System,
			  crazyflie_interface::GzCrazyflieInterface::ISystemConfigure,
			  crazyflie_interface::GzCrazyflieInterface::ISystemPreUpdate,
			  crazyflie_interface::GzCrazyflieInterface::ISystemPostUpdate)
using namespace crazyflie_interface;

GzCrazyflieInterface::GzCrazyflieInterface() :
	namespace_(kDefaultNamespace),
	motor_velocity_reference_pub_topic_(kDefaultMotorVelocityReferencePubTopic),
	imu_sub_topic_(kDefaultImuTopic),
	magnetic_field_sub_topic_(kDefaultMagneticFieldTopic),
	barometer_sub_topic_(kDefaultFluidPressureTopic),
	odom_sub_topic_(kDefaultOdomTopic),
	cf_prefix(kDefaultCfPrefix),
	isPluginOn(true){
}

GzCrazyflieInterface::~GzCrazyflieInterface() {
	isPluginOn = false;
	cfLibAddrInitialized_ = false;
	socketInit = false;
	isInit = false;
	
	senderCfFirmwareThread.join();
	senderCfLibThread.join();
	receiverCfFirmwareThread.join();
	receiverCfLibThread.join();
}

void GzCrazyflieInterface::Configure(const gz::sim::Entity &_entity,
						const std::shared_ptr<const sdf::Element> &_sdf,
						gz::sim::EntityComponentManager &_ecm,
						gz::sim::EventManager &_eventMgr) {
	gzdbg << __FUNCTION__ << "() called." << std::endl;
		
  	auto sdfClone = _sdf->Clone();

	/* Get Crazylfie SDF parameters */
	cffirm_addr = sdfClone->Get<std::string>("cffirm_addr");
	cffirm_port = sdfClone->Get<std::string>("cffirm_port");
	cflib_addr = sdfClone->Get<std::string>("cflib_addr");
	cflib_port = sdfClone->Get<std::string>("cflib_port");
	cf_prefix = sdfClone->Get<std::string>("cfPrefix");
	imu_sub_topic_ = sdfClone->Get<std::string>("imuSubTopic");
	magnetic_field_sub_topic_ = sdfClone->Get<std::string>("magSubTopic");
	barometer_sub_topic_ = sdfClone->Get<std::string>("baroSubTopic");
	cf_id_ = sdfClone->Get<int>("cfId");

	// ---------- Setup socket for Crazyflie firmware -----------
	std::string dest_addr_cfFirm = cffirm_addr;
	std::string dest_port_cfFirm = cffirm_port;
	// open socket in UDP mode
	if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		gzerr << "create CfFirm socket failed " << std::endl;
		return;
	}
	memset((char *)&myaddr, 0, sizeof(myaddr));
	myaddr.sin_family = AF_INET;
	if (dest_addr_cfFirm == "INADDR_ANY")
		myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	else if (inet_addr(dest_addr_cfFirm.c_str()) == INADDR_NONE){
		gzerr << "Adress IP CfFirm invalid !" << std::endl;
		return;
	}else
		myaddr.sin_addr.s_addr = inet_addr(dest_addr_cfFirm.c_str());
	port = std::stoi(dest_port_cfFirm);
	myaddr.sin_port = htons(port);
	// Bind the socket
	if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0){
		gzerr << "Bind socket CfFirm failed " << std::endl;
		return;
	}
	addrlen_rcv = sizeof(remaddr_rcv);
	

	// ---------- Setup socket for Crazyflie Python api -----------
	std::string dest_addr_cfLib = cflib_addr;
	std::string dest_port_cfLib = cflib_port;
	// open CfLib socket in UDP mode
	if ((fd_cfLib = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		gzerr << "create CfLib socket failed " << std::endl;
		return;
	}
	memset((char *)&myaddr_CfLib, 0, sizeof(myaddr_CfLib));
	myaddr_CfLib.sin_family = AF_INET;
	if (dest_addr_cfLib == "INADDR_ANY")
		myaddr_CfLib.sin_addr.s_addr = htonl(INADDR_ANY);
	else if (inet_addr(dest_addr_cfLib.c_str()) == INADDR_NONE){
		gzerr << "Adress IP CfLib invalid !" << std::endl;
		return;
	}else
		myaddr_CfLib.sin_addr.s_addr = inet_addr(dest_addr_cfLib.c_str());
	port_cfLib = std::stoi(dest_port_cfLib);
	myaddr_CfLib.sin_port = htons(port_cfLib);
	// Bind the socket
	if (bind(fd_cfLib, (struct sockaddr *)&myaddr_CfLib, sizeof(myaddr_CfLib)) < 0){
		gzerr << "Bind socket CfLib failed " << std::endl;
		return;
	}
	addrlen_rcv_cfLib = sizeof(remaddr_rcv_cfLib);

	imu_queue = moodycamel::BlockingConcurrentQueue<crtpPacket_t>(20);
	barometer_queue = moodycamel::BlockingConcurrentQueue<crtpPacket_t>(20);
	odom_queue = moodycamel::BlockingConcurrentQueue<crtpPacket_t>(20);
	cflib_to_firmware_queue = moodycamel::BlockingConcurrentQueue<crtpPacket_t>(20);
	firmware_to_cflib_queue = moodycamel::BlockingConcurrentQueue<crtpPacket_t>(20);

	addrlen = sizeof(remaddr);
	addrlen_cfLib = sizeof(remaddr_cfLib);
	isInit = false;
	socketInit = false;
	cfLibAddrInitialized_ = false;
	m_motor_command_.m1 = 0;
	m_motor_command_.m2 = 0;
	m_motor_command_.m3 = 0;
	m_motor_command_.m4 = 0;

	receiverCfFirmwareThread = std::thread(&GzCrazyflieInterface::recvCfFirmwareThread, this);
	receiverCfLibThread = std::thread(&GzCrazyflieInterface::recvCfLibThread, this);
	senderCfFirmwareThread = std::thread(&GzCrazyflieInterface::sendCfFirmwareThread, this);
	senderCfLibThread = std::thread(&GzCrazyflieInterface::sendCfLibThread, this);
	initializeSubsAndPub();
}

void GzCrazyflieInterface::PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) {
	writeMotors();
}

void GzCrazyflieInterface::PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm) {

}

void GzCrazyflieInterface::ImuCallback(const gz::msgs::IMU& imu_msg) {
	struct imu_s m_imu_info = {
		.header = crtp(CRTP_PORT_SETPOINT_SIM, 0),
		.type = SENSOR_GYRO_ACC_SIM,
		.acc = { static_cast<int16_t>(imu_msg.linear_acceleration().x()  	/ 	SENSORS_G_PER_LSB_CFG 		/ GRAVITY_MAGNITUDE_CF),
				 static_cast<int16_t>(imu_msg.linear_acceleration().y()  	/ 	SENSORS_G_PER_LSB_CFG 		/ GRAVITY_MAGNITUDE_CF),
				 static_cast<int16_t>(imu_msg.linear_acceleration().z()  	/ 	SENSORS_G_PER_LSB_CFG 		/ GRAVITY_MAGNITUDE_CF)},
		.gyro = {static_cast<int16_t>(imu_msg.angular_velocity().x()	   	/ 	SENSORS_DEG_PER_LSB_CFG	 	/ DEG_TO_RAD_CF),
				 static_cast<int16_t>(imu_msg.angular_velocity().y()    	/ 	SENSORS_DEG_PER_LSB_CFG 	/ DEG_TO_RAD_CF),
				 static_cast<int16_t>(imu_msg.angular_velocity().z()    	/	SENSORS_DEG_PER_LSB_CFG		/ DEG_TO_RAD_CF)}};

	crtpPacket_t msg;
	msg.size = sizeof(m_imu_info);
	memcpy(msg.raw, (const uint8_t*) &m_imu_info , sizeof(m_imu_info));
	imu_queue.enqueue(msg);
}

void GzCrazyflieInterface::BarometerCallback(const gz::msgs::FluidPressure& air_pressure_msg) {
	double temperature_at_altitude_kelvin = kSeaLevelTempKelvin * exp(- log(air_pressure_msg.pressure() / kPressureOneAtmospherePascals)/kAirConstantDimensionless);
	double height_geopotential_m = (kSeaLevelTempKelvin - temperature_at_altitude_kelvin)/kTempLapseKelvinPerMeter;
	double height_geometric_m = height_geopotential_m * kEarthRadiusMeters / (kEarthRadiusMeters - height_geopotential_m);
	
	struct baro_s m_baro_info = {
		.header = crtp(CRTP_PORT_SETPOINT_SIM, 0),
		.type = SENSOR_BARO_SIM,
		.pressure = static_cast<float>(air_pressure_msg.pressure() * 0.01), // Convert in mbar
		.temperature = static_cast<float>(temperature_at_altitude_kelvin - 273.15),
		.asl = static_cast<float>(height_geometric_m)
	};

	crtpPacket_t msg;
	msg.size = sizeof(m_baro_info);
	memcpy(msg.raw, (const uint8_t*) &m_baro_info , sizeof(m_baro_info));
	barometer_queue.enqueue(msg);
}

void GzCrazyflieInterface::OdomCallback(const gz::msgs::Odometry& odom_msg) {
	// Just need to transfer to the fcu the external position
	struct CrtpExtPose_s ext_pose  = {
		.header = crtp(CRTP_PORT_LOCALIZATION, CRTP_LOC_CHANNEL_GEN_LOC),
		.id = CRTP_GEN_LOC_ID_EXT_POS,
		.x = static_cast<float>(odom_msg.pose().position().x()),
		.y = static_cast<float>(odom_msg.pose().position().y()), 
		.z = static_cast<float>(odom_msg.pose().position().z()),
		.qx = static_cast<float>(odom_msg.pose().orientation().x()),
		.qy = static_cast<float>(odom_msg.pose().orientation().y()),
		.qz = static_cast<float>(odom_msg.pose().orientation().z()),
		.qw = static_cast<float>(odom_msg.pose().orientation().w())
	}; 
	
	crtpPacket_t msg;
	msg.size = sizeof(ext_pose);
	memcpy(msg.raw, (const uint8_t*) &ext_pose , sizeof(ext_pose));
	odom_queue.enqueue(msg);
}

bool GzCrazyflieInterface::sendCfFirmware(const uint8_t* data , uint32_t length) {	
	int transferred = sendto(fd, data, length, 0, (struct sockaddr *) &(remaddr), addrlen);
	if (transferred <= 0)
		throw std::runtime_error(strerror(errno));
    return true;
}

bool GzCrazyflieInterface::sendCfLib(const uint8_t* data , uint32_t length) {	
	int transferred = sendto(fd_cfLib, data, length, 0, (struct sockaddr *) &(remaddr_cfLib), addrlen_cfLib);
	if (transferred <= 0)
		throw std::runtime_error(strerror(errno));
    return true;
}

void GzCrazyflieInterface::recvCfFirmwareThread() {
	uint8_t buf[32];
	while(isPluginOn)
	{
		int len = recvfrom(fd, buf, sizeof(buf), 0, (struct sockaddr *) &remaddr_rcv, &addrlen_rcv);
		if (len <= 0 )
			continue;
		if (!socketInit && buf[0] == 0xF3 && len == 1){
			gzmsg << "Received firmware handshake message..." << std::endl;
			remaddr = remaddr_rcv;
			addrlen = addrlen_rcv;
			socketInit = true;
			// Send response to SITL instance
			uint8_t data[1] = {0xF3};
			sendCfFirmware(data , sizeof(data));
		}
		else if (socketInit && (crtp(buf[0]) == crtp(0x09,0)) && (len == 9)) // Motor command message
			handleMotorsMessage(&buf[0]);
		else if (socketInit && cfLibAddrInitialized_){
			crtpPacket_t packet;
			packet.size = len - 1;
			memcpy(&packet.raw[0] , &buf[0], sizeof(uint8_t) * len);
			bool succeeded  = firmware_to_cflib_queue.enqueue(packet);
			if(!succeeded)
				gzmsg << "Failed to enqueue firmware_to_cflib_queue in recvCfFirmwareThread()." << std::endl;
		}
	}
}

void GzCrazyflieInterface::recvCfLibThread() {
	uint8_t buf[32];
	while(isPluginOn)
	{
		if (!socketInit)
			continue;
		int len = recvfrom(fd_cfLib, buf, sizeof(buf), 0, (struct sockaddr *) &remaddr_rcv_cfLib, &addrlen_rcv_cfLib);
		if (len <= 0 )
			continue;

		// Update cflib address on every received packet (activity-based detection)
		remaddr_cfLib = remaddr_rcv_cfLib;
		addrlen_cfLib = addrlen_rcv_cfLib;
		cfLibAddrInitialized_ = true;

		// Echo back null CRTP packets (0xFF) for scan/ping
		if (buf[0] == 0xFF && len == 1) {
			uint8_t echo[1] = {0xFF};
			sendCfLib(echo, sizeof(echo));
		}

		// Forward all packets to firmware
		recvCfLib(&buf[0], len);
	}
}

void GzCrazyflieInterface::sendCfFirmwareThread() {
	crtpPacket_t msgs[10];
	std::pair<std::chrono::steady_clock::duration, crtpPacket_t> stamp_msg_pair;
	while(isPluginOn){
		if(!socketInit)
			continue;

		size_t imu_msg_count = imu_queue.try_dequeue_bulk(msgs, 10);
		for (size_t i = 0; i != imu_msg_count; i++) {
			sendCfFirmware(msgs[i].raw, msgs[i].size+1);
		}

		size_t baro_msg_count = barometer_queue.try_dequeue_bulk(msgs, 10);
		for (size_t i = 0; i != baro_msg_count; i++) {
			sendCfFirmware(msgs[i].raw, msgs[i].size+1);
		}

		size_t odom_msg_count = odom_queue.try_dequeue_bulk(msgs, 10);
		for (size_t i = 0; i != odom_msg_count; i++) {
			sendCfFirmware(msgs[i].raw, msgs[i].size+1);
		}

		size_t cflib_msg_count = cflib_to_firmware_queue.try_dequeue_bulk(msgs, 10);
		for (size_t i = 0; i != cflib_msg_count; i++) {
			sendCfFirmware(msgs[i].raw, msgs[i].size+1);
		}
	}
}

void GzCrazyflieInterface::sendCfLibThread() {
	crtpPacket_t msgs[10];
	while(isPluginOn){
		if(!socketInit || !cfLibAddrInitialized_)
			continue;

		size_t cflib_msg_count = firmware_to_cflib_queue.wait_dequeue_bulk(msgs, 10);
		for (size_t i = 0; i != cflib_msg_count; i++) {
			sendCfLib(msgs[i].raw, msgs[i].size+1);
		}
	}
} 


void GzCrazyflieInterface::initializeSubsAndPub() {
	node_.Subscribe(cf_prefix + "_" + std::to_string(cf_id_) + imu_sub_topic_, &GzCrazyflieInterface::ImuCallback , this);
	node_.Subscribe(cf_prefix + "_" + std::to_string(cf_id_) + barometer_sub_topic_, &GzCrazyflieInterface::BarometerCallback, this );
	node_.Subscribe(cf_prefix + "_" + std::to_string(cf_id_) + odom_sub_topic_, &GzCrazyflieInterface::OdomCallback, this );
	
	motor_velocity_reference_pub_ = node_.Advertise<gz::msgs::Actuators>(cf_prefix + "_" + std::to_string(cf_id_) + motor_velocity_reference_pub_topic_);
	
	isInit = true;
	gzmsg << "Init subs and Pubs done : " << std::endl;
}

void GzCrazyflieInterface::handleMotorsMessage(const uint8_t* data) {
	crtpMotorsDataResponse* motorsData = (crtpMotorsDataResponse *) data;
	{
		std::unique_lock<std::mutex> mlock(motors_mutex);
		m_motor_command_.m1 = PWM2OMEGA(motorsData->m1);
		m_motor_command_.m2 = PWM2OMEGA(motorsData->m2);
		m_motor_command_.m3 = PWM2OMEGA(motorsData->m3);
		m_motor_command_.m4 = PWM2OMEGA(motorsData->m4);
	}

}

void GzCrazyflieInterface::recvCfLib(uint8_t* data , int len) {
	crtpPacket_t packet;
	packet.size = len - 1;
	memcpy(&packet.raw[0] , data , sizeof(uint8_t) * len);
	bool succeeded  = cflib_to_firmware_queue.enqueue(packet);
	if(!succeeded)
		gzmsg << "Failed recvCfLib" << std::endl;
}


void GzCrazyflieInterface::writeMotors() {
	m_motor_speed.clear_velocity();
	motors_mutex.lock();
	m_motor_speed.add_velocity(m_motor_command_.m1); // 0
	m_motor_speed.add_velocity(m_motor_command_.m2); // 1
	m_motor_speed.add_velocity(m_motor_command_.m3); // 2
	m_motor_speed.add_velocity(m_motor_command_.m4); // 3
	motors_mutex.unlock();
	motor_velocity_reference_pub_.Publish(m_motor_speed);
}