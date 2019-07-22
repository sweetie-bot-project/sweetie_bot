#include <map>
#include <memory>
#include <algorithm>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <sweetie_bot_joystick/KeyPressed.h>


namespace JoyDecoder {

class JoyDecoderBase
{
	protected:
		ros::Subscriber joy_sub_;
		ros::Subscriber set_enable_sub_;
		bool is_enabled_;

	protected:
		virtual void decodeHook(const sensor_msgs::Joy& msg) = 0;
		virtual void enableHook() {}

		void callbackSetEnable(const std_msgs::Bool::ConstPtr& msg);
		void callbackJoyMsg(const sensor_msgs::Joy::ConstPtr& msg);

	public:
		JoyDecoderBase(ros::NodeHandle& nh, const YAML::Node& node);
		virtual ~JoyDecoderBase() = default;

		bool isEnabled() { return is_enabled_; }
		void decode(const sensor_msgs::Joy& msg) { if (is_enabled_) decodeHook(msg); }
		void setEnabled(bool enabled);
};

JoyDecoderBase::JoyDecoderBase(ros::NodeHandle& nh, const YAML::Node& node)
{
	set_enable_sub_ = nh.subscribe(node["enable_group"].as<std::string>(), 1, &JoyDecoderBase::callbackSetEnable, this);
	is_enabled_ = false;
	joy_sub_ = nh.subscribe(node["joy_topic"].as<std::string>(), 1, &JoyDecoderBase::callbackJoyMsg, this);
}

void JoyDecoderBase::setEnabled(bool enabled) 
{ 
	if (is_enabled_ == false && enabled == true) enableHook();
	is_enabled_ = enabled; 
}

void JoyDecoderBase::callbackSetEnable(const std_msgs::Bool::ConstPtr& msg)
{
	is_enabled_ = msg->data;
}

void JoyDecoderBase::callbackJoyMsg(const sensor_msgs::Joy::ConstPtr& msg)
{
	if (is_enabled_) decodeHook(*msg);
}

class KeyPressed : public JoyDecoderBase
{
	public:
		typedef std::map<int, int> KeyMapping; // from key/axis code to index
		typedef std::vector<std::string> KeyNames;
		typedef std::vector<bool> PressedKeySet;

	protected:
		KeyMapping key_map_, paxis_map_, naxis_map_;
		KeyNames key_names_;
		PressedKeySet key_set_;
		ros::Publisher pub_;

	protected:
		void addKey(KeyMapping& key_map, int key, const std::string& key_value);
		void processKey(KeyMapping& key_map, int key, sweetie_bot_joystick::KeyPressed& key_msg);
		void decodeHook(const sensor_msgs::Joy& msg) override;

	public:
		KeyPressed(ros::NodeHandle& nh, const YAML::Node& node);
};

void KeyPressed::addKey(KeyMapping& key_map, int key, const std::string& key_value) {
	int key_index;
	// find index in PressedKeySet if it exists
	auto key_name_it = std::find(key_names_.begin(), key_names_.end(), key_value);
	if (key_name_it == key_names_.end()) {
		key_index = key_names_.size();	
		key_names_.push_back(key_value);
		key_set_.push_back(false);
	}
	else key_index = key_name_it - key_names_.begin();
	// add KeyInfo	
	bool success;
	std::tie(std::ignore, success) = key_map.insert( std::make_pair(key,  key_index) );
	if (!success) throw YAML::Exception(YAML::Mark::null_mark(), "Dublicate key");
}


KeyPressed::KeyPressed(ros::NodeHandle& nh, const YAML::Node& node) :
		JoyDecoderBase(nh, node)
{
	// extract key_map and fill key_set
	YAML::Node map_node = node["buttons"];
	for(YAML::iterator it = map_node.begin(); it != map_node.end(); it++) {
		addKey(key_map_, it->first.as<int>(), it->second.as<std::string>());
	}
	map_node = node["axes_positive"];
	for(YAML::iterator it = map_node.begin(); it != map_node.end(); it++) {
		addKey(paxis_map_, it->first.as<int>(), it->second.as<std::string>());
	}
	map_node = node["axes_negative"];
	for(YAML::iterator it = map_node.begin(); it != map_node.end(); it++) {
		addKey(naxis_map_, it->first.as<int>(), it->second.as<std::string>());
	}
	// get topic name
	std::string topic = node["key_topic"].as<std::string>();
	// create publisher
	pub_ = nh.advertise< sweetie_bot_joystick::KeyPressed >(topic, 1);
}

void KeyPressed::processKey(KeyMapping& key_map, int key, sweetie_bot_joystick::KeyPressed& key_msg) 
{
	auto it = key_map.find(key);
	if (it != key_map.end() && !key_set_[it->second]) {
		key_set_[it->second] = true;
		key_msg.keys.push_back(key_names_[it->second]);
	}
}

void KeyPressed::decodeHook(const sensor_msgs::Joy& joy_msg) {
	sweetie_bot_joystick::KeyPressed key_msg;
	key_msg.header = joy_msg.header;
	//  clear pressed set
	//  TODO: event press/unpress detection, minimize message number
	key_set_.assign(key_set_.size(), false);
	// process keys
	for(int k = 0; k < joy_msg.buttons.size(); k++) {
		if (joy_msg.buttons[k]) processKey(key_map_, k, key_msg);
	}
	// process axes
	for(int k = 0; k < joy_msg.axes.size(); k++) {
		if (joy_msg.axes[k] < 0.0)  processKey(naxis_map_, k, key_msg);
		if (joy_msg.axes[k] > 0.0)  processKey(paxis_map_, k, key_msg);
	}
	// publish result
	pub_.publish(key_msg);
}


class JointAxes: public JoyDecoderBase
{
	public:
		struct KeyInfo {
			int index;
			double speed;
			double min;
			double max;
		};

		typedef std::map<int, KeyInfo> KeyMapping;

	protected:
		KeyMapping key_map_;
		KeyMapping axis_map_;
		sensor_msgs::JointState joints_;
		ros::Publisher pub_;

	protected:
		void addKey(KeyMapping& key_map, int key, const YAML::Node& node);
		void decodeKey(KeyMapping& key_map, int key, double value, double dt);

		void decodeHook(const sensor_msgs::Joy& msg) override;
		void enableHook() override;
	public:
		JointAxes(ros::NodeHandle& nh, const YAML::Node& node);
};


void JointAxes::addKey(KeyMapping& key_map, int key, const YAML::Node& node) {
	std::string joint_name = node["joint"].as<std::string>();
	int joint_index;
	// find index in PressedKeySet if it exists
	auto joint_it = std::find(joints_.name.begin(), joints_.name.end(), joint_name );
	if (joint_it == joints_.name.end()) {
		joint_index = joints_.name.size();	
		joints_.name.push_back(joint_name);
		joints_.position.push_back(0.0);
	}
	else joint_index = joint_it - joints_.name.begin();	
	// add KeyInfo	
	KeyInfo key_info;
	key_info.index = joint_index;
	key_info.speed = node["speed"].as<double>();
	key_info.min = node["min"].as<double>();
	key_info.max = node["max"].as<double>();
	bool success;
	std::tie(std::ignore, success) = key_map.insert( std::make_pair( key, key_info ) );
	if (!success) throw YAML::Exception(YAML::Mark::null_mark(), "Dublicate key");
	// set position to default value
	joints_.position[joint_index] = key_info.speed ? 0.5*(key_info.min + key_info.max) : key_info.min;
}

JointAxes::JointAxes(ros::NodeHandle& nh, const YAML::Node& node) :
		JoyDecoderBase(nh, node)
{
	// extract key_map and fill key_set
	YAML::Node map_node = node["buttons"];
	for(YAML::iterator it = map_node.begin(); it != map_node.end(); it++) {
		addKey(key_map_, it->first.as<int>(), it->second);
	}
	map_node = node["axes"];
	for(YAML::iterator it = map_node.begin(); it != map_node.end(); it++) {
		addKey(axis_map_, it->first.as<int>(), it->second); // axes are added as negative integers
	}
	// reset timestamp
	joints_.header.stamp = ros::Time();
	// get topic name
	std::string topic = node["joints_topic"].as<std::string>();
	// create publisher
	pub_ = nh.advertise< sensor_msgs::JointState >(topic, 1);
}

void JointAxes::enableHook() 
{
	// reset timestamp
	joints_.header.stamp = ros::Time();
	// reset position
	for(auto& p : key_map_) joints_.position[p.second.index] = p.second.speed ? 0.5*(p.second.min + p.second.max) : p.second.min;
	for(auto& p : axis_map_) joints_.position[p.second.index] = p.second.speed ? 0.5*(p.second.min + p.second.max) : p.second.min;
}

void JointAxes::decodeKey(KeyMapping& key_map, int key, double value, double dt) 
{
	auto it = key_map.find(key);
	if (it != key_map.end()) {
		const KeyInfo& ki = it->second;
		// proceess KeyInfo
		if (ki.speed == 0.0) {
			double alpha = 0.5*(value + 1.0);
			joints_.position[ki.index] = (1.0-alpha)*ki.min + alpha*ki.max;
		}
		else {
			if (value != 0.0) {
				double pos = joints_.position[ki.index] + ki.speed*dt;
				if (pos < ki.min) pos = ki.min;
				if (pos > ki.max) pos = ki.max;
				joints_.position[ki.index] = pos;
			}
		}
	}
}

void JointAxes::decodeHook(const sensor_msgs::Joy& msg) {
	// caculate time shift and update timestamp
	if (joints_.header.stamp.isZero()) {
		joints_.header.stamp = msg.header.stamp;
		return;
	}
	double dt = (msg.header.stamp - joints_.header.stamp).toSec();
	joints_.header.stamp = msg.header.stamp;
	// now process axis and keys
	// process keys
	for(int k = 0; k < msg.buttons.size(); k++) decodeKey(key_map_, k, msg.buttons[k], dt);
	// process axes
	for(int k = 0; k < msg.axes.size(); k++) decodeKey(axis_map_, k, msg.axes[k], dt);
	// publish result
	pub_.publish(joints_);
}

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "joystick_decoder");
	ros::NodeHandle nh("~");
	std::vector< std::unique_ptr<JoyDecoder::JoyDecoderBase> > decoders;

	ROS_INFO("SweetieBot JoystickDecoder created.");

	// get YAML configuration from parameter server
	std::string yaml_config;
	if (!ros::param::get("~config", yaml_config)) {;
		ROS_ERROR("JoystickDecoder: unable to load configuration.");
		return -1;
	}
	try {
		// parse aconfiguration
		YAML::Node node = YAML::Load(yaml_config);
		for(YAML::iterator it = node.begin(); it != node.end(); it++) {
			std::string decoder_type = it->first.as<std::string>();
			// create corresponding decoder
			if (decoder_type == "KeyPressed") decoders.emplace_back( new JoyDecoder::KeyPressed(nh, it->second) );
			else if (decoder_type == "JointAxes") decoders.emplace_back( new JoyDecoder::JointAxes(nh, it->second) );
			else {
				throw YAML::Exception(YAML::Mark::null_mark(), "JoystickDecoder: unknown decoder_type: " + decoder_type);
			}
		}
	}
	catch (YAML::Exception& e) {
		ROS_ERROR_STREAM("JoystickDecoder: configuration parse error: " << e.what());
		return -1;
	}
	// start decoders
	for(auto& decoder : decoders) decoder->setEnabled(true);
	ros::spin();

	return 0;
}
