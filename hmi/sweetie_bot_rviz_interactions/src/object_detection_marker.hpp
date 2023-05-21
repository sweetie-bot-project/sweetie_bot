#ifndef  OBJECT_DETECTION_MARKER_HPP
#define  OBJECT_DETECTION_MARKER_HPP

#include <random>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <sweetie_bot_text_msgs/Detection.h>
#include <sweetie_bot_text_msgs/DetectionArray.h>

namespace sweetie_bot {
namespace hmi {

class ObjectDetectionMarker {
public:
	ObjectDetectionMarker(const std::string& _name, ros::Publisher& detection_publisher, std::shared_ptr<interactive_markers::InteractiveMarkerServer> server, ros::NodeHandle node_handle);

    // mark class not copy/move assignable: boost::bind stores this value to call callbacks
	ObjectDetectionMarker(const ObjectDetectionMarker&) = delete;
	ObjectDetectionMarker& operator= (const ObjectDetectionMarker&) = delete;

protected:
	visualization_msgs::Marker makeSphereMarker(float r, float g, float b);
	visualization_msgs::InteractiveMarker  makeInteractiveMarker(const geometry_msgs::Pose& pose);
	void updateInteractiveMarker();
	void updateInteractiveMarkerColor(float r, float b, float g);
	void makeMenu();

	void publishCallback(const ros::TimerEvent&);

    void processToggleVisibility( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback );
	void processChangeLabel( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback );
	void processChangeType( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback );
    void processClickNewId( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback );
    void processClickNormalize( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback );
    void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback );

protected:
	// CONNECTION
	// publishers
	ros::Publisher publisher;
	
	// PARAMETERS
	// node name 
	std::string name;
	// detections labels and types
	std::vector<std::string> labels, types;
	// marker frame
	std::string frame;
	// marker sacle parameter
	double scale;
	// normaized height
	double normalized_z_level; 
	// is 3D control enabled
	bool is6DOF;
	
	// COMPONENT STATE
	// is emulated object is visible
	bool is_publishing;
	// object detection message
	sweetie_bot_text_msgs::Detection detection;
	// timer 
	ros::Timer publish_timer;
	// interactive marker server
	std::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
	// menu
	interactive_markers::MenuHandler menu_handler;
};

class RandomObjectDetectionMarker : public ObjectDetectionMarker
{
protected:
	// state change probabilities
	double prob_vanish, prob_appear;
	// marker state
	bool is_randomizing;
	bool is_visible;
	ros::Time change_visibility_timestamp;
	// random generators
	std::mt19937 rand_gen;
	std::uniform_real_distribution<float> appear_time_dist;
	std::uniform_real_distribution<float> vanish_time_dist;
	std::uniform_real_distribution<float> lag_dist;

public:
	RandomObjectDetectionMarker(const std::string& _name, ros::Publisher& detection_publisher, std::shared_ptr<interactive_markers::InteractiveMarkerServer>& server, ros::NodeHandle& node_handle); 

	void publishCallback(const ros::TimerEvent&);
	void processToggleRandomize( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback );
};
} // namespace hmi
} // namespace sweetie_bot

#endif
