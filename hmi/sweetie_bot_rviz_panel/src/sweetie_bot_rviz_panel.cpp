#include "sweetie_bot_rviz_panel/sweetie_bot_rviz_panel.hpp"
#include <pluginlib/class_list_macros.hpp>


PLUGINLIB_EXPORT_CLASS(sweetie_bot_rviz_panel::sweetieBotRvizPanel, rviz::Panel)

namespace sweetie_bot_rviz_panel
{
    sweetieBotRvizPanel::sweetieBotRvizPanel(QWidget * parent)
    :   rviz::Panel(parent),
        ui_(std::make_shared<Ui::control_buttons>())
    {
        // Extend the widget with all attributes and children from UI file
        ui_->setupUi(this);

        // Absolute, idempotent operational control + ground-truth state feed.
        set_operational_caller_ = nh_.serviceClient<std_srvs::SetBool>("/soar/set_operational");
        operational_sub_ = nh_.subscribe("/soar/operational", 1, &sweetieBotRvizPanel::operational_callback, this);

        connect(ui_->pushButtonToggleOperational, SIGNAL(clicked()), this, SLOT(button_toggle_operational()));
        connect(ui_->pushButtonMotorStateViewer, SIGNAL(clicked()), this, SLOT(button_start_motor_state_viewer()));
        connect(ui_->pushButtonTrajectoryEditor, SIGNAL(clicked()), this, SLOT(button_start_trajectory_editor()));
        connect(ui_->pushButtonKillJointStateRef, SIGNAL(clicked()), this, SLOT(button_kill_joint_state_ref()));

        // Reflect /soar/operational into the label on the GUI thread. AutoConnection is Direct when
        // the ROS callback already runs on the GUI thread, Queued otherwise -- correct either way.
        connect(this, SIGNAL(operationalStateChanged(bool)), this, SLOT(set_operational_label(bool)));
    }


    void sweetieBotRvizPanel::button_toggle_operational()
    {
        ROS_INFO_STREAM("Button 'toggle_operational' pressed.");
        // Send an absolute target (flip of the last known state) -- idempotent, so a stray
        // duplicate carries the same value and cannot cause a spurious extra toggle.
        srv_.request.data = !operational_;
        if ( !set_operational_caller_.call(srv_) || !srv_.response.success )
        {
            ROS_ERROR("Failed to set operational state via /soar/set_operational");
        }
        // The label is driven by /soar/operational (ground truth), not by this response.
    }


    void sweetieBotRvizPanel::operational_callback(const std_msgs::Bool::ConstPtr & msg)
    {
        // Marshal to the GUI thread; never touch Qt widgets or shared state from here.
        Q_EMIT operationalStateChanged(msg->data);
    }


    void sweetieBotRvizPanel::set_operational_label(bool operational)
    {
        operational_ = operational;
        ui_->labelToggleOperational->setText( operational ? "True" : "False" );
    }


    void sweetieBotRvizPanel::button_start_motor_state_viewer()
    {
        ROS_INFO_STREAM("Button 'motor_state_viewer' is pressed.");
        std::string command = "rosrun sweetie_bot_motor_state_viewer motor_state_viewer &";
        int result = system(command.c_str());
        if (result == -1)
        {
            ROS_ERROR("Failed to execute motor_state_viewer");
            ui_->labelMotorStateViewer->setText( QString("Failed") );
        }
	else
	{
            ui_->labelMotorStateViewer->setText( QString("Started") );
        }
    }


    void sweetieBotRvizPanel::button_start_trajectory_editor()
    {
        ROS_INFO_STREAM("Button 'trajectory_editor' is pressed.");
        std::string command = "roslaunch sweetie_bot_deploy joint_trajectory_editor.launch &";
        int result = system(command.c_str());
        if (result == -1)
        {
            ROS_ERROR("Failed to execute trajectory_editor");
            ui_->labelTrajectoryEditor->setText( QString("Failed") );
        }
	else
	{
            ui_->labelTrajectoryEditor->setText( QString("Started") );
        }
    }


    void sweetieBotRvizPanel::button_kill_joint_state_ref()
    {
        ROS_INFO_STREAM("Button 'kill_joint_state_ref' is pressed.");
        std::string command = "rosnode kill /hmi/joint_state_ref &";
        int result = system(command.c_str());
        if (result == -1)
        {
            ROS_ERROR("Failed to kill joint_state_ref");
            ui_->labelKillJointStateRef->setText( QString("Failed") );
        }
	else
	{
            ui_->labelKillJointStateRef->setText( QString("Killed") );
        }
    }


    /**
     *  Save all configuration data from this panel to the given
     *  Config object. It is important here that you call save()
     *  on the parent class so the class id and panel name get saved.
     */
    void sweetieBotRvizPanel::save(rviz::Config config) const
    {
        rviz::Panel::save(config);
    }

    /**
     *  Load all configuration data for this panel from the given Config object.
     */
    void sweetieBotRvizPanel::load(const rviz::Config & config)
    {
        rviz::Panel::load(config);
    }
} // namespace sweetie_bot_rviz_panel
