#ifndef sweetie_bot_rviz_panel_H_
#define sweetie_bot_rviz_panel_H_

#include <ros/ros.h>
#include <rviz/panel.h>

/** 
 *  Include header generated from ui file
 *  Note that you will need to use add_library function first
 *  in order to generate the header file from ui.
 */
#include <ui_sweetie_bot_rviz_panel.h>

// Other ROS dependencies
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>

namespace sweetie_bot_rviz_panel
{
    /**
     *  Here we declare our new subclass of rviz::Panel. Every panel which
     *  can be added via the Panels/Add_New_Panel menu is a subclass of
     *  rviz::Panel.
     */

    class sweetieBotRvizPanel : public rviz::Panel
    {
        /**
         * This class uses Qt slots and is a subclass of QObject, so it needs
         * the Q_OBJECT macro.
         */
        Q_OBJECT

        public:
            /**
             *  QWidget subclass constructors usually take a parent widget
             *  parameter (which usually defaults to 0).  At the same time,
             *  pluginlib::ClassLoader creates instances by calling the default
             *  constructor (with no arguments). Taking the parameter and giving
             *  a default of 0 lets the default constructor work and also lets
             *  someone using the class for something else to pass in a parent
             *  widget as they normally would with Qt.
             */
            sweetieBotRvizPanel(QWidget * parent = 0);

            /**
             *  Now we declare overrides of rviz::Panel functions for saving and
             *  loading data from the config file.  Here the data is the topic name.
             */
            virtual void save(rviz::Config config) const;
            virtual void load(const rviz::Config & config);

        /**
         *  Signals emitted from ROS callback threads; the GUI thread updates
         *  widgets in the connected slot (queued when cross-thread).
         */
        Q_SIGNALS:

            void operationalStateChanged(bool operational);

        /**
         *  Next come a couple of public Qt Slots.
         */
        public Q_SLOTS:

        /**
         *  Here we declare some internal slots.
         */
        private Q_SLOTS:

            void button_toggle_operational();
            void button_start_motor_state_viewer();
            void button_start_trajectory_editor();
            void button_kill_joint_state_ref();
            // Runs on the GUI thread: cache the operational state and update the label.
            void set_operational_label(bool operational);

        /**
         *  Finally, we close up with protected member variables
         */
        protected:
            // Reflect /soar/operational into the label (marshalled to the GUI thread).
            void operational_callback(const std_msgs::Bool::ConstPtr & msg);

            // UI pointer
            std::shared_ptr<Ui::control_buttons> ui_;
            // ROS declaration
            ros::NodeHandle nh_;
	    ros::ServiceClient set_operational_caller_;
            ros::Subscriber operational_sub_;
            std_srvs::SetBool srv_;
            // Last state seen on /soar/operational; written only on the GUI thread.
            bool operational_ = false;
    };
} // namespace sweetie_bot_rviz_panel
#endif
