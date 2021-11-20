#include "trajectoryeditor.h"

#include <cmath>
#include <sstream>

#include <QStandardItemModel>
#include <QMessageBox>

#include <std_srvs/SetBool.h>

TrajectoryEditor::TrajectoryEditor(int argc, char *argv[], ros::NodeHandle node, QWidget *parent) :
	QMainWindow(parent),
	node_(node),
	loader_(node), // paramtere loader
	joint_trajectory_data_(), // data model
	joint_list_table_model_(parent, joint_trajectory_data_), // joint list table model
	joint_trajectory_point_table_model_(parent, joint_trajectory_data_) // trajectory points table model
{
	// setup GUI
	ui.setupUi(this);
	bootstrap();

	// load parameters
	if (!ros::param::get("~trajectory_storage", trajectories_param_name)) {
		trajectories_param_name = "joint_trajectory";
	}
	ROS_INFO_STREAM( "Trajectory storage namespace: " << trajectories_param_name );
	updateParamList();
	ui.comboBox->setCurrentText("default");

	// initialize limbs
	for (auto &limb_on_state: is_limb_on) {
		limb_on_state = true;
	}
	// @Note: Think of a better way to specify leg joints without hardcoding names
	limb_joint_names = {
		std::vector<std::string>({"joint11", "joint12", "joint13", "joint14", "joint15"}),
		std::vector<std::string>({"joint21", "joint22", "joint23", "joint24", "joint25"}),
		std::vector<std::string>({"joint31", "joint32", "joint33", "joint34", "joint35"}),
		std::vector<std::string>({"joint41", "joint42", "joint43", "joint44", "joint45"}),
		std::vector<std::string>({"joint51", "joint52", "joint53", "joint54"})
	};

	for (int i = 1; i <= 5; i++) {
		for (int j = 1; j <= 5; j++) {
			if (i == 5 && j == 5) break;

			std::ostringstream joint_name;
			joint_name << "joint" << i << j;
			is_joint_torque_on.insert(std::pair<std::string, JointTorqueState>(joint_name.str(), UNINITIALIZED));
		}
	}

	// initialize ROS interface
	sub_joints_ = node.subscribe<sensor_msgs::JointState>("joint_states", 1, &TrajectoryEditor::jointsCallback, this);
	pub_joints_set_ = node.advertise<sensor_msgs::JointState>("joint_states_set", 1);
	pub_joints_marker_set_ = node.advertise<sensor_msgs::JointState>("joints_marker_set", 1);
	torque_main_switch_ = node.serviceClient<std_srvs::SetBool>("set_torque_off"); //TODO persistent connection and button disable

	pub_servo_commands_ = node.advertise<sweetie_bot_herkulex_msgs::ServoCommands>("servo_commands", 1);
	sub_servo_states_ = node.subscribe<sweetie_bot_herkulex_msgs::HerkulexState>("servo_states", 1, &TrajectoryEditor::servoStateCallback, this);
	sub_servo_joint_states_ = node.subscribe<sweetie_bot_herkulex_msgs::HerkulexJointState>("servo_joint_states", 1, &TrajectoryEditor::servoJointsCallback, this);

	action_execute_trajectory_ = new ActionClient("joint_trajectory", true);
	//action_execute_trajectory_->waitForServer();
	//state_ = boost::make_shared<actionlib::SimpleClientGoalState>(client->getState());
	//Client::ResultConstPtr result = *client->getResult();

	// setup timer to signal rosSpin()
	timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(rosSpin()));
	timer->start(50);

}

TrajectoryEditor::~TrajectoryEditor() 
{
	delete timer;
	delete action_execute_trajectory_;
}


void TrajectoryEditor::jointsCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	joint_state_ = *msg;
	if (!ui.addRobotPoseButton->isEnabled()) {
		ui.addRobotPoseButton->setEnabled(true);
	}

	if (ui.autoUpdatePoseCheckBox->isChecked()) {
		updateSelectedPose(true);
	}
}

void TrajectoryEditor::servoJointsCallback(const sweetie_bot_herkulex_msgs::HerkulexJointState::ConstPtr& msg) {
	auto joint_state = *msg;

	sensor_msgs::JointState joint_state_msg;
	joint_state_msg.header = std_msgs::Header();
	joint_state_msg.header.frame_id = "odom_combined";

	// Publish positions of only disabled joints
	for (int i = 0; i < joint_state.name.size(); i++) {
		auto &joint_name = joint_state.name[i];
		if (is_joint_torque_on[joint_name] == TORQUE_OFF) {
			joint_state_msg.name.push_back(joint_name);
			joint_state_msg.position.push_back(joint_state.pos[i]);
		}
	}
	pub_joints_set_.publish(joint_state_msg);
}

void TrajectoryEditor::servoStateCallback(const sweetie_bot_herkulex_msgs::HerkulexState::ConstPtr& msg) {
	bool is_torque_on = msg->status_detail & 0x40;
	if (is_torque_on) {
		is_joint_torque_on[msg->name] = TORQUE_ON;
	} else {
		is_joint_torque_on[msg->name] = TORQUE_OFF;
	}
}

void TrajectoryEditor::updateParamList()
{
	QString tmp = ui.comboBox->currentText();
	std::vector<std::string> trajectory_names = loader_.getNames(trajectories_param_name);
	ui.comboBox->clear();
	for(auto &tname: trajectory_names)
	{
		ui.comboBox->addItem(QString::fromStdString(tname));
	}
	if(tmp != "") ui.comboBox->setCurrentText(tmp);
}

void TrajectoryEditor::rosSpin()
{
	// TODO trottle checks down
	ui.turnAllServoOnButton->setEnabled(torque_main_switch_.exists());

	if (pub_servo_commands_.getNumSubscribers() > 0) {
		ui.leg1ToggleButton->setEnabled(true);
		ui.leg2ToggleButton->setEnabled(true);
		ui.leg3ToggleButton->setEnabled(true);
		ui.leg4ToggleButton->setEnabled(true);
		ui.headToggleButton->setEnabled(true);
	}

	if(!ros::ok()) close();
	ros::spinOnce();
}


void TrajectoryEditor::bootstrap()
{
	// configure joint trajectory pointis' TableView
	ui.pointsTableView->setModel(&joint_trajectory_point_table_model_);
	ui.pointsTableView->setSelectionBehavior(QTableView::SelectRows);
	ui.pointsTableView->setSelectionMode(QTableView::SingleSelection); 

	// configure joints' TableView
	ui.jointsTableView->setModel(&joint_list_table_model_);
	ui.jointsTableView->setSelectionBehavior(QTableView::SelectRows);
	ui.jointsTableView->setSelectionMode(QTableView::SingleSelection); 
}

void TrajectoryEditor::on_loadTrajectoryButton_clicked()
{
	control_msgs::FollowJointTrajectoryGoal msg;
	std::string param_name = trajectories_param_name + "/" + ui.comboBox->currentText().toStdString();

	bool param_ok = loader_.getParam(param_name, msg);

	if (param_ok) {
		ROS_INFO("Loading FollowJointTrajectoryGoal message `%s`", param_name.c_str());
		try {
			joint_trajectory_data_.loadFromMsg( msg );
		} 
		catch (std::exception e) {
			ROS_ERROR("Incorrect FollowJointTrajectoryGoal message `%s`: %s", param_name.c_str(), e.what());
		}
	}
	else {
		joint_trajectory_data_.clear();
		// check if joint_state data is available
		if(ui.addRobotPoseButton->isEnabled()) {
			for(auto &name: joint_state_.name){
				joint_trajectory_data_.addJoint(name, ui.pathToleranceSpinBox->value(), ui.goalToleranceSpinBox->value());
			}
		}
	}
	joint_list_table_model_.reReadData();
	joint_trajectory_point_table_model_.reReadData();
	ui.goalTimeToleranceSpinBox->setValue(joint_trajectory_data_.getGoalTimeTolerance());
}


void TrajectoryEditor::setServoTorqueOn(bool value) 
{
	std_srvs::SetBool srv;
	srv.request.data = value;

	if (! torque_main_switch_.call(srv)) {
		// Operation unavailable
		ui.servoStateLabel->setText("UNAVALIBLE");
		ui.servoStateLabel->setStyleSheet("font-weight: bold; color: red");
		ROS_ERROR("TorqueMainSwitch setOperational service is not available.");
		return;
	}
	if (!srv.response.success) {
		// Operation failed
		ui.servoStateLabel->setText("ERROR");
		ui.servoStateLabel->setStyleSheet("font-weight: bold; color: red");
		ROS_ERROR("TorqueMainSwitch setOperational service returned false: %s.", srv.response.message.c_str());
		return;
	}
	// Operation has succesed
	ROS_INFO("TorqueMainSwitch setOperational call successed. Servos torque_off = %d.", (int) srv.request.data);
	// Change button label
	if (srv.request.data) {
		ui.servoStateLabel->setText("ON");
		ui.servoStateLabel->setStyleSheet("font-weight: bold; color: orange");
	}
	else {
		ui.servoStateLabel->setText("OFF");
		ui.servoStateLabel->setStyleSheet("font-weight: bold; color: green");
	}
}

void TrajectoryEditor::on_turnAllServoOnButton_clicked()
{
	setServoTorqueOn(true);
}

void TrajectoryEditor::on_turnAllServoOffButton_clicked()
{
	setServoTorqueOn(false);
}

void TrajectoryEditor::on_leg1ToggleButton_clicked()
{
	handleLimbButtonToggle(0, ui.leg1ToggleButton);
}

void TrajectoryEditor::on_leg2ToggleButton_clicked()
{
	handleLimbButtonToggle(1, ui.leg2ToggleButton);
}

void TrajectoryEditor::on_leg3ToggleButton_clicked()
{
	handleLimbButtonToggle(2, ui.leg3ToggleButton);
}

void TrajectoryEditor::on_leg4ToggleButton_clicked()
{
	handleLimbButtonToggle(3, ui.leg4ToggleButton);
}

void TrajectoryEditor::on_headToggleButton_clicked()
{
	handleLimbButtonToggle(4, ui.headToggleButton);
}

void TrajectoryEditor::handleLimbButtonToggle(int limb_index, QPushButton *button) {
	if (is_limb_on[limb_index]) {
		button->setText("ON");
		is_limb_on[limb_index] = false;
		setLimbServoTorqueOn(limb_index, is_limb_on[limb_index]);
	} else {
		button->setText("OFF");
		is_limb_on[limb_index] = true;
		setLimbServoTorqueOn(limb_index, is_limb_on[limb_index]);
	}
}

void TrajectoryEditor::setLimbServoTorqueOn(int limb_index, bool set_on) 
{
	sweetie_bot_herkulex_msgs::ServoCommands command_msg;
	if (set_on) {
		command_msg.command = sweetie_bot_herkulex_msgs::ServoCommands::TORQUE_ON;
	} else {
		command_msg.command = sweetie_bot_herkulex_msgs::ServoCommands::TORQUE_OFF;
	}
	for (auto &joint_name: limb_joint_names[limb_index]) {
		command_msg.name.push_back(joint_name);
	}
	pub_servo_commands_.publish(command_msg);
}

void TrajectoryEditor::on_addRobotPoseButton_clicked()
{
	double time_from_start = ui.timeFromStartSpinBox->value();
	ROS_INFO_STREAM("Add JointState to JointTrajectory, time_from_start = " << time_from_start);
	ROS_DEBUG_STREAM("\n" << joint_state_ );
	try { 
		joint_trajectory_data_.addPointMsg(joint_state_, time_from_start);
	}
	catch (std::exception e) {
		ROS_ERROR("Unable to add point to JointTrajectory: %s", e.what());
		return;
	}
	ui.timeFromStartSpinBox->setValue(time_from_start + ui.timeIncrementSpinBox->value());
	joint_trajectory_point_table_model_.reReadData();
}

void TrajectoryEditor::on_dublicatePoseButton_clicked()
{
	QModelIndex index = ui.pointsTableView->selectionModel()->currentIndex();
	if (index.isValid() && index.row() < joint_trajectory_data_.pointCount()) {
		double time_from_start = ui.timeFromStartSpinBox->value();
		ROS_INFO_STREAM("Duplicate pose with index " << index.row() << " at time " << time_from_start);

		try { 
			sweetie_bot::hmi::JointTrajectoryData::TrajectoryPoint point = joint_trajectory_data_.getPoint(index.row());
			point.time_from_start = time_from_start;
			joint_trajectory_data_.addPoint(point);
		}
		catch (std::exception e) {
			ROS_ERROR("Unable to add point to JointTrajectory: %s", e.what());
			return;
		}
		ui.timeFromStartSpinBox->setValue(time_from_start + ui.timeIncrementSpinBox->value());
		joint_trajectory_point_table_model_.reReadData();
	}
}

void TrajectoryEditor::on_updateRobotPoseButton_clicked()
{
	updateSelectedPose();
}

void TrajectoryEditor::updateSelectedPose(bool update_only_disabled)
{
	for (int i = 0; i < joint_state_.name.size(); i++) {
		QModelIndex index = ui.pointsTableView->selectionModel()->currentIndex();
		if (index.isValid() && index.row() < joint_trajectory_data_.pointCount()) {
			auto &joint_name = joint_state_.name[i];
			if (update_only_disabled && !(is_joint_torque_on[joint_name] == TORQUE_ON))  continue;
				
			auto joint_index = joint_trajectory_data_.getJointIndex(joint_name);
			if (joint_index != -1) {
				joint_trajectory_data_.setPointJointPosition(index.row(), joint_index, joint_state_.position[i]);
			}
		}
	}
}

void TrajectoryEditor::on_saveTrajectoryButton_clicked()
{
	loader_.setParam(trajectories_param_name + "/" + ui.comboBox->currentText().toStdString(), joint_trajectory_data_.getTrajectoryMsg());
	// std::string cmd = "rosparam dump `rospack find sweetie_bot_deploy`/joint_state_control/joint_trajectories.yaml " + trajectories_param_name;
	// system( cmd.c_str() );
	updateParamList();
}

void TrajectoryEditor::on_deletePoseButton_clicked()
{
	QModelIndex index = ui.pointsTableView->selectionModel()->currentIndex();
	ROS_INFO_STREAM("Delete pose with index " << index.row());
	if (index.isValid() && index.row() < joint_trajectory_data_.pointCount()) {
		joint_trajectory_point_table_model_.removeRow(index.row(), QModelIndex());
		joint_trajectory_point_table_model_.reReadData();
	}
}

/*void TrajectoryEditor::executeActionCallback(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryActionResultConstPtr& result)
{
	ROS_INFO("TrajectoryEditor::executeActionCallback");
}*/

void TrajectoryEditor::on_executeButton_clicked()
{
	bool reverse = ui.backwardCheckBox->isChecked();
	double scale = ui.scaleSpinBox->value(); 

	control_msgs::FollowJointTrajectoryGoal goal = joint_trajectory_data_.getTrajectoryMsg(reverse, scale);
	ROS_DEBUG_STREAM("\n" << goal);

	// check if action server is available 
	if (!action_execute_trajectory_->isServerConnected()) {
		if (!action_execute_trajectory_->waitForServer(ros::Duration(1, 0))) {
			ROS_ERROR("FollowJointTrajectory action server is unavailible");
			ui.statusLabel->setText("UNAVALABLE");
			ui.statusLabel->setStyleSheet("font-weight: bold; color: red");
			return;
		}
	}
	// execute
	ROS_INFO("Send goal to FollowJointTrajectory action server.");
	actionlib::SimpleClientGoalState state = action_execute_trajectory_->sendGoalAndWait( goal );

	// TODO analize result
	ROS_INFO("Result: %s", state.toString().c_str());
	ui.statusLabel->setText(QString::fromStdString(state.toString()) );
	ui.statusLabel->setStyleSheet("font-weight: bold");

	// TODO callback does not compile T_T
	//client->sendGoal(joint_trajectory_data_.follow_joint_trajectory_goal_, boost::bind(&TrajectoryEditor::executeActionCallback, this, _1, _2));
	//, Client::SimpleActiveCallback(), Client::SimpleFeedbackCallback());
}

void TrajectoryEditor::on_jointsTableView_clicked(const QModelIndex &index)
{
	if (index.isValid()) {
		std::string name;
		if (index.row() < joint_trajectory_data_.supportCount()) name = joint_trajectory_data_.getSupport(index.row()).name;
		else name = joint_trajectory_data_.getJoint(index.row() - joint_trajectory_data_.supportCount()).name;
		ui.jointNameEditBox->setText(QString::fromStdString(name));
	}
}

void TrajectoryEditor::on_addJointButton_clicked()
{
	std::string name = ui.jointNameEditBox->text().toStdString();
	if (name.compare(0,8,"support/") == 0) joint_trajectory_data_.addSupport(name.substr(8));
	else joint_trajectory_data_.addJoint(name, ui.pathToleranceSpinBox->value(), ui.goalToleranceSpinBox->value());
	joint_list_table_model_.reReadData();
	joint_trajectory_point_table_model_.reReadData();
}

void TrajectoryEditor::on_resetTolerancesButton_clicked()
{
	joint_trajectory_data_.setPathTolerance(ui.pathToleranceSpinBox->value());
	joint_trajectory_data_.setGoalTolerance(ui.goalToleranceSpinBox->value());
	joint_list_table_model_.reReadData();
}

void TrajectoryEditor::on_delJointButton_clicked()
{
	QModelIndex index  = ui.jointsTableView->selectionModel()->currentIndex();
	if (index.isValid() && index.row() < joint_trajectory_data_.jointCount()+joint_trajectory_data_.supportCount()) {
		joint_list_table_model_.removeRow(index.row(), QModelIndex());
		joint_list_table_model_.reReadData();
		joint_trajectory_point_table_model_.reReadData();
	}
}

void TrajectoryEditor::on_delTrajectoryButton_clicked()
{
	std::string name = ui.comboBox->currentText().toStdString();
	if (name != "default") {
		node_.deleteParam(trajectories_param_name + "/" + name);
		updateParamList();
		ui.comboBox->setCurrentText("default");
	}
}

void TrajectoryEditor::on_goalTimeToleranceSpinBox_valueChanged(double value)
{
	joint_trajectory_data_.setGoalTimeTolerance(value);
}

void TrajectoryEditor::on_setRobotPoseButton_clicked()
{
	QModelIndex index = ui.pointsTableView->selectionModel()->currentIndex();
	if (index.isValid() && index.row() < joint_trajectory_data_.pointCount()) {
		sensor_msgs::JointState msg = joint_trajectory_data_.getPointMsg(index.row());
		ROS_DEBUG_STREAM("\n" << msg);
		pub_joints_set_.publish(msg);
	}
}

void TrajectoryEditor::on_pointsTableView_clicked(const QModelIndex &index)
{
	if (index.isValid() && index.row() < joint_trajectory_data_.pointCount()) {
		sensor_msgs::JointState msg = joint_trajectory_data_.getPointMsg(index.row());
		ROS_DEBUG_STREAM("\n" << msg);
		pub_joints_marker_set_.publish(msg);
		if (ui.autoSetPoseCheckBox->isChecked()) {
			 pub_joints_set_.publish(msg);
		}
	}
}

void TrajectoryEditor::on_pointsTableView_doubleClicked(const QModelIndex &index)
{
	if (index.isValid() && index.row() < joint_trajectory_data_.pointCount()) {
		// publish market msg
		sensor_msgs::JointState msg = joint_trajectory_data_.getPointMsg(index.row());
		pub_joints_marker_set_.publish(msg);
		// set time_from_start
		double time_from_start = joint_trajectory_data_.getPoint(index.row()).time_from_start;
		ui.timeFromStartSpinBox->setValue(time_from_start + ui.timeIncrementSpinBox->value());
		if (ui.autoSetPoseCheckBox->isChecked()) {
			 pub_joints_set_.publish(msg);
		}
	}
}


void TrajectoryEditor::on_applyScaleButton_clicked() 
{
	joint_trajectory_data_.scaleTrajectory(ui.scaleSpinBox->value());
	joint_trajectory_point_table_model_.reReadData();
}

void TrajectoryEditor::on_scaleSpinBox_valueChanged(double scale) 
{
	// double scale = ui.scaleSpinBox->value();
	ui.scaleSlider->setValue(50 + 50*std::log10(scale));
}

void TrajectoryEditor::on_scaleSlider_sliderMoved(int scale) 
{
	// int scale = ui.scaleSlider->value();
	ui.scaleSpinBox->setValue(std::pow(10.0, (scale - 50.0) / 50.0));
}
