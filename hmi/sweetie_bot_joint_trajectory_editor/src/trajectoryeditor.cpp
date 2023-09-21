#include "trajectoryeditor.h"

#include <cmath>

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

	// initialize ROS interface
    sub_joints_ = node.subscribe<sensor_msgs::JointState>("joint_states", 1, &TrajectoryEditor::jointsCallback, this);
	pub_joints_set_ = node.advertise<sensor_msgs::JointState>("joint_states_set", 1);
	pub_joints_marker_set_ = node.advertise<sensor_msgs::JointState>("joints_marker_set", 1);
    torque_main_switch_ = node.serviceClient<std_srvs::SetBool>("set_torque_off"); //TODO persistent connection and button disable

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
	if(!ui.addRobotPoseButton->isEnabled())
		ui.addRobotPoseButton->setEnabled(true);
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
	//client->sendGoal(joint_trajectory_data_.follow_joint_trajectory_goal_, boost::bind(&TrajectoryEditor::executeActionCallback, this, boost::placeholders::_1, boost::placeholders::_2));
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
