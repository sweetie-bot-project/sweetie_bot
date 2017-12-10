#include "trajectoryeditor.h"
#include "ui_Editor.h"

#include <QStandardItemModel>
#include <QMessageBox>

#include <std_srvs/SetBool.h>


TrajectoryEditor::TrajectoryEditor(int argc, char *argv[], ros::NodeHandle node, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::TrajectoryEditor),
    node_(node),
    loader_(new sweetie_bot::tools::ParamMsgLoader<control_msgs::FollowJointTrajectoryGoal>(node))
{

	if (!ros::param::get("~trajectory_storage", trajectories_param_name)) {
		trajectories_param_name = "/stored/joint_trajectory";
	}
    ROS_INFO_STREAM( "Trajectory storage namespace: " << trajectories_param_name );


    ui->setupUi(this);
    updateParamList();
	ui->comboBox->setCurrentText("default");

    ui->timeIncrementSpinBox->setValue(0.0);

    QTimer *timer = new QTimer(this);
    connect(timer,SIGNAL(timeout()),this,SLOT(rosSpin()));

    timer->start(50);

    sub_real_ = node.subscribe<sensor_msgs::JointState>("joints_real", 1, &TrajectoryEditor::jointsRealCallback, this);
    sub_virtual_ = node.subscribe<sensor_msgs::JointState>("joints_virtual", 1, &TrajectoryEditor::jointsVirtualCallback, this);
	pub_joints_virtual_set = node.advertise<sensor_msgs::JointState>("joints_virtual_set", 1);
	pub_joints_marker_set = node.advertise<sensor_msgs::JointState>("joints_marker_set", 1);
	
    torque_main_switch_ = node.serviceClient<std_srvs::SetBool>("set_torque_off"); //TODO persistent connection and button disable

    client_virtual = new Client("joint_trajectory_virtual", true);
    client_real    = new Client("joint_trajectory_real", true);
    //client_virtual->waitForServer();
    //state_ = boost::make_shared<actionlib::SimpleClientGoalState>(client->getState());
    //Client::ResultConstPtr result = *client->getResult();

    joint_trajectory_data_ = new sweetie_bot::interface::JointTrajectoryData();
    joint_list_table_model_ = new sweetie_bot::interface::JointListTableModel(parent, *joint_trajectory_data_);
    joint_trajectory_point_table_model_ = new sweetie_bot::interface::JointTrajectoryPointTableModel(parent, *joint_trajectory_data_);

    bootstrap();
}

TrajectoryEditor::~TrajectoryEditor()
{
    delete ui;
}

void TrajectoryEditor::jointsRealCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	joint_state_real_ = *msg;
	if(!ui->addRealPoseButton->isEnabled())
		ui->addRealPoseButton->setEnabled(true);
}

void TrajectoryEditor::jointsVirtualCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	joint_state_virtual_ = *msg;
	if(!ui->addVirtualPoseButton->isEnabled())
		ui->addVirtualPoseButton->setEnabled(true);
}

void TrajectoryEditor::updateParamList()
{
    QString tmp = ui->comboBox->currentText();
    std::vector<std::string> trajectory_names = loader_->getNames(trajectories_param_name);
    ui->comboBox->clear();
    for(auto &tname: trajectory_names)
    {
        ui->comboBox->addItem(QString::fromStdString(tname));
    }
    if(tmp != "") ui->comboBox->setCurrentText(tmp);
}

void TrajectoryEditor::rosSpin()
{
	// TODO trottle checks down
    ui->turnAllServoOnButton->setEnabled(torque_main_switch_.exists());
    if(!ros::ok()) close();
    ros::spinOnce();
}


void TrajectoryEditor::bootstrap()
{
	ui->pointsTableView->setModel(joint_trajectory_point_table_model_);
	ui->pointsTableView->setSelectionBehavior(QTableView::SelectRows);
	ui->pointsTableView->setSelectionMode(QTableView::SingleSelection); 

	ui->jointsTableView->setModel(joint_list_table_model_);
	ui->jointsTableView->setSelectionBehavior(QTableView::SelectRows);
	ui->jointsTableView->setSelectionMode(QTableView::SingleSelection); 
}

void TrajectoryEditor::on_loadTrajectoryButton_clicked()
{
	control_msgs::FollowJointTrajectoryGoal msg;
	std::string param_name = trajectories_param_name + "/" + ui->comboBox->currentText().toStdString();

	bool param_ok = loader_->getParam(param_name, msg);

	if (param_ok) {
		ROS_INFO("Loading FollowJointTrajectoryGoal message `%s`", param_name.c_str());
		try {
			joint_trajectory_data_->loadFromMsg( msg );
		} 
		catch (std::exception e) {
			ROS_ERROR("Incorrect FollowJointTrajectoryGoal message `%s`: %s", param_name.c_str(), e.what());
		}
	}
	else {
		joint_trajectory_data_->clear();
		if(ui->addRealPoseButton->isEnabled())
		{
			for(auto &name: joint_state_real_.name){
				joint_trajectory_data_->addJoint(name, ui->pathToleranceSpinBox->value(), ui->goalToleranceSpinBox->value());
			}
		} else if (ui->addVirtualPoseButton->isEnabled()){
			for(auto &name: joint_state_virtual_.name){
				joint_trajectory_data_->addJoint(name, ui->pathToleranceSpinBox->value(), ui->goalToleranceSpinBox->value());
			}
		}
	}
	joint_list_table_model_->reReadData();
	joint_trajectory_point_table_model_->reReadData();
	ui->goalTimeToleranceSpinBox->setValue(joint_trajectory_data_->getGoalTimeTolerance());
}

void TrajectoryEditor::on_turnAllServoOnButton_clicked()
{
	std_srvs::SetBool srv;

	// When TorqueMainSwitch controler is operational servos are off.
	// So we have to send false if we want to activate servos.
	srv.request.data  = !(ui->turnAllServoOnButton->text() == "Turn all servos on");

	if (! torque_main_switch_.call(srv)) {
		ROS_ERROR("TorqueMainSwitch setOperational service is not available.");
		return;
	}
	if (! srv.response.success) {
		ROS_ERROR("TorqueMainSwitch setOperational service returned false: %s.", srv.response.message.c_str());
		return;
	}

	// Operation has succesed
	ROS_INFO("TorqueMainSwitch setOperational call successed. Servos torque_off = %d.", (int) srv.request.data);
	// Change button label
	if (srv.request.data) ui->turnAllServoOnButton->setText("Turn all servos on");
	else ui->turnAllServoOnButton->setText("Turn all servos off");
}

void TrajectoryEditor::on_turnAllTrajectoryServosButton_clicked()
{
	ROS_INFO("Command to switch trajectory servos");
}

void TrajectoryEditor::on_turnAllSelectedServosButton_clicked()
{
	ROS_INFO("Command to switch selected servos");
}

void TrajectoryEditor::on_addVirtualPoseButton_clicked()
{
	ROS_INFO_STREAM("Add JointState to JointTrajectory:\n" << joint_state_virtual_ );
	joint_trajectory_data_->addPointMsg(joint_state_virtual_, ui->timeIncrementSpinBox->value());
	joint_trajectory_point_table_model_->reReadData();
}

void TrajectoryEditor::on_addRealPoseButton_clicked()
{
	try { 
		joint_trajectory_data_->addPointMsg(joint_state_real_, ui->timeIncrementSpinBox->value());
	}
	catch (std::exception e) {
		ROS_ERROR("Unable to add point to JointTrajectory: %s", e.what());
	}
    joint_trajectory_point_table_model_->reReadData();
}

void TrajectoryEditor::on_saveTrajectoryButton_clicked()
{
    loader_->setParam(trajectories_param_name + "/" + ui->comboBox->currentText().toStdString(), joint_trajectory_data_->getTrajectoryMsg());
    // std::string cmd = "rosparam dump `rospack find sweetie_bot_deploy`/joint_state_control/joint_trajectories.yaml " + trajectories_param_name;
    // system( cmd.c_str() );
    updateParamList();
}

void TrajectoryEditor::on_deletePoseButton_clicked()
{
	QModelIndex index = ui->pointsTableView->selectionModel()->currentIndex();
	if (index.isValid()) {
		joint_trajectory_point_table_model_->removeRow(index.row(), QModelIndex());
		joint_trajectory_point_table_model_->reReadData();
	}
}

/*void TrajectoryEditor::executeActionCallback(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryActionResultConstPtr& result)
{
	ROS_INFO("TrajectoryEditor::executeActionCallback");
}*/

void TrajectoryEditor::on_executeButton_clicked()
{
	bool reverse = ui->backwardCheckBox->isChecked();
	double scale = 1.0; //TODO add gui element
	control_msgs::FollowJointTrajectoryGoal goal = joint_trajectory_data_->getTrajectoryMsg(reverse);
	if(ui->virtualCheckBox->isChecked())
	{
		client_virtual->waitForServer();
		actionlib::SimpleClientGoalState state = client_virtual->sendGoalAndWait( goal );
		ROS_INFO_STREAM("\n" << goal);
		ROS_INFO("%s", state.toString().c_str());
		ui->statusLabel->setText("Status: " + QString::fromStdString(state.toString()) );
	}
	else {
		client_real->waitForServer();
		actionlib::SimpleClientGoalState state = client_real->sendGoalAndWait( goal );
		ROS_INFO_STREAM("\n" << goal);
		ROS_INFO("%s", state.toString().c_str());
		ui->statusLabel->setText("Status: " + QString::fromStdString(state.toString()) );
	}
	// TODO callback does not compile T_T
	//client->sendGoal(joint_trajectory_data_->follow_joint_trajectory_goal_, boost::bind(&TrajectoryEditor::executeActionCallback, this, _1, _2));
	//, Client::SimpleActiveCallback(), Client::SimpleFeedbackCallback());
}

void TrajectoryEditor::on_jointsTableView_clicked(const QModelIndex &index)
{
	int row = index.row();	
	// TODO row check?
	const sweetie_bot::interface::JointTrajectoryData::Joint& joint = joint_trajectory_data_->getJoint(row);
	ui->jointNameEditBox->setText(QString::fromStdString(joint.name));
	//ui->pathToleranceSpinBox->setValue(joint.path_tolerance);
	//ui->goalToleranceSpinBox->setValue(joint.goal_tolerance);
}

void TrajectoryEditor::on_addButton_clicked()
{
    joint_trajectory_data_->addJoint(ui->jointNameEditBox->text().toStdString(), ui->pathToleranceSpinBox->value(), ui->goalToleranceSpinBox->value());
    joint_list_table_model_->reReadData();
    joint_trajectory_point_table_model_->reReadData();
}

void TrajectoryEditor::on_applyButton_clicked()
{
    joint_trajectory_data_->setPathTolerance(ui->pathToleranceSpinBox->value());
    joint_trajectory_data_->setGoalTolerance(ui->goalToleranceSpinBox->value());
    joint_list_table_model_->reReadData();
}

void TrajectoryEditor::on_delButton_clicked()
{
	QModelIndex index  = ui->jointsTableView->selectionModel()->currentIndex();
	if (index.isValid()) {
		joint_list_table_model_->removeRow(index.row(), QModelIndex());
		joint_list_table_model_->reReadData();
		joint_trajectory_point_table_model_->reReadData();
	}
}

void TrajectoryEditor::on_delTrajectoryButton_clicked()
{
	std::string name = ui->comboBox->currentText().toStdString();
	if (name != "default") {
		node_.deleteParam(trajectories_param_name + "/" + name);
		updateParamList();
		ui->comboBox->setCurrentText("default");
	}
}


void TrajectoryEditor::on_goalTimeToleranceSpinBox_valueChanged(double value)
{
    joint_trajectory_data_->setGoalTimeTolerance(value);
}

void TrajectoryEditor::on_setVirtualPoseButton_clicked()
{
	QModelIndex index = ui->pointsTableView->selectionModel()->currentIndex();
	if (index.isValid()) {
		sensor_msgs::JointState msg = joint_trajectory_data_->getPointMsg(index.row());
		ROS_INFO_STREAM("\n" << msg);
		pub_joints_virtual_set.publish(msg);
	}
}

void TrajectoryEditor::on_pointsTableView_clicked(const QModelIndex &index)
{
	if(index.isValid()){
	  sensor_msgs::JointState msg = joint_trajectory_data_->getPointMsg(index.row());
	  ROS_INFO_STREAM("\n" << msg);
	  pub_joints_marker_set.publish(msg);
	}
}

void TrajectoryEditor::on_pointsTableView_doubleClicked(const QModelIndex &index)
{

}

void TrajectoryEditor::on_virtualCheckBox_clicked(bool checked)
{

}
