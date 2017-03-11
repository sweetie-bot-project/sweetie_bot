#include "trajectoryeditor.h"
#include "ui_Editor.h"

#include <QStandardItemModel>
#include <QMessageBox>

TrajectoryEditor::TrajectoryEditor(int argc, char *argv[], ros::NodeHandle node, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::TrajectoryEditor),
    node_(node),
    loader_(new sweetie_bot::tools::ParamMsgLoader<control_msgs::FollowJointTrajectoryGoal>(node))
{
    ui->setupUi(this);

    //*
        control_msgs::FollowJointTrajectoryGoal msg;
        msg.trajectory.header.stamp = ros::Time::now();
        msg.goal_time_tolerance.sec=2.0;
        msg.goal_time_tolerance.nsec=1.0;

        trajectory_msgs::JointTrajectoryPoint traj;

        for(int i=0; i<6; i++){
          traj.positions.push_back(0.0);
        }
        traj.time_from_start.sec=0.0;
        traj.time_from_start.nsec=0.0;
        msg.trajectory.points.push_back(traj);

        traj.positions.clear();
        traj.positions.push_back(qDegreesToRadians(45.0));
        traj.positions.push_back(qDegreesToRadians(-90.0));
        traj.positions.push_back(qDegreesToRadians(45.0));
        traj.positions.push_back(qDegreesToRadians(45.0));
        traj.positions.push_back(qDegreesToRadians(-90.0));
        traj.positions.push_back(qDegreesToRadians(45.0));

        traj.time_from_start.sec=1.0;
        traj.time_from_start.nsec=0.0;
        msg.trajectory.points.push_back(traj);

        traj.positions.clear();
        for(int i=0; i<6; i++){
          traj.positions.push_back(0.0);
        }
        traj.time_from_start.sec=2.0;
        traj.time_from_start.nsec=0.0;
        msg.trajectory.points.push_back(traj);

        msg.trajectory.joint_names.push_back("joint12");
        msg.trajectory.joint_names.push_back("joint13");
        msg.trajectory.joint_names.push_back("joint14");
        msg.trajectory.joint_names.push_back("joint42");
        msg.trajectory.joint_names.push_back("joint43");
        msg.trajectory.joint_names.push_back("joint44");
    /*
        control_msgs::JointTolerance tol1;
        tol1.name = "joint11";
        tol1.position = 0.1;
        tol1.velocity = 0.1;
        tol1.acceleration = 0.1;
        msg.path_tolerance.push_back(tol1);

        tol1.name = "joint12";
        tol1.position = 0.2;
        tol1.velocity = 0.2;
        tol1.acceleration = 0.2;
        msg.path_tolerance.push_back(tol1);


        control_msgs::JointTolerance tol2;
        tol2.name = "joint11";
        tol2.position = 0.1;
        tol2.velocity = 0.1;
        tol2.acceleration = 0.1;
        msg.goal_tolerance.push_back(tol2);

    // */
        loader_->setParam("/sweetie_bot_joint_trajectory_editor/trajectories/dance", msg);

    std::vector<std::string> trajectory_names = loader_->getNames("/sweetie_bot_joint_trajectory_editor/trajectories");
    for(auto &tname: trajectory_names)
    {
        ui->comboBox->addItem(QString::fromStdString(tname));
    }

    int index = ui->comboBox->findText("default"); //use default exact match
    if(index >= 0)
         ui->comboBox->setCurrentIndex(index);

    ui->timeIncrementSpinBox->setValue(1.0);

    QTimer *timer = new QTimer(this);
    connect(timer,SIGNAL(timeout()),this,SLOT(rosSpin()));
    timer->start(50);

    sub_real_ = node.subscribe<sensor_msgs::JointState>("/trajectory_editor/joints_real", 1000, &TrajectoryEditor::jointsRealCallback, this);
    sub_virtual_ = node.subscribe<sensor_msgs::JointState>("/trajectory_editor/joints_virtual", 1000, &TrajectoryEditor::jointsVirtualCallback, this);

    client = new Client("/sweetie_bot/motion/controller/joint_trajectory", true);


    control_msgs::FollowJointTrajectoryGoal sd = loader_->getParam("/sweetie_bot_joint_trajectory_editor/trajectories/default");
    ROS_INFO_STREAM( "\n" << sd );
    joint_trajectory_data_ = new sweetie_bot::interface::JointTrajectoryData(sd);

    joint_list_table_view_ = new sweetie_bot::interface::JointListTableView(parent, *joint_trajectory_data_);
    joint_trajectory_point_table_view_ = new sweetie_bot::interface::JointTrajectoryPointTableView(parent, *joint_trajectory_data_);

    bootstrap();
}

TrajectoryEditor::~TrajectoryEditor()
{
    delete ui;
}

//*
void TrajectoryEditor::jointsRealCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  ROS_INFO_STREAM("\n" <<  *msg);
  ui->addRealPoseButton->setEnabled(true);
} // */

//*
void TrajectoryEditor::jointsVirtualCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  ROS_INFO_STREAM("\n" <<  *msg);
  ui->addVirtualPoseButton->setEnabled(true);
} // */

void TrajectoryEditor::rosSpin()
{
    ros::spinOnce();
}

void TrajectoryEditor::bootstrap()
{
  ui->pointsTableView->setModel(joint_trajectory_point_table_view_);
  ui->jointsTableView->setModel(joint_list_table_view_);
}

void TrajectoryEditor::on_loadTrajectoryButton_clicked()
{
  qDebug("Load");
  control_msgs::FollowJointTrajectoryGoal msg = loader_->getParam("/sweetie_bot_joint_trajectory_editor/trajectories/" + ui->comboBox->currentText().toStdString());
  joint_trajectory_data_->loadFromMsg( msg );
  joint_list_table_view_->rereadData();
  joint_trajectory_point_table_view_->rereadData();
}

void TrajectoryEditor::on_turnAllServoOnButton_clicked()
{

  if (ui->turnAllServoOnButton->text() == "Turn all servos on")
  {
    qDebug("Command to turn on servos");
    ui->turnAllServoOnButton->setText("Turn all servos off");
  }
  else
  {
    qDebug("Command to turn off servos");
    ui->turnAllServoOnButton->setText("Turn all servos on");
  }
}

void TrajectoryEditor::on_turnAllTrajectoryServosButton_clicked()
{
  qDebug("Command to switch trajectory servos");
}

void TrajectoryEditor::on_turnAllSelectedServosButton_clicked()
{
  qDebug("Command to switch selected servos");
}

void TrajectoryEditor::on_addVirtualPoseButton_clicked()
{
  joint_trajectory_data_->addPoint(joint_state_virtual_, ui->timeIncrementSpinBox->value());
  joint_trajectory_point_table_view_->rereadData();
}

void TrajectoryEditor::on_addRealPoseButton_clicked()
{
    joint_trajectory_data_->addPoint(joint_state_real_, ui->timeIncrementSpinBox->value());
    joint_trajectory_point_table_view_->rereadData();
}

void TrajectoryEditor::on_saveTrajectoryButton_clicked()
{
    loader_->setParam("/sweetie_bot_joint_trajectory_editor/trajectories/" + ui->comboBox->currentText().toStdString(), joint_trajectory_data_->follow_joint_trajectory_goal_);
    system("rosparam dump `rospack find sweetie_bot_joint_trajectory_editor`/launch/trajectories.yaml /sweetie_bot_joint_trajectory_editor/trajectories" );
}

void TrajectoryEditor::on_deletePoseButton_clicked()
{
    ROS_INFO("%d", ui->pointsTableView->selectionModel()->currentIndex().row());
    joint_trajectory_point_table_view_->removeRow(ui->pointsTableView->selectionModel()->currentIndex().row(), QModelIndex());
    joint_trajectory_point_table_view_->rereadData();
}


void TrajectoryEditor::on_executeButton_clicked()
{
  client->waitForServer();
  client->sendGoal(joint_trajectory_data_->follow_joint_trajectory_goal_);
}

/*
void TrajectoryEditor::executeActionCallback(const SimpleClientGoalState& state, const ResultConstPtr& result)
{

}
*/
