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

    std::vector<std::string> trajectory_names = loader_->getNames("/sweetie_bot_joint_trajectory_editor/trajectories");
    for(auto &tname: trajectory_names)
    {
        ui->comboBox->addItem(QString::fromStdString(tname));
    }


    QTimer *timer = new QTimer(this);
    connect(timer,SIGNAL(timeout()),this,SLOT(rosSpin()));
    timer->start(50);
//*
    // only for msg publication demonstration, delete this in real app
    control_msgs::FollowJointTrajectoryGoal msg;

    trajectory_msgs::JointTrajectoryPoint traj;
    traj.positions.push_back(0.0);
    traj.velocities.push_back(0.0);
    traj.accelerations.push_back(0.0);
    traj.time_from_start.sec=0.0;
    traj.time_from_start.nsec=0.0;

    msg.trajectory.joint_names.push_back("joint11");
    msg.trajectory.joint_names.push_back("joint12");
    msg.trajectory.joint_names.push_back("joint13");
    msg.trajectory.joint_names.push_back("joint14");
    msg.trajectory.points.push_back(traj);

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

    msg.goal_time_tolerance.sec=0.0;
    msg.goal_time_tolerance.nsec=0.0;

    loader_->setParam("/sweetie_bot_joint_trajectory_editor/trajectories/default", msg);
    //pub.publish(msg);
    // only for msg publication demonstration, delete this in real app
// */

    control_msgs::FollowJointTrajectoryGoal sd = loader_->getParam("/sweetie_bot_joint_trajectory_editor/trajectories/default");
    ROS_INFO_STREAM( "\n" << sd );
    joint_trajectory_data_ = new sweetie_bot::interface::JointTrajectoryData(sd);

    joint_list_table_view_ = new sweetie_bot::interface::JointListTableView(parent, *joint_trajectory_data_);
    joint_trajectory_point_table_view_ = new sweetie_bot::interface::JointTrajectoryPointTableView(parent, *joint_trajectory_data_);

    bootstrap();
    //joint_trajectory_data_->sendTraj();
/*
    // Load msg from file
    sensor_msgs::JointState msg;
    std::ifstream ifs("/tmp/filename.txt", std::ios::in|std::ios::binary);

    ifs.seekg (0, std::ios::end);
    std::streampos end = ifs.tellg();
    ifs.seekg (0, std::ios::beg);
    std::streampos begin = ifs.tellg();

    uint32_t file_size = end-begin;
    ROS_INFO("file_size=%d", file_size );
    if(file_size > 0){
      boost::shared_array<uint8_t> ibuffer(new uint8_t[file_size]);

      ifs.read((char*) ibuffer.get(), file_size);
      ros::serialization::IStream istream(ibuffer.get(), file_size);
      ros::serialization::deserialize(istream, msg);
      ifs.close();
      ROS_INFO("Msg loaded from file!" );
      //ROS_INFO_STREAM("\n" <<  msg);
    }
    // Load msg from file
*/
}

TrajectoryEditor::~TrajectoryEditor()
{
    delete ui;
}

/* 
void TrajectoryEditor::jointsRealCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  ROS_INFO_STREAM("\n" <<  *msg);

  // Save msg to File
  std::ofstream ofs("/tmp/filename.txt", std::ios::out|std::ios::binary);

  uint32_t serial_size = ros::serialization::serializationLength(*msg);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);

  ser::OStream ostream(buffer.get(), serial_size);
  ser::serialize(ostream, *msg);

  ofs.write((char*) buffer.get(), serial_size);
  ofs.close();
  // Save msg to File
} // */

void TrajectoryEditor::rosSpin()
{
    //ROS_INFO("spinOnce");
    ros::spinOnce();
}

void TrajectoryEditor::bootstrap()
{
  //QStandardItemModel *model = new QStandardItemModel(3, 4, ui->tableView);
  ui->tableView->setModel(joint_trajectory_point_table_view_);
  ui->tableView_2->setModel(joint_list_table_view_);
/*
  ui->tableView->verticalHeader()->hide();
  ui->tableView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  QStringList headerLabels = QStringList() << tr("time") << tr("marker") << tr ("active") << tr("angles");
  model->setHorizontalHeaderLabels(headerLabels);

  for(int row=0; row!=model->rowCount(); ++row)
  {
      for(int column=0; column!=model->columnCount(); ++column)
      {
          QStandardItem *newItem = new QStandardItem(tr("%1").arg((row+1)*(column+1)));
          model->setItem(row, column, newItem);
      }
  }
*/
//  model->itemChanged();
}

void TrajectoryEditor::on_pushButton_4_clicked()
{
  if (ui->pushButton_4->text() == "Turn all servos on")
  {
    qDebug("Command to turn on servos");
    ui->pushButton_4->setText("Turn all servos off");
  }
  else
  {
    qDebug("Command to turn off servos");
    ui->pushButton_4->setText("Turn all servos on");
  }
}

void TrajectoryEditor::on_pushButton_5_clicked()
{
  qDebug("Command to switch selected servos");
}

void TrajectoryEditor::on_pushButton_6_clicked()
{
  qDebug("Command to switch trajectory servos");

}

void TrajectoryEditor::on_pushButton_10_clicked()
{
  qDebug("Load");
  //ui->comboBox->currentText().toStdString()
  control_msgs::FollowJointTrajectoryGoal msg = loader_->getParam("/sweetie_bot_joint_trajectory_editor/trajectories/" + ui->comboBox->currentText().toStdString());
  joint_trajectory_data_->loadFromMsg( msg );
  //bootstrap();

  //emit joint_list_table_view_->modelReset();
  //joint_trajectory_point_table_view_->dataChanged(startIndex, stopIndex );
  //ui->tableView->update();
//  ui->tableView_2->
}
