#include "trajectoryeditor.h"
#include "ui_Editor.h"

ros::Publisher  pub;
ros::Subscriber sub;

namespace ser = ros::serialization;

TrajectoryEditor::TrajectoryEditor(int argc, char *argv[], QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::TrajectoryEditor)
{
    ui->setupUi(this);

    // ROS
    ros::init(argc, argv, "trajectory_editor");
    ros::NodeHandle n;
    pub = n.advertise<control_msgs::FollowJointTrajectoryGoal>("pub_goal", 1);
    sub = n.subscribe<sensor_msgs::JointState>("/trajectory_editor/joints_real", 1000, &TrajectoryEditor::jointsRealCallback, this);
    QTimer *timer = new QTimer(this);
    connect(timer,SIGNAL(timeout()),this,SLOT(rosSpin()));
    timer->start(50);

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
}

TrajectoryEditor::~TrajectoryEditor()
{
    delete ui;
}

 
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
}

void TrajectoryEditor::rosSpin()
{
    // only for msg publication demonstration, delete this in real app
    trajectory_msgs::JointTrajectoryPoint traj;
    traj.positions.push_back(0.0);
    traj.velocities.push_back(0.0);
    traj.accelerations.push_back(0.0);
    traj.time_from_start.sec=0.0;
    traj.time_from_start.nsec=0.0;

    control_msgs::FollowJointTrajectoryGoal msg;
    msg.trajectory.joint_names.push_back("joint11");
    msg.trajectory.joint_names.push_back("joint12");
    msg.trajectory.points.push_back(traj);

    control_msgs::JointTolerance tol1;
    tol1.name = "joint11";
    tol1.position = 0.0;
    tol1.velocity = 0.0; 
    tol1.acceleration = 0.0;
    msg.path_tolerance.push_back(tol1);

    control_msgs::JointTolerance tol2;
    tol2.name = "joint11";
    tol2.position = 0.0;
    tol2.velocity = 0.0; 
    tol2.acceleration = 0.0;
    msg.goal_tolerance.push_back(tol2);

    msg.goal_time_tolerance.sec=0.0;
    msg.goal_time_tolerance.nsec=0.0;

    pub.publish(msg);
    // only for msg publication demonstration, delete this in real app

    ros::spinOnce();
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
