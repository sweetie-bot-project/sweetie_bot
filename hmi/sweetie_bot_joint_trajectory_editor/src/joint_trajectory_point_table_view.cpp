#include "joint_trajectory_point_table_view.h"

namespace sweetie_bot {
namespace interface {

JointTrajectoryPointTableView::JointTrajectoryPointTableView(QObject *parent, JointTrajectoryData &trajectory_data)
  :QAbstractTableModel(parent),
   trajectory_data_(trajectory_data)
{

}

int JointTrajectoryPointTableView::rowCount(const QModelIndex & /*parent*/) const
{
   return trajectory_data_.pointCount();
}

int JointTrajectoryPointTableView::columnCount(const QModelIndex & /*parent*/) const
{
    return 1;
}

QVariant JointTrajectoryPointTableView::data(const QModelIndex &index, int role) const
{
    if (role == Qt::DisplayRole)
    {
       return QString::number(trajectory_data_.getPointTimeFromStart(index.row()));
       /*
       return QString("%1.%2")
                   .arg(d.sec)
                   .arg(round(d.nsec));
                   // */
    }
    return QVariant();
}

Qt::ItemFlags JointTrajectoryPointTableView::flags (const QModelIndex &index) const
{
    return QAbstractItemModel::flags(index) | Qt::ItemIsEditable;
}

bool JointTrajectoryPointTableView::removeRow(int row, const QModelIndex &parent)
{
    beginRemoveRows(parent, row, row-1);
    trajectory_data_.removePoint(row);
    endRemoveRows();
    return true;
}

bool JointTrajectoryPointTableView::setData(const QModelIndex &index, const QVariant &value, int role)
{
  if (!index.isValid())
    return false;
  double d = 0.0;
  if(value.toString() == "") return false;
  if(value.isValid() && value.canConvert(QMetaType::Double))
    d = value.toDouble();
  else return false;
  //trajectory_data_.follow_joint_trajectory_goal_.trajectory.points[index.row()].time_from_start.fromSec(d);
  trajectory_data_.setPointTimeFromStart(index.row(), d);
  //ROS_INFO("%f %s", d, value.toString().toStdString().c_str());

  emit dataChanged(index, index);
  return true;
}

bool JointTrajectoryPointTableView::reReadData()
{
	emit layoutChanged();
}

} // namespace interface
} // namespace sweetie_bot
