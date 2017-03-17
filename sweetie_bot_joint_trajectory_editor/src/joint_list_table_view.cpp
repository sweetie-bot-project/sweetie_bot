#include "joint_list_table_view.h"

namespace sweetie_bot {
namespace interface {

JointListTableView::JointListTableView(QObject *parent, JointTrajectoryData &trajectory_data)
  :QAbstractTableModel(parent),
   trajectory_data_(trajectory_data)
{

}

int JointListTableView::rowCount(const QModelIndex & /*parent*/) const
{
   return trajectory_data_.follow_joint_trajectory_goal_.trajectory.joint_names.size();
}

int JointListTableView::columnCount(const QModelIndex & /*parent*/) const
{
    return 1;
}

QVariant JointListTableView::data(const QModelIndex &index, int role) const
{
    if (role == Qt::DisplayRole)
    {
      return QString(trajectory_data_.follow_joint_trajectory_goal_.trajectory.joint_names.at(index.row()).c_str());
    }
    return QVariant();
}

bool JointListTableView::setData(const QModelIndex &index, const QVariant &value, int role)
{
  ROS_INFO("setData");
  if (!index.isValid())
    return false;
/*
  QList<QVariant> data;
  data<<value;
  TreeItem *item = static_cast<TreeItem *>(index.internalPointer());
  item->setData(data);
  emit dataChanged(index, index);
*/
  emit dataChanged(index, index);
  return true;
}

bool JointListTableView::removeRow(int row, const QModelIndex &parent)
{
    beginRemoveRows(parent, row, row-1);
    trajectory_data_.removeJoint(row);
    endRemoveRows();
    return true;
}

bool JointListTableView::rereadData()
{
  ROS_INFO("JointListTableView::rereadData");
  /*
  QModelIndex indexBegin = createIndex(0,0);
  QModelIndex indexEnd = createIndex(1,trajectory_data_.joint_names_.size());
  emit dataChanged(indexBegin, indexEnd);
  // */
  emit layoutChanged();
}

} // namespace interface
} // namespace sweetie_bot
