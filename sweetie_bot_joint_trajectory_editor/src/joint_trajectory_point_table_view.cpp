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
   return trajectory_data_.follow_joint_trajectory_goal_.trajectory.points.size();
}

int JointTrajectoryPointTableView::columnCount(const QModelIndex & /*parent*/) const
{
    return 1;
}

QVariant JointTrajectoryPointTableView::data(const QModelIndex &index, int role) const
{
    if (role == Qt::DisplayRole)
    {
       return QString("Row%1, Column%2")
                   .arg(index.row() + 1)
                   .arg(index.column() +1);
    }
    return QVariant();
}

} // namespace interface
} // namespace sweetie_bot
