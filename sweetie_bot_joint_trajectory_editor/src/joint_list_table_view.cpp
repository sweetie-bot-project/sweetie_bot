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
   return trajectory_data_.joint_names_.size();
}

int JointListTableView::columnCount(const QModelIndex & /*parent*/) const
{
    return 1;
}

QVariant JointListTableView::data(const QModelIndex &index, int role) const
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
