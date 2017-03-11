#ifndef JOINT_TRAJECTORY_POINT_TABLE_VIEW_H
#define JOINT_TRAJECTORY_POINT_TABLE_VIEW_H

#include "joint_trajectory_data.h"

#include <QAbstractTableModel>

namespace sweetie_bot {
namespace interface {

class JointTrajectoryPointTableView : public QAbstractTableModel
{
  Q_OBJECT
private:
  JointTrajectoryData &trajectory_data_;
public:
  JointTrajectoryPointTableView(QObject *parent, JointTrajectoryData &trajectory_data);
  int rowCount(const QModelIndex &parent = QModelIndex()) const override ;
  int columnCount(const QModelIndex &parent = QModelIndex()) const override;
  QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
};

} // namespace interface
} // namespace sweetie_bot

#endif // JOINT_TRAJECTORY_POINT_TABLE_VIEW_H
