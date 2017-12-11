#ifndef JOINT_TRAJECTORY_VIEW_H
#define JOINT_TRAJECTORY_VIEW_H

#include "joint_trajectory_data.h"

#include <QAbstractTableModel>

namespace sweetie_bot {
namespace interface {

	class JointListTableModel : public QAbstractTableModel
	{
		Q_OBJECT
		private:
			JointTrajectoryData &trajectory_data_;
		public:
			JointListTableModel(QObject *parent, JointTrajectoryData &trajectory_data);
			int rowCount(const QModelIndex &parent = QModelIndex()) const override ;
			int columnCount(const QModelIndex &parent = QModelIndex()) const override;
			QVariant headerData(int section, Qt::Orientation orientation, int role) const;
			QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
			bool removeRow(int row, const QModelIndex &parent);
			Qt::ItemFlags flags(const QModelIndex &index) const;
			bool setData(const QModelIndex &index, const QVariant &value, int role);
			bool reReadData();
	};

} // namespace interface
} // namespace sweetie_bot

#endif // JOINT_TRAJECTORY_VIEW_H
