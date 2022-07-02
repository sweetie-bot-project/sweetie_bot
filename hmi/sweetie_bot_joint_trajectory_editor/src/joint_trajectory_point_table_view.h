#ifndef JOINT_TRAJECTORY_POINT_TABLE_VIEW_H
#define JOINT_TRAJECTORY_POINT_TABLE_VIEW_H

#include "joint_trajectory_data.h"

#include <QAbstractTableModel>

namespace sweetie_bot {
namespace hmi {

class JointTrajectoryPointTableModel : public QAbstractTableModel
{
	Q_OBJECT
	private:
		static const std::vector<std::string> symbols;

	private:
		JointTrajectoryData &trajectory_data_;
	public:
		JointTrajectoryPointTableModel(QObject *parent, JointTrajectoryData &trajectory_data);
		int rowCount(const QModelIndex &parent = QModelIndex()) const override ;
		int columnCount(const QModelIndex &parent = QModelIndex()) const override;
		QVariant headerData(int section, Qt::Orientation orientation, int role) const;
		QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
		bool removeRow(int row, const QModelIndex &parent);
		Qt::ItemFlags flags (const QModelIndex &index) const override;
		bool setData(const QModelIndex &index, const QVariant &value, int role);
		bool reReadData();
	signals:
		void itemAboutToChange(const QModelIndex &index, double old_value, double new_value);
};

} // namespace hmi
} // namespace sweetie_bot

#endif // JOINT_TRAJECTORY_POINT_TABLE_VIEW_H
