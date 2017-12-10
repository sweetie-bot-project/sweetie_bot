#include "joint_list_table_view.h"

namespace sweetie_bot {
namespace interface {

JointListTableModel::JointListTableModel(QObject *parent, JointTrajectoryData &trajectory_data) :
	QAbstractTableModel(parent),
	trajectory_data_(trajectory_data)
{ }

int JointListTableModel::rowCount(const QModelIndex & /*parent*/) const
{
   return trajectory_data_.jointCount();
}

int JointListTableModel::columnCount(const QModelIndex & /*parent*/) const
{
    return 3;
}

QVariant JointListTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (role == Qt::DisplayRole)
    {
        if (orientation == Qt::Horizontal) {
            switch (section)
            {
            case 0:
                return QString("joint_name");
            case 1:
                return QString("path_tolerance");
            case 2:
                return QString("goal_tolerance");
            }
        }
    }
    return QVariant();
}

QVariant JointListTableModel::data(const QModelIndex &index, int role) const
{
    if (role == Qt::DisplayRole)
    {
		// if (index.row() >= 0 && index.row() < trajectory_data_.jointCount();
		if (index.isValid()) {
			const JointTrajectoryData::Joint& joint = trajectory_data_.getJoint(index.row());
			switch (index.column()) {
				case 0: 
					return QString::fromStdString(joint.name);
				case 1:
					return QString::number(joint.path_tolerance);
				case 2:
					return QString::number(joint.goal_tolerance);
			}
		}
    }
    return QVariant();
}

bool JointListTableModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
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

bool JointListTableModel::removeRow(int row, const QModelIndex &parent)
{
	//TODO somehow we lock  JointTrajectoryPointTableModel coluns to make this opperation correct. 
	// It cannot be done from hear. Possible solutions:
	// 1. Do not use column per joint view.
	// 2. Lock rows and JointTrajectoryPointTableModel and cols of JointTrajectoryPointTableModel in trajectoryeditor.cpp.
	//TODO row index check
	beginRemoveRows(parent, row, row);
	trajectory_data_.removeJoint(row);
	endRemoveRows();
	return true;
}

bool JointListTableModel::reReadData()
{
	/*
	   QModelIndex indexBegin = createIndex(0,0);
	   QModelIndex indexEnd = createIndex(1,trajectory_data_.joint_names_.size());
	   emit dataChanged(indexBegin, indexEnd);
	// */
	emit layoutChanged();
}

} // namespace interface
} // namespace sweetie_bot
