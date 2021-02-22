#include "joint_list_table_view.h"

namespace sweetie_bot {
namespace hmi {

JointListTableModel::JointListTableModel(QObject *parent, JointTrajectoryData &trajectory_data) :
	QAbstractTableModel(parent),
	trajectory_data_(trajectory_data)
{ }

int JointListTableModel::rowCount(const QModelIndex & /*parent*/) const
{
   return trajectory_data_.supportCount() + trajectory_data_.jointCount();
}

int JointListTableModel::columnCount(const QModelIndex & /*parent*/) const
{
    return 4;
}

QVariant JointListTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (role == Qt::DisplayRole)
    {
        if (orientation == Qt::Horizontal) {
            switch (section) {
            case 0:
                return QString("joint_name");
            case 1:
                return QString("type");
            case 2:
                return QString("path_tolerance");
            case 3:
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
			if (index.row() < trajectory_data_.supportCount()) {
				const JointTrajectoryData::Support& joint = trajectory_data_.getSupport(index.row());
				switch (index.column()) {
					case 0: 
						return QString::fromStdString(joint.name);
					case 1:
						return QString("support");
				}
			}
			else {
				const JointTrajectoryData::Joint& joint = trajectory_data_.getJoint(index.row() -  trajectory_data_.supportCount());
				switch (index.column()) {
					case 0: 
						return QString::fromStdString(joint.name);
					case 1:
						return QString("joint");
					case 2:
						return QString::number(joint.path_tolerance);
					case 3:
						return QString::number(joint.goal_tolerance);
				}
			}
		}
    }
    return QVariant();
}

Qt::ItemFlags JointListTableModel::flags(const QModelIndex &index) const
{
	switch (index.column()) {
		case 0:
		case 1:
			return Qt::ItemIsSelectable |  Qt::ItemIsEnabled;
		case 2:
		case 3:
			if (index.row() < trajectory_data_.supportCount()) return Qt::ItemIsSelectable;
			else return Qt::ItemIsSelectable |  Qt::ItemIsEditable | Qt::ItemIsEnabled;
	}
	return QAbstractItemModel::flags(index);
}

bool JointListTableModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
	if (!index.isValid()) return false;
	if (!value.isValid()) return false;
	if (value.toString() == "") return false;
	if (!value.canConvert(QMetaType::Double)) return false;
	double d = value.toDouble();

	int joint_index = index.row() - trajectory_data_.supportCount();
	if (joint_index < 0) return false;

	switch (index.column()) {
		case 1:
			trajectory_data_.setJointPathTolerance(joint_index, d);
			break;
		case 2:
			trajectory_data_.setJointPathTolerance(joint_index, d);
			break;
		default:
			return false;
	}
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
	if (row < trajectory_data_.supportCount()) trajectory_data_.removeSupport(row);
	else trajectory_data_.removeJoint(row - trajectory_data_.supportCount());
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
	return true;
}

} // namespace hmi
} // namespace sweetie_bot
