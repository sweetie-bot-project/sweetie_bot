#include "joint_trajectory_point_table_view.h"

#include <QFont>

namespace sweetie_bot {
namespace interface {

JointTrajectoryPointTableModel::JointTrajectoryPointTableModel(QObject *parent, JointTrajectoryData &trajectory_data) :
	QAbstractTableModel(parent),
	trajectory_data_(trajectory_data)
{
}

int JointTrajectoryPointTableModel::rowCount(const QModelIndex & /*parent*/) const
{
   return trajectory_data_.pointCount();
}

int JointTrajectoryPointTableModel::columnCount(const QModelIndex & /*parent*/) const
{
    return 1 + trajectory_data_.jointCount();
}

QVariant JointTrajectoryPointTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if (orientation == Qt::Horizontal) {
		switch (role) {
			case Qt::DisplayRole:
				if (section == 0) {
					return QString("time");
				}
				else if (section >= 1 && section <= trajectory_data_.jointCount()) {
				//else {
					const std::string& name = trajectory_data_.getJoint(section - 1).name;
					return QString::fromStdString(name);
				}
				/*QString tmp;
				const std::string& name = trajectory_data_.getJoint(i).name;
				for(int i = 0; i < trajectory_data_.jointCount(); i++) {
					tmp.append( QString("%1, ").arg(QString::fromStdString( name.substr(std::max<int>(name.size()-5,0)) )) );
				}
				return tmp;*/
				break;

			/*case Qt::TextAlignmentRole:
				return Qt::AlignLeft + Qt::AlignVCenter;*/
    	}
	}
    return QVariant();
}

QVariant JointTrajectoryPointTableModel::data(const QModelIndex &index, int role) const
{
    if (role == Qt::DisplayRole) {
		//TODO row index check
		const JointTrajectoryData::TrajectoryPoint& point = trajectory_data_.getPoint(index.row());
		switch (index.column()) {
			case 0: 
				return QString::number(point.time_from_start, 'f', 2);
			/*case 1:
				if (point.positions.size() == 0) { 
					return QString();
				}
				else {
					QString tmp;
					for(auto it = point.positions.begin(); it != point.positions.end(); it++)
						tmp.append( QString("%1, ").arg(*it, 5, 'f', 2) );
					return tmp;
				}*/
			default:
				if (index.column() >= 0 && index.column() <= trajectory_data_.jointCount()) {
					return QString::number(point.positions[index.column()-1], 'f', 2);
				}
		}
    }
    return QVariant();
}

Qt::ItemFlags JointTrajectoryPointTableModel::flags (const QModelIndex &index) const
{
	switch (index.column()) {
		case 0:
			return Qt::ItemIsSelectable |  Qt::ItemIsEditable | Qt::ItemIsEnabled;
		case 1:
			return Qt::ItemIsSelectable |  Qt::ItemIsEnabled;
	}
	return QAbstractItemModel::flags(index);
}

bool JointTrajectoryPointTableModel::removeRow(int row, const QModelIndex &parent)
{
	//TODO row index check
	beginRemoveRows(parent, row, row);
	trajectory_data_.removePoint(row);
	endRemoveRows();
	return true;
}

bool JointTrajectoryPointTableModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
	if (!index.isValid()) return false;
	if (index.column() == 0) {
		if (!value.isValid()) return false;
		if (value.toString() == "") return false;
		if (!value.canConvert(QMetaType::Double)) return false;
		double d = value.toDouble();
		trajectory_data_.setPointTimeFromStart(index.row(), d);
		// ROS_INFO("%f %s", d, value.toString().toStdString().c_str());
		emit dataChanged(index, index);
	}
	return false;
}

bool JointTrajectoryPointTableModel::reReadData()
{
	emit layoutChanged();
}

} // namespace interface
} // namespace sweetie_bot
