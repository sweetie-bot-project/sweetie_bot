#include "joint_trajectory_point_table_view.h"

#include <QFont>
#include <QBrush>

namespace sweetie_bot {
namespace hmi {

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
	return 2 + trajectory_data_.supportCount() + trajectory_data_.jointCount();
}

QVariant JointTrajectoryPointTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if (orientation == Qt::Horizontal) {
		switch (role) {
			case Qt::DisplayRole:
				switch (section) {
					case 0:
						return QString("label");
					case 1:
						return QString("time");
					default:
						if (section < 2 + trajectory_data_.supportCount()) {
							const std::string& name = trajectory_data_.getSupport(section - 2).name;
							return QString::fromStdString(name);
						} 
						else {
							const std::string& name = trajectory_data_.getJoint(section - 2 - trajectory_data_.supportCount()).name;
							return QString::fromStdString(name);
						}
				}
				break;
			/*case Qt::TextAlignmentRole:
				return Qt::AlignLeft + Qt::AlignVCenter;*/
		}
	}
	return QVariant();
}

const std::vector<std::string> JointTrajectoryPointTableModel::symbols = {
	":)) ", ":-) ", "=) ", "%) ", ":o)", ":( ", ":-( ", ";-) ", ";v)", ":D", ":-D ", "=D ", ":^D", ":-/ ", 
	":/", ":P ", ":-р ", "=p ", ":-b ", "=b", ":-* ", ":-x", "8-] ", ":-] ", "=]", ":-|", "8) ", "8-)", ":-o ", 
	"=O", ":'-( ", ":'( ", ":,-(", ":,-) ", ":'-)", ":*-) ", "%*", ":-Q", "X-)", ":-X", 
	"(^_^)", "(^.^)", "(^__^)", "(^..^)", "(^___^)", "(o_O)", "(;_;)", "(T_T)", "(ToT)", "($_$)", "(@ @)", "(*_*)", 
	"(+_+)", "(^o^)", "(^3^)", "(*-*)", "(-_-)", "(-_-)Zzzz", 
	"IMO", "IMHO ", "LOL", "ROFL", "GG", "GL", "HF", "BB", "AKA", "BTW", "FYA", "THX", "10X", 
};


QVariant JointTrajectoryPointTableModel::data(const QModelIndex &index, int role) const
{
	if (!index.isValid()) return QVariant();

	const JointTrajectoryData::TrajectoryPoint& point = trajectory_data_.getPoint(index.row());
	switch (role) {
		case Qt::DisplayRole:
			switch (index.column()) {
				case 0: 
					return QString::fromStdString(symbols[point.crc % symbols.size()]);
				case 1: 
					return QString::number(point.time_from_start, 'f', 2);
				default:
					if (index.column() < 2 + trajectory_data_.supportCount()) {
						return QString::number(point.supports.at(index.column() - 2), 'f', 2);
					}
					else {
						return QString::number(point.positions.at(index.column() - 2 - trajectory_data_.supportCount()), 'f', 2);
					}
			}
			break;

		case Qt::CheckStateRole:
			if (index.column() >= 2 && index.column() < 2 + trajectory_data_.supportCount()) {
				if (point.supports.at(index.column() - 2) > 0) return Qt::Checked;
				else return Qt::Unchecked;
			}
			break;

		case Qt::BackgroundRole:
			switch (index.column()) {
				case 0: 
					return QBrush(QColor(248 - 32 + 2*(point.crc & 0xf), 185 - 32 + 4*((point.crc >> 4) & 0xf), 206 - 16 + 2*((point.crc >> 8) & 0xf)));
				case 1: 
					return QBrush(QColor(163,223,249));
			}
	}
	return QVariant();
}

Qt::ItemFlags JointTrajectoryPointTableModel::flags (const QModelIndex &index) const
{
	switch (index.column()) {
		case 0:
			return Qt::ItemIsSelectable |  Qt::ItemIsEnabled;
		default:
			if (index.column() < 2 + trajectory_data_.supportCount()) {
				return Qt::ItemIsSelectable |  Qt::ItemIsEditable | Qt::ItemIsEnabled | Qt::ItemIsUserCheckable;
			}
			else {
				return Qt::ItemIsSelectable |  Qt::ItemIsEditable | Qt::ItemIsEnabled;
			}
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
	// check value
	if (!value.isValid()) return false;
	if (value.toString() == "") return false;
	if (!value.canConvert(QMetaType::Double)) return false;
	double d = value.toDouble();
	//ROS_INFO("%f %s", d, value.toString().toStdString().c_str());

	switch (index.column()) {
		case 0:
			return false;
		case 1: {
			auto& point = trajectory_data_.getPoint(index.row());
			double old_time_from_start = point.time_from_start;

			// edit time_from_start
			trajectory_data_.setPointTimeFromStart(index.row(), d);

			emit itemChangedWithOldValue(index, old_time_from_start);
			emit dataChanged(index, index);
			return true;
		}
		default: 
			if (index.column() < 2 + trajectory_data_.supportCount()) {
				auto support_index = index.column() - 2;

				auto& point = trajectory_data_.getPoint(index.row());
				double old_support_state = point.supports[support_index];

				// edit support state
				if (role == Qt::CheckStateRole)	trajectory_data_.setPointSupport(index.row(), support_index, (Qt::CheckState)value.toInt() == Qt::Checked ? 1.0 : 0.0);
				else trajectory_data_.setPointSupport(index.row(), support_index, d);

				emit itemChangedWithOldValue(index, old_support_state);
				emit dataChanged(index, index);
				return true;
			}
			else {
				auto position_value_index = index.column() - 2 - trajectory_data_.supportCount();

				auto& point = trajectory_data_.getPoint(index.row());
				double old_position = point.positions[position_value_index];

				// edit joint position
				trajectory_data_.setPointJointPosition(index.row(), position_value_index, d);

				emit itemChangedWithOldValue(index, old_position);
				emit dataChanged(index, index);
				return true;
			}
	}

	return false;
}

bool JointTrajectoryPointTableModel::reReadData()
{
	emit layoutChanged();
	return true;
}

} // namespace hmi
} // namespace sweetie_bot
