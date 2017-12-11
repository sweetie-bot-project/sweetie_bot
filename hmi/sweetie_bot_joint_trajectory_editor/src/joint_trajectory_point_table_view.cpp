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
    return 2 + trajectory_data_.jointCount();
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
						if (section >= 2 && section < 2 + trajectory_data_.jointCount()) {
							const std::string& name = trajectory_data_.getJoint(section - 2).name;
							return QString::fromStdString(name);
						}
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
	const JointTrajectoryData::TrajectoryPoint& point = trajectory_data_.getPoint(index.row());
	switch (role) {
		case Qt::DisplayRole:
			switch (index.column()) {
				case 0: 
					return QString::fromStdString(symbols[point.crc % symbols.size()]);
				case 1: 
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
					if (index.column() >= 2 && index.column() < 2 + trajectory_data_.jointCount()) {
						return QString::number(point.positions[index.column()-2], 'f', 2);
					}
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
			return Qt::ItemIsSelectable |  Qt::ItemIsEditable | Qt::ItemIsEnabled;
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
	// ROS_INFO("%f %s", d, value.toString().toStdString().c_str());

	switch (index.column()) {
		case 1:
			// edit time_from_start
			trajectory_data_.setPointTimeFromStart(index.row(), d);
			emit dataChanged(index, index);
			return true;
		default: 
			if (index.column() >= 2 && index.column() < 2 + trajectory_data_.jointCount()) {
				// edit position
				trajectory_data_.setPointJointPosition(index.row(), index.column() - 2, d);
				emit dataChanged(index, index);
				return true;
			}
	}

	return false;
}

bool JointTrajectoryPointTableModel::reReadData()
{
	emit layoutChanged();
}

} // namespace hmi
} // namespace sweetie_bot
