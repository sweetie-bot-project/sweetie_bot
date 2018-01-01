#ifndef PARAMMSG_H
#define PARAMMSG_H

/**
 * @brief ParamMsgLoader class.
 * Provide methods for easy load, save and edit serialised message to parameter.
 **/

namespace sweetie_bot {
namespace tools {

template <class MsgType>
	class ParamMsgLoader
	{
		public:
			ParamMsgLoader(ros::NodeHandle& node);
			~ParamMsgLoader();
			MsgType operator()(const std::string& name) { return getParam(name); }

			bool getParam(const std::string& name, MsgType& msg);
			MsgType getParam(const std::string& name);

			bool setParam(const std::string& name, const MsgType& msg);

			std::vector<std::string> getNames(const std::string& name);
		private:
			ros::NodeHandle node_;
	};


template<class MsgType> 
	ParamMsgLoader<MsgType>::ParamMsgLoader(ros::NodeHandle& node) :
		node_(node)
{
}

template<class MsgType> 
	ParamMsgLoader<MsgType>::~ParamMsgLoader()
	{
	}


template<class MsgType> 
	bool ParamMsgLoader<MsgType>::getParam(const std::string& name, MsgType& msg)
	{
		XmlRpc::XmlRpcValue param;
		if (!node_.hasParam(name)) return false;
		if (!node_.getParam(name, param)) return false;
		if ( param.getType() != XmlRpc::XmlRpcValue::TypeBase64) return false;
		std::vector<char> tmp = param;

		if(tmp.size() == 0) return false;
		boost::shared_array<uint8_t> ibuffer(new uint8_t[tmp.size()]);
		ros::serialization::IStream istream((unsigned char*)&tmp[0], tmp.size());
		try{
			ros::serialization::deserialize(istream, msg);
		}catch(...){
			return false;
		}
		return true;
	}


template<class MsgType>
	MsgType ParamMsgLoader<MsgType>::getParam(const std::string& name)
	{
		MsgType msg;
		getParam(name, msg);
		return msg;
		/*
		   if (!node_.getParam(name, param)) return msg;
		   std::vector<char> tmp = param;

		   if(tmp.size() == 0) return msg;
		   boost::shared_array<uint8_t> ibuffer(new uint8_t[tmp.size()]);
		   ros::serialization::IStream istream((unsigned char*)&tmp[0], tmp.size());
		   ros::serialization::deserialize(istream, msg);
		   return msg;
		   */
	}

template<class MsgType>
	bool ParamMsgLoader<MsgType>::setParam(const std::string& name, const MsgType& msg)
	{
		uint32_t serial_size = ros::serialization::serializationLength(msg);
		boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);

		ros::serialization::OStream ostream(buffer.get(), serial_size);
		ros::serialization::serialize(ostream, msg);

		XmlRpc::XmlRpcValue param((unsigned char*)&buffer[0], serial_size);
		node_.setParam( name, param);
		return true;
	}

template<class MsgType>
	std::vector<std::string> ParamMsgLoader<MsgType>::getNames(const std::string& name)
	{
		std::vector<std::string> param_list;
		XmlRpc::XmlRpcValue param;
		if (node_.getParam(name, param))
		{
			for(auto &i: param)
			{
				if(i.second.getType() == XmlRpc::XmlRpcValue::TypeBase64){
					param_list.push_back(i.first);
				}
			}
		}
		return param_list;
	}


} // namespace sweetiebot
} // namespace tools

#endif // PARAMMSG_H
