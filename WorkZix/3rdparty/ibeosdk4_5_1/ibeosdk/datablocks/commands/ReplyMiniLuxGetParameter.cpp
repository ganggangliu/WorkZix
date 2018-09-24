//======================================================================
/*! \file ReplyMiniLuxGetParameter.cpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Apr 10, 2015
 *///-------------------------------------------------------------------
//======================================================================

#include <ibeosdk/datablocks/commands/ReplyMiniLuxGetParameter.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================

ReplyMiniLuxGetParameter::ReplyMiniLuxGetParameter()
  : MiniLuxCommandReply<CommandId::CmdLuxGetParameter>(),
    m_parameterIndex(ParameterIndex(0)),
    m_parameterData(ParameterData(0))
{}

//======================================================================

bool ReplyMiniLuxGetParameter::deserialize(std::istream& is, const IbeoDataHeader& dh)
{
	const std::istream::pos_type startPos = is.tellg();

	readLE(is, m_commandId);
	m_parameterIndex.readLE(is);
	m_parameterData.readLE(is);

	return !is.fail()
	       && ((is.tellg() - startPos) == this->getSerializedSize())
	       && this->getSerializedSize() == dh.getMessageSize();
}

//======================================================================

bool ReplyMiniLuxGetParameter::serialize(std::ostream& os) const
{
	const std::istream::pos_type startPos = os.tellp();

	writeLE(os, m_commandId);
	m_parameterIndex.writeLE(os);
	m_parameterData.writeLE(os);

	return !os.fail() && ((os.tellp() - startPos) == this->getSerializedSize());
}

//======================================================================

} // namespace ibeosdk

//======================================================================
