//======================================================================
/*! \file CommandMiniLuxSetParameter.cpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Apr 10, 2015
 *///-------------------------------------------------------------------
//======================================================================

#include <ibeosdk/datablocks/commands/CommandMiniLuxSetParameter.hpp>

#include <ibeosdk/io.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================

CommandMiniLuxSetParameter::CommandMiniLuxSetParameter(const ParameterIndex parameterIndex,
                                               const ParameterData parameterData)
  : MiniLuxCommand<CommandId::CmdLuxSetParameter>(),
    m_reserved(0x0000),
    m_parameterIndex(parameterIndex),
    m_parameterData(parameterData)
{}

//======================================================================

bool CommandMiniLuxSetParameter::deserialize(std::istream& is, const IbeoDataHeader& dh)
{
	const std::istream::pos_type startPos = is.tellg();

	readLE(is, m_commandId);
	readLE(is, m_reserved);
	m_parameterIndex.readLE(is);
	m_parameterData.readLE(is);

	return !is.fail()
	       && ((is.tellg() - startPos) == this->getSerializedSize())
	       && this->getSerializedSize() == dh.getMessageSize();
}

//======================================================================

bool CommandMiniLuxSetParameter::serialize(std::ostream& os) const
{
	const std::istream::pos_type startPos = os.tellp();

	writeLE(os, m_commandId);
	writeLE(os, m_reserved);
	m_parameterIndex.writeLE(os);
	m_parameterData.writeLE(os);

	return !os.fail() && ((os.tellp() - startPos) == this->getSerializedSize());
}

//======================================================================

}// namespace ibeosdk

//======================================================================
