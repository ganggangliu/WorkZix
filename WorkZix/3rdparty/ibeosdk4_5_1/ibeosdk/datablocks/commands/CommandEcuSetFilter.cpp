//======================================================================
/*! \file CommandEcuSetFilter.cpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Apr 7, 2015
 *///-------------------------------------------------------------------

//======================================================================

#include <ibeosdk/datablocks/commands/CommandEcuSetFilter.hpp>

#include <ibeosdk/io.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================

CommandEcuSetFilter::CommandEcuSetFilter(const DataTypeId rangeStart,
                                         const DataTypeId rangeEnd)
  : EcuCommand<CommandId::CmdManagerSetFilter>(),
    m_version(version),
    m_rangeStart(rangeStart),
    m_rangeEnd(rangeEnd)
{}

//======================================================================

bool CommandEcuSetFilter::deserialize(std::istream& is, const IbeoDataHeader& dh)
{
	const std::istream::pos_type startPos = is.tellg();

	readBE(is, m_commandId);
	readBE(is, m_version);
	m_rangeStart.readBE(is);
	m_rangeEnd.readBE(is);

	return !is.fail()
	       && ((is.tellg() - startPos) == this->getSerializedSize())
	       && this->getSerializedSize() == dh.getMessageSize();
}

//======================================================================

bool CommandEcuSetFilter::serialize(std::ostream& os) const
{
	const std::istream::pos_type startPos = os.tellp();

	writeBE(os, m_commandId);
	writeBE(os, m_version);
	m_rangeStart.writeBE(os);
	m_rangeEnd.writeBE(os);

	return !os.fail() && ((os.tellp() - startPos) == this->getSerializedSize());
}

//======================================================================

} // namespace ibeosdk

//======================================================================
