//==============================================================================
/**
 * \file CarriageWayList.cpp
 * \brief CarriageWayList which is used for storing the map and providing
 * suitable interface for source and drain
 *
 * -----------------------------------------------------------------------------
 * \copyright &copy; 2014 Ibeo Automotive Systems GmbH, Hamburg, Germany
 *
 * \date   Oct 9, 2014
 * \author Stefan Kaufmann (stk)
 */
//==============================================================================

#include <ibeosdk/datablocks/snippets/CarriageWayList.hpp>

namespace ibeosdk {
namespace lanes{

//=============================================================================

bool CarriageWayList::serialize(std::ostream& os) const
{
	const std::istream::pos_type startPos = os.tellp();

	ibeosdk::writeBE(os, static_cast<UINT64>(size() ));      // 8

	for(CarriageWayList::const_iterator it = begin(); it != end(); it++)
		(*it)->serialize(os);

	return !os.fail() && ((os.tellp() - startPos) == this->getSerializedSize());
}

//=============================================================================

std::streamsize CarriageWayList::getSerializedSize() const
{
	std::streamsize s_size = 0;

	for(CarriageWayList::const_iterator it = begin(); it != end(); it++)
		s_size += (*it)->getSerializedSize();

	return 8 + s_size;
}

//=============================================================================

bool CarriageWayList::deserialize(std::istream& is)
{
	const std::istream::pos_type startPos = is.tellg();

	clear();
	UINT64 nWays;

	ibeosdk::readBE(is, nWays);

	for(UINT64 i = 0; i < nWays; i++)
	{
		CarriageWayPtr way = CarriageWay::create();
		way->deserialize(is);
		way->resolveConnections(way);
		push_back(way);
	}

	return !is.fail() && ((is.tellg() - startPos) == this->getSerializedSize());
}

//=============================================================================

}
}
