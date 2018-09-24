//==============================================================================
/**
 * \file CarriageWayList.hpp
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

#ifndef IBEOSDK_CARRIAGEWAYLIST_HPP
#define IBEOSDK_CARRIAGEWAYLIST_HPP

//==============================================================================

#include <ibeosdk/datablocks/snippets/CarriageWay.hpp>

#include <ibeosdk/LaneType.hpp>

//==============================================================================

namespace ibeosdk{
namespace lanes{

//==============================================================================
/*! \brief A list type for storing multiple elements of type \ref CarriageWay.
 * Inherited from std::vector.
 * \sa CarriageWay
 *///---------------------------------------------------------------------------
class CarriageWayList : public std::vector<CarriageWayPtr>,  public ibeosdk::Snippet {
public:
	/*! \brief default constructor */
	CarriageWayList(){}

public:
	virtual std::streamsize getSerializedSize() const;
	virtual bool deserialize(std::istream& is);
	virtual bool serialize(std::ostream& os) const;
}; // CarriageWayList

//=============================================================================

} // namespace ibeosdk
} // namespace lanes

//=============================================================================

#endif // IBEOSDK_CARRIAGEWAYLIST_HPP

//=============================================================================
