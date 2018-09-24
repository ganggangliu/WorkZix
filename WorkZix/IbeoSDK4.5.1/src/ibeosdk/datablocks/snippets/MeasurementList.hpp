//======================================================================
/*! \file MeasurementList.hpp
 *
 * \copydoc Copyright
 * \author Kai-Uwe von Deylen (kd)
 * \date Mar 14, 2014
 *///-------------------------------------------------------------------

#ifndef IBEOSDK_MEASUREMENTLIST_HPP_SEEN
#define IBEOSDK_MEASUREMENTLIST_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/inttypes.hpp>
#include <ibeosdk/datablocks/snippets/Measurement.hpp>

#include <ibeosdk/datablocks/snippets/Snippet.hpp>
#include <ibeosdk/misc/deprecatedwarning.hpp>

#include <vector>
#include <iostream>

//======================================================================

namespace ibeosdk {

//======================================================================

class MeasurementList : public ibeosdk::Snippet {

public:
	MeasurementList();

public:
	virtual std::streamsize getSerializedSize() const;
	virtual bool deserialize(std::istream& is);
	virtual bool serialize(std::ostream& os) const;

public:
	void addMeasurement(const Measurement& meas) { m_measurements.push_back( meas ); }

public: // getter
	UINT16 getSize() const { return UINT16(m_measurements.size()); }

	const std::vector<Measurement>& getMeasurements() const { return m_measurements; }
	std::vector<Measurement>& getMeasurements() { return m_measurements; }

public: // setter
	void setMeasurements(const std::vector<Measurement>& measurements) { m_measurements = measurements; }

protected:
	std::vector<Measurement> m_measurements;
}; // MeasurementList

//======================================================================

}// namespace ibeosdk

//======================================================================

#endif // IBEOSDK_MEASUREMENTLIST_HPP_SEEN

//======================================================================

