//======================================================================
/*! \file Measurement.hpp
 *
 * \copydoc Copyright
 * \author Kai-Uwe von Deylen (kd)
 * \date Mar 14, 2014
 *///-------------------------------------------------------------------

#ifndef IBEOSDK_MEASUREMENT_HPP_SEEN
#define IBEOSDK_MEASUREMENT_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/MeasurementKey.hpp>

#include <ibeosdk/inttypes.hpp>

#include <ibeosdk/datablocks/snippets/Snippet.hpp>
#include <ibeosdk/misc/deprecatedwarning.hpp>

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/any.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================

class Measurement : public ibeosdk::Snippet {

public:
	enum MeasurementType {
		TypeVoid      = 0x00,
		TypeFloat     = 0x01,
		TypeDouble    = 0x02,
		TypeINT8      = 0x03,
		TypeUINT8     = 0x04,
		TypeINT16     = 0x05,
		TypeUINT16    = 0x06,
		TypeINT32     = 0x07,
		TypeUINT32    = 0x08,
		TypeINT64     = 0x09,
		TypeUINT64    = 0x0A,
		TypeBool      = 0x0B,
		TypeStdString = 0x0C
	};

public:
	Measurement();

public:
	virtual std::streamsize getSerializedSize() const;
	virtual bool deserialize(std::istream& is);
	virtual bool serialize(std::ostream& os) const;

public: // getter
	MeasurementKey getKey() const { return m_key; }
	MeasurementType getMeasurementType() const { return m_measurementType; }

	template<typename T>
	T getData() const { return boost::any_cast<T>(m_data); }

public: // setter
	void setKey(const MeasurementKey key) { m_key = key; }
	void setMeasurementType(const MeasurementType type) { m_measurementType = type; }
	template<typename T> void setData(const T& data) { m_data = data; }

protected:
	MeasurementKey m_key;
	MeasurementType m_measurementType;
	boost::any m_data;
}; // Measurement

//======================================================================

std::ostream& operator<<(std::ostream& oss, const Measurement& m);

//======================================================================

}// namespace ibeosdk

//======================================================================

#endif // IBEOSDK_MEASUREMENT_HPP_SEEN

//======================================================================

