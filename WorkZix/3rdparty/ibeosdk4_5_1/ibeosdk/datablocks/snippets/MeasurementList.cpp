//======================================================================
/*! \file ObjectEcuEt.cpp
 *
 * \copydoc Copyright
 * \author Kai-Uwe von Deylen (kd)
 * \date Mar 14, 2014
 *///-------------------------------------------------------------------

#include <ibeosdk/datablocks/snippets/MeasurementList.hpp>
#include <ibeosdk/io.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================

MeasurementList::MeasurementList()
  : m_measurements()
{}

//======================================================================

std::streamsize MeasurementList::getSerializedSize() const
{
	std::streamsize result = std::streamsize(sizeof(UINT16));
	std::vector<Measurement>::const_iterator mIter = m_measurements.begin();
	for (; mIter != m_measurements.end(); ++mIter) {
		result += mIter->getSerializedSize();
	}
	return result;
}
//======================================================================

bool MeasurementList::deserialize(std::istream& is)
{
	const std::istream::pos_type startPos = is.tellg();

	UINT16 size;
	readBE(is, size);
	m_measurements.resize(size);

	std::vector<Measurement>::iterator mIter = m_measurements.begin();
	for (; mIter != m_measurements.end(); ++mIter) {
		if (!mIter->deserialize(is))
			return false;
	}

	return !is.fail() && ((is.tellg() - startPos) == this->getSerializedSize());
}

//======================================================================

bool MeasurementList::serialize(std::ostream& os) const
{
	const std::istream::pos_type startPos = os.tellp();

	writeBE(os, UINT16(m_measurements.size()));
	std::vector<Measurement>::const_iterator mIter = m_measurements.begin();
	for (; mIter != m_measurements.end(); ++mIter) {
		if (!mIter->serialize(os))
			return false;
	}

	return !os.fail() && ((os.tellp() - startPos) == this->getSerializedSize());
}

//======================================================================

} // namespace ibeosdk

//======================================================================
