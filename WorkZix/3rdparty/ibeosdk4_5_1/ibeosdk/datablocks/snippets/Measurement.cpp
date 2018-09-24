//======================================================================
/*! \file ObjectEcuEt.cpp
 *
 * \copydoc Copyright
 * \author Kai-Uwe von Deylen (kd)
 * \date Mar 14, 2014
 *///-------------------------------------------------------------------

#include <ibeosdk/datablocks/snippets/Measurement.hpp>
#include <ibeosdk/io.hpp>

#include <boost/scoped_array.hpp>

#include <stdexcept>
#include <sstream>

//======================================================================

namespace ibeosdk {

//======================================================================

Measurement::Measurement()
 :  m_key(0),
    m_measurementType(TypeVoid),
    m_data(0)
{}

//======================================================================

std::streamsize Measurement::getSerializedSize() const
{
	std::streamsize result = std::streamsize(sizeof(UINT16)) // m_key
	                       + std::streamsize(sizeof(UINT8)); // m_measurementType

	switch (m_measurementType) {
	case TypeVoid:   return result;
	case TypeFloat:  return result + std::streamsize(sizeof(float));
	case TypeDouble: return result + std::streamsize(sizeof(double));
	case TypeINT8:
	case TypeUINT8:  return result + std::streamsize(sizeof(UINT8));
	case TypeINT16:
	case TypeUINT16: return result + std::streamsize(sizeof(UINT16));
	case TypeINT32:
	case TypeUINT32: return result + std::streamsize(sizeof(UINT32));
	case TypeINT64:
	case TypeUINT64: return result + std::streamsize(sizeof(UINT64));
	case TypeBool:   return result + std::streamsize(sizeof(UINT8));
	case TypeStdString:
		result += 4; // 4 bytes for the BLOB data length
		result += std::streamsize(boost::any_cast<std::string>(m_data).size()); // and the BLOB data itself
		break;
	default:
		throw std::runtime_error("Unknown MeasurementType in Measurement");
	} // switch
	return result;
}

//======================================================================

bool Measurement::deserialize(std::istream& is)
{
	const std::istream::pos_type startPos = is.tellg();

	union allTypes {
		float flt;
		double dbl;
		int8_t int8;
		uint8_t uint8;
		int16_t int16;
		uint16_t uint16;
		int32_t int32;
		uint32_t uint32;
		int64_t int64;
		uint64_t uint64;
		bool boolean;
	};

	allTypes tmp;

	readBE(is, m_key);

	UINT8 type;
	readBE(is, type);
	m_measurementType = MeasurementType(type);

	switch (m_measurementType) {
	case TypeVoid:  break;
	case TypeFloat:  readBE(is, tmp.flt);     m_data = tmp.flt;     break;
	case TypeDouble: readBE(is, tmp.dbl);     m_data = tmp.dbl;     break;
	case TypeINT8:   readBE(is, tmp.int8);    m_data = tmp.int8;    break;
	case TypeUINT8:  readBE(is, tmp.uint8);   m_data = tmp.uint8;   break;
	case TypeINT16:  readBE(is, tmp.int16);   m_data = tmp.int16;   break;
	case TypeUINT16: readBE(is, tmp.uint16);  m_data = tmp.uint16;  break;
	case TypeINT32:  readBE(is, tmp.int32);   m_data = tmp.int32;   break;
	case TypeUINT32: readBE(is, tmp.uint32);  m_data = tmp.uint32;  break;
	case TypeINT64:  readBE(is, tmp.int64);   m_data = tmp.int64;   break;
	case TypeUINT64: readBE(is, tmp.uint64);  m_data = tmp.uint64;  break;
	case TypeBool:   readBE(is, tmp.boolean); m_data = tmp.boolean; break;
	case TypeStdString:
		{
			UINT32 strLen;
			readBE(is, strLen);
			boost::scoped_array<char> strBuf (new char[strLen]);
			is.read(strBuf.get(), strLen);
			m_data = std::string(strBuf.get(), strLen);
		}
		break;
	default:
		throw std::runtime_error("Unknown MeasurementType in Measurement");
	} // switch

	return !is.fail() && ((is.tellg() - startPos) == this->getSerializedSize());
}

//======================================================================

bool Measurement::serialize(std::ostream& os) const
{
	const std::istream::pos_type startPos = os.tellp();

	writeBE(os, m_key);

	UINT8 type = UINT8(m_measurementType);
	writeBE(os, type);

	switch (m_measurementType) {
	case TypeVoid:   break;
	case TypeFloat:  writeBE(os, boost::any_cast<float>(m_data));  break;
	case TypeDouble: writeBE(os, boost::any_cast<double>(m_data)); break;
	case TypeINT8:   writeBE(os, boost::any_cast<INT8>(m_data));   break;
	case TypeUINT8:  writeBE(os, boost::any_cast<UINT8>(m_data));  break;
	case TypeINT16:  writeBE(os, boost::any_cast<INT16>(m_data));  break;
	case TypeUINT16: writeBE(os, boost::any_cast<UINT16>(m_data)); break;
	case TypeINT32:  writeBE(os, boost::any_cast<INT32>(m_data));  break;
	case TypeUINT32: writeBE(os, boost::any_cast<UINT32>(m_data)); break;
	case TypeINT64:  writeBE(os, boost::any_cast<INT64>(m_data));  break;
	case TypeUINT64: writeBE(os, boost::any_cast<UINT64>(m_data)); break;
	case TypeBool:   writeBE(os, boost::any_cast<bool>(m_data));   break;
	case TypeStdString:
		writeBE(os, UINT32(boost::any_cast<std::string>(m_data).size()));
		os.write(boost::any_cast<std::string>(m_data).c_str(), std::streamsize(boost::any_cast<std::string>(m_data).size()));
		break;
	default:
		throw std::runtime_error("Unknown MeasurementType in Measurement");
	} // switch

	return !os.fail() && ((os.tellp() - startPos) == this->getSerializedSize());
}

//======================================================================

std::ostream& operator<<(std::ostream& oss, const Measurement& m)
{
	oss << "Key=" << m.getKey() << " type=" << m.getMeasurementType() << " ";

	switch(m.getMeasurementType()) {
	case Measurement::TypeFloat:     oss << "TypeFloat(" << m.getData<float>() << ")";           break;
	case Measurement::TypeDouble:    oss << "TypeDouble(" << m.getData<double>() << ")";         break;
	case Measurement::TypeINT8:      oss << "TypeINT8(" << int(m.getData<INT8>()) << ")";        break;
	case Measurement::TypeUINT8:     oss << "TypeUINT8(" << int(m.getData<UINT8>()) << ")";      break;
	case Measurement::TypeINT16:     oss << "TypeINT16(" << m.getData<INT16>() << ")";           break;
	case Measurement::TypeUINT16:    oss << "TypeUINT16(" << m.getData<UINT16>() << ")";         break;
	case Measurement::TypeINT32:     oss << "TypeINT32(" << m.getData<INT32>() << ")";           break;
	case Measurement::TypeUINT32:    oss << "TypeUINT32(" << m.getData<UINT32>() << ")";         break;
	case Measurement::TypeINT64:     oss << "TypeINT64(" << m.getData<INT64>() << ")";           break;
	case Measurement::TypeUINT64:    oss << "TypeUINT64(" << m.getData<UINT64>() << ")";         break;
	case Measurement::TypeBool:      oss << "TypeBool(" << m.getData<bool>() << ")";             break;
	case Measurement::TypeStdString: oss << "TypeStdString(" << m.getData<std::string>() << ")"; break;
	default:
		oss << "Undefined Type(..) ";
		break;
	}

	return oss;
}

//======================================================================

} // namespace ibeosdk

//======================================================================
