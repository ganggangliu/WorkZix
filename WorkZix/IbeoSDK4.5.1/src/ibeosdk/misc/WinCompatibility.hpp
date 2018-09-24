//======================================================================
/*! \file WinCompatibility.hpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Oct 2, 2013
 *///-------------------------------------------------------------------

#ifndef IBEOSDK_WINCOMPATIBILITY_HPP_SEEN
#define IBEOSDK_WINCOMPATIBILITY_HPP_SEEN

//======================================================================

#ifdef _WIN32
#	ifndef _USE_MATH_DEFINES
#		define _USE_MATH_DEFINES
#	endif // _USE_MATH_DEFINES

#	ifndef NOMINMAX
#		define NOMINMAX
#	endif // NOMINMAX

#	define __func__ __FUNCTION__

#	ifndef __BIG_ENDIAN
#		define __BIG_ENDIAN 1234
#	endif // __BIG_ENDIAN

#	ifndef __LITTLE_ENDIAN
#		define __LITTLE_ENDIAN 3412
#	endif // __LITTLE_ENDIAN

#endif // _WIN32

//======================================================================

#endif // IBEOSDK_WINCOMPATIBILITY_HPP_SEEN

//======================================================================
