//======================================================================
/*! \file Unconvertable.hpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Aug 29, 2013
 *///-------------------------------------------------------------------

#ifndef IBEOSDK_UNCONVERTABLE_HPP_SEEN
#define IBEOSDK_UNCONVERTABLE_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================
/*!\class Unconvertable
 * \brief A wrapper for classes to prohibit implicit type casting in generation.
 * \author Jan Christian Dittmer (jcd)
 * \version 0.1
 * \date Jul 21, 2011
 *
 * \tparam T  class to be wrapped.
 *///-------------------------------------------------------------------
template<class T>
class Unconvertable {
public:
	//========================================
	/*!\brief Constructor to create an Unconvertable from an
	 *        instance of the wrapped class. (explicit)
	 *
	 * This template class is an instrument to disable
	 * implicit compiler type casting.
	 *
	 * Casting from the class T to Unconvertable<T> is not
	 * allowed implicitly but only by calling the constructor
	 * explicitly.
	 *
	 * Casting from Unconvertable<T> to T is allowed implicitly
	 * by defining the cast operator.
	 *
	 * This class, e.g. can be used to define several index types
	 * (derived from integer) and you can't mix them up by accident
	 * by implicit type casting since the constructor taking an int (T)
	 * is declared explicit.
	 *
	 * \param[in] t  Object of the to be wrapped class to setup
	 *               an Unconvertable.
	*///-------------------------------------
	explicit Unconvertable(const T t) : m_data(t) {}


	//========================================
	/*!\brief Type cast operator to wrapped class.
	 * Automatic cast to the wrapped class is allowed.
	 * The other way around (via constructor) is not
	 * allowed automatically/implicitly.
	 *///-------------------------------------
	operator T const () const { return this->m_data; }

public:
	bool operator==(const T t) const { return m_data == t.m_data; }
	bool operator!=(const T t) const { return m_data != t.m_data; }

protected:
	//========================================
	/*!\brief Wrapped object.
	 *///-------------------------------------
	T m_data;
}; // Unconvertable

//======================================================================
//======================================================================
//======================================================================




//======================================================================
/*!\class ComparableUnconvertable
 * \brief A wrapper for classes to prohibit implicit type casting in generation.
 *    For types that have a lesser than operator. (explicit)
 * \author Jan Christian Dittmer (jcd)
 * \version 0.1
 * \date Jul 21, 2011
 *
 * \tparam T  class to be wrapped.
 *///-------------------------------------------------------------------
template<class T>
class ComparableUnconvertable {
public:
	//========================================
	/*!\brief Constructor to create an ComparableUnconvertable from an
	 *        instance of the wrapped class.
	 *
	 * This template class is an instrument to disable
	 * implicit compiler type casting.
	 *
	 * Casting from the class T to ComparableUnconvertable<T> is not
	 * allowed implicitly but only by calling the constructor
	 * explicitly.
	 *
	 * Casting from ComparableUnconvertable<T> to T is allowed implicitly
	 * by defining the cast operator.
	 *
	 * The only difference to class Unconvertable is that there is
	 * a lesser than operator available.
	 *
	 * This class, e.g. can be used to define several index types
	 * (derived from integer) and you can't mix them up by accident
	 * by implicit type casting since the constructor taking an int (T)
	 * is declared explicit.
	 *
	  * \param[in] t  Object of the to be wrapped class to setup
	  *               an ComparableUnconvertable.
	  *///-------------------------------------
	explicit ComparableUnconvertable(const T t) : m_data(t) {}

	//========================================
	/*!\brief Type cast operator to wrapped class.
	 * Automatic cast to the wrapped class is allowed.
	 * The other way around (via constructor) is not
	 * allowed automatically/implicitly.
	 *///-------------------------------------
	operator T const () const { return this->m_data; }

	//========================================
	/*!\brief Lesser than operator for ComparableUnconvertable.
	 *
	 * \param[in] l  Left hand operand of <.
	 * \param[in] r  Light hand operand of <.
	 *
	 * \return \c true if \a l<\a r, \c false otherwise.
	 *///-------------------------------------
	friend bool operator<(const ComparableUnconvertable l, const ComparableUnconvertable r) { return l.m_data < r.m_data; }

	//========================================
	/*!\brief Lesser than operator for ComparableUnconvertable and its base type.
	 *
	 * \param[in] l  Left hand operand of <.
	 * \param[in] r  Light hand operand of <.
	 *
	 * \return \c true if \a l<\a r, \c false otherwise.
	 *///-------------------------------------
	friend bool operator<(const ComparableUnconvertable l, const T r) { return l.m_data < r; }

	//========================================
	/*!\brief Lesser than operator for ComparableUnconvertable and its base type.
	 *
	 * \param[in] l  Left hand operand of <.
	 * \param[in] r  Light hand operand of <.
	 *
	 * \return \c true if \a l<\a r, \c false otherwise.
	 *///-------------------------------------
	friend bool operator<(const T l, const ComparableUnconvertable r) { return l < r.m_data; }

	//========================================
	/*!\brief Greater than operator for ComparableUnconvertable.
	 *
	 * \param[in] l  Left hand operand of >.
	 * \param[in] r  Light hand operand of >.
	 *
	 * \return \c true if \a l>\a r, \c false otherwise.
	 *///-------------------------------------
	friend bool operator>(const ComparableUnconvertable l, const ComparableUnconvertable r) { return l.m_data > r.m_data; }

	//========================================
	/*!\brief Greater than operator for ComparableUnconvertable and its base type.
	 *
	 * \param[in] l  Left hand operand of >.
	 * \param[in] r  Light hand operand of >.
	 *
	 * \return \c true if \a l>\a r, \c false otherwise.
	 *///-------------------------------------
	friend bool operator>(const ComparableUnconvertable l, const T r) { return l.m_data > r; }

	//========================================
	/*!\brief Greater than operator for ComparableUnconvertable and its base type.
	 *
	 * \param[in] l  Left hand operand of >.
	 * \param[in] r  Light hand operand of >.
	 *
	 * \return \c true if \a l>\a r, \c false otherwise.
	 *///-------------------------------------
	friend bool operator>(const T l, const ComparableUnconvertable r) { return l > r.m_data; }

protected:
	T m_data;
}; // ComparableUnconvertable

//======================================================================

} // namespace ibeosdk

//======================================================================

#endif // IBEOSDK_UNCONVERTABLE_HPP_SEEN

//======================================================================

