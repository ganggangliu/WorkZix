#ifndef __SOPAS_LOGIN_LEVEL_H__
#define __SOPAS_LOGIN_LEVEL_H__

/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "SopasBasicTypes.h"

/*======================================================================================*/
/*! \brief Provides login level and password for CSopasInterface::Login()

    \ingroup common
   
    This class is used to define login levels including passwords to log in to a device.
    There is a namespace SopasLoginLevel defined here which contains the default 
    passwords for following levels:
    - SopasLoginLevel::RUN
    - SopasLoginLevel::MAINTENANCE
    - SopasLoginLevel::AUTHORIZEDCLIENT    

    You can create an own global instance of CSopasLoginLevel in the SopasLoginLevel 
    namespace. You have to do this when a password for a certain level has been changed.

    Also the password hash for the <em>SERVICE</em> level is not supplied here. 
    If you've got the according password hash from SICK you can add it here, too.

    \note Only the password hashes are stored here. They are transmitted to the device.

    \section Example
    If you changed the password for the authorized client you could add it like this:

    <em>File: MyLogin.h</em>
    \code
#include "SopasLoginLevel.h"

namespace SopasLoginLevel
{
  extern const ::CSopasLoginLevel MY_AUTHROIZED_CLIENT;
}
    \endcode

    The definition of the global object is done here:
    <em>File: MyLogin.cpp</em>
    \code
namespace SopasLoginLevel
{
  const ::CSopasLoginLevel MY_AUTHORIZED_CLIENT(3, 0x12345678); 
    // Level 3 is authorized client
    // The hash value needs to be calculated out of your password. Contact SICK for details.
}
    \endcode

    And it is used like this:
    \code
#include "MyLogin.h"

void TestLogin(CSopasInterface &rSopas)
{
  rSopas.Login(SopasLoginLevel::MY_AUTHORIZED_CLIENT);
}
    \endcode
 */
/*======================================================================================*/
class CSopasLoginLevel
{
public:
  CSopasLoginLevel(SOPAS_Int8 level, SOPAS_UInt32 passwordHash);
    //!< Constructs a pair of login level and password hash.

  SOPAS_Int8 GetLevel(void) const;
    //!< Gets numerical login userlevel stored in this class.
  SOPAS_UInt32 GetPasswordHash(void) const;
    //!< Gets password hash stored in this class.

private:
  CSopasLoginLevel(void);
    //!< Explicitly disallow default constructor.
  CSopasLoginLevel(const CSopasLoginLevel&);
    //!< Do not allow copying (this class is used as a const tag)
  CSopasLoginLevel& operator=(const CSopasLoginLevel&);
    //!< Do not allow copying (this class is used as a const tag)

  const SOPAS_Int8 m_Level;
    //!< Numerical SOPAS userlevel used for login.
  const SOPAS_UInt32 m_PasswordHash;
    //!< Hashvalue of password to according level.
};

/*======================================================================================*/
/*! \brief Contains default login levels

    \ingroup common
   
    The login levels are constant instances of the CSopasLoginLevel class.
    The following default passwords are stored here:
    - SopasLoginLevel::RUN
    - SopasLoginLevel::MAINTENANCE
    - SopasLoginLevel::AUTHORIZEDCLIENT    
 */
/*======================================================================================*/
namespace SopasLoginLevel
{
  // Declare global objects (defined in SopasLoginLevel.cpp)
  extern const ::CSopasLoginLevel RUN;
    //!< Login to level RUN
  extern const ::CSopasLoginLevel MAINTENANCE;
    //!< Login to level MAINTENANCE
  extern const ::CSopasLoginLevel AUTHORIZEDCLIENT;
    //!< Login to level AUTHORIZEDCLIENT
}

#endif