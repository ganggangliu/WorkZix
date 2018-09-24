/*
==========================================================================================
Copyright (c) SICK AG - Division Auto Ident, 2008

==========================================================================================
*/

#include "SopasLoginLevel.h"

CSopasLoginLevel::CSopasLoginLevel(SOPAS_Int8 level, SOPAS_UInt32 passwordHash) :
  m_Level(level), m_PasswordHash(passwordHash)
{
}

SOPAS_Int8 CSopasLoginLevel::GetLevel(void) const
{
  return m_Level;
}

SOPAS_UInt32 CSopasLoginLevel::GetPasswordHash(void) const
{
  return m_PasswordHash;
}

namespace SopasLoginLevel
{
  const ::CSopasLoginLevel RUN(1, 0x00000000);
  const ::CSopasLoginLevel MAINTENANCE(2, 0x7692C570);
  const ::CSopasLoginLevel AUTHORIZEDCLIENT(3, 0x7A99FDC6);
}