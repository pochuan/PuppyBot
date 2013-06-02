// *******************************************************************
// PrTransform.h
//
// This class provides an affine transform [R, P], where R is a
// rotation and P is a translation.
//
// modification history
// --------------------
//
// 06/21/04: Dan Merget: Added comments, made compatible with new
//                       PrMatrix6 and PrVector6 structures
// 11/20/97: K.C. Chang: added inline methods.
// 11/05/97: K.C. Chang: created.
// *******************************************************************

#include "PrTransform.h"

// ===================================================================
// multiply():  Combine two transformations into one, using the rule
// [r1, p1] * [r2, p2] = [r1*r2, r1*p2 + p1]
// ===================================================================
void PrTransform::multiply( const PrTransform& rhs, PrTransform& dest ) const
{
  m_rot.multiply( rhs.m_rot, dest.m_rot );
  m_rot.multiply( rhs.m_trans, dest.m_trans );
  dest.m_trans += m_trans;
}

// ===================================================================
// multiply():  Apply a transformation to a 3x1 vector, using the rule
// [r, p] * v = r*v + p
// ===================================================================
void PrTransform::multiply( const PrVector& rhs, PrVector& dest ) const
{
  SAIAssert( rhs.size() == 3 );
  dest.setSize( 3 );

  m_rot.multiply( rhs, dest );
  dest += m_trans;
}

// ===================================================================
// inverse():  Find the inverse of a transformation, using the rule
// ~[r, p] = [~r, -(~r * p)]
// ===================================================================
void PrTransform::inverse( PrTransform& dest ) const
{
  m_rot.inverse( dest.m_rot );
  dest.m_rot.multiply( -m_trans, dest.m_trans );
}

// ===================================================================
// error(): Given a desired position/orientation, find the 6-element
// vector that contains the positional error & instantaneous angular
// error between this frame and the desired frame.
//
// dest = this - Td
// ===================================================================
void PrTransform::error( const PrTransform& Td, PrVector& dest )
{
  dest.setSize( 6 );
  dest.setSubvector( 0, m_trans - Td.m_trans );
  dest.setSubvector( 3, m_rot.angularError( Td.m_rot ) );
}

// ===================================================================
// Xform(): Transform a force/moment vector from frame i to j, given
// a transformation from i to j.  Equivalent to multiplying by the
// matrix X:
//
//           [ R           | 0 ]
//           [ P.cross * R | R ] 
// ===================================================================
void PrTransform::Xform( const PrVector6& rhs, PrVector6& dest ) const
{
  m_rot.multiply( rhs.linearPart(), dest.linearPart() );
  m_trans.cross( dest.linearPart(), dest.angularPart() );
  dest.angularPart() += m_rot * rhs.angularPart();
}

void PrTransform::Xform( const PrVector& rhs, PrVector& dest ) const
{
  PrVector6 tmpRhs(rhs);
  PrVector6 tmpDest;
  Xform( tmpRhs, tmpDest );
  dest = tmpDest;
}

// ===================================================================
// XformT(): Transform a linear/angular velocity vector from frame j
// to i, given a transformation from i to j.  Equivalent to
// multiplying by the transpose of the matrix X given in Xform():
//
//           [ R^T | -R^T * P.cross ] 
//           [ 0   | R^T            ]
// ===================================================================
void PrTransform::XformT( const PrVector6& rhs, PrVector6& dest ) const
{
  ( ~m_rot ).multiply( rhs.linearPart() - m_trans.cross( rhs.angularPart() ),
                       dest.linearPart() );
  ( ~m_rot ).multiply( rhs.angularPart(), dest.angularPart() );
}

void PrTransform::XformT( const PrVector& rhs, PrVector& dest ) const
{
  PrVector6 tmpRhs(rhs);
  PrVector6 tmpDest;
  Xform( tmpRhs, tmpDest );
  dest = tmpDest;
}

// ===================================================================
// display(): Display the transform
// ===================================================================
void PrTransform::display( const char* name ) const
{
  if( name != NULL )
  {
    printf("%s =\n", name );
  }

  printf("[rot=");
  m_rot.display();
  printf(",\n trans=");
  m_trans.display();
  printf("]\n");
}
