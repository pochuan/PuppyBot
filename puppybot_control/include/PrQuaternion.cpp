// *******************************************************************
// PrQuaternion.cpp
//
// This implements a quaternion class.  See PrQuaternion.h for
// details.
//
// modification history
// --------------------
//
// 06/17/04: Dan Merget: Added comments
// 11/20/97: K.C. Chang: added inline methods.
// 11/05/97: K.C. Chang: created.
// *******************************************************************

#include "PrQuaternion.h"

#define PR_QUATERNION_EPSILON 0.00001
#define PR_QUATERNION_COS_THRESHHOLD (1.0 - PR_QUATERNION_EPSILON )

// ===================================================================
// values(): Create a rotation quaternion that will rotate unit vector
// vFrom to +vTo or -vTo.
// ===================================================================
void PrQuaternion::values( const PrVector& vFrom, const PrVector& vTo )
{
  SAIAssert( vFrom.size() == 3 );
  SAIAssert( vTo.size() == 3 );

  Float fCosTheta = vFrom.dot( vTo );

  if( fCosTheta > PR_QUATERNION_COS_THRESHHOLD ) 
  {
    // the vectors are the same
    identity();
  }
  else if( fCosTheta < -PR_QUATERNION_COS_THRESHHOLD ) 
  {
    // the vectors are opposing
    identity();
  }
  else
  {
    PrVector3 axis;
    vFrom.cross( vTo, axis );
    axis.normalize();
    values(axis, (Float) acos( fCosTheta ) );
  }
}

// ============================================================================
// values(): Loads quarternion from specified rotation matrix
// (direction cosines).  Converts up to the sign.
// ============================================================================
void PrQuaternion::values( const PrMatrix& m )
{
  SAIAssert( m.row() == 3 && m.column() == 3 );

  double e1 = 1 + m[0][0] - m[1][1] - m[2][2];
  double e2 = 1 - m[0][0] + m[1][1] - m[2][2];
  double e3 = 1 - m[0][0] - m[1][1] + m[2][2];
  double e4 = 1 + m[0][0] + m[1][1] + m[2][2];

  // divide by the greater number; one element is for sure greater than .5
  if( e4 >= 1.0 ) 
  {
    e4          = sqrt( e4 );
    m_scalar    = (Float) (.5 * e4);
    m_vector[0] = (Float) ((m[2][1] - m[1][2]) / (2*e4));  
    m_vector[1] = (Float) ((m[0][2] - m[2][0]) / (2*e4));
    m_vector[2] = (Float) ((m[1][0] - m[0][1]) / (2*e4));
  } 
  else if( e1 >= 1.0 ) 
  {
    e1          = sqrt( e1 );
    m_scalar    = (Float) ((m[2][1] - m[1][2]) / (2*e1));
    m_vector[0] = (Float) (.5 * e1);
    m_vector[1] = (Float) ((m[1][0] + m[0][1]) / (2*e1));
    m_vector[2] = (Float) ((m[2][0] + m[0][2]) / (2*e1));
  } 
  else if( e2 >= 1.0 ) 
  {
    e2          = sqrt( e2 );
    m_scalar    = (Float) ((m[0][2] - m[2][0]) / (2*e2));
    m_vector[0] = (Float) ((m[0][1] + m[1][0]) / (2*e2));
    m_vector[1] = (Float) (.5 * e2);
    m_vector[2] = (Float) ((m[1][2] + m[2][1]) / (2*e2));
  } 
  else if( e3 >= 1.0 ) 
  {
    e3          = sqrt( e3 );
    m_scalar    = (Float) ((m[1][0] - m[0][1]) / (2*e3));
    m_vector[0] = (Float) ((m[2][0] + m[0][2]) / (2*e3));
    m_vector[1] = (Float) ((m[2][1] + m[1][2]) / (2*e3));
    m_vector[2] = (Float) (.5 * e3);
  }
  else
  {
    SAIAssertSz( false, "Renormalization of quarternion needed." );
  }
}

// ===================================================================
// axisAngle():  Convert to axis-angle notation.
// ===================================================================
void PrQuaternion::axisAngle( PrVector& axis, Float& angle ) const
{
  SAIAssert( axis.size() == 3 );

  if( m_scalar > PR_QUATERNION_COS_THRESHHOLD ||
      m_scalar < -PR_QUATERNION_COS_THRESHHOLD )
  {
    // no rotation
    angle   = 0;
    axis[0] = 1;
    axis[1] = 0;
    axis[2] = 0;
  }
  else
  {
    angle = (Float) (2 * acos( m_scalar ));
    Float magnitude = m_vector.magnitude();
    axis = m_vector / magnitude;
  }
}

// ===================================================================
// eulerAngles(): Convert to Euler angles, using the usual
// x-convention (rotate around z-axis, then x-axis, then z-axis).
// ===================================================================
void PrQuaternion::eulerAngles( Float& psi, Float& theta, Float& phi ) const
{
   PrMatrix3 rot;  // Rotation matrix
   rotationMatrix( rot );

   Float cTheta = rot[2][2];
   Float sTheta = (Float) (sqrt( rot[0][2] * rot[0][2] + rot[1][2] * rot[1][2] ));
   if( rot[1][2] > 0 )
   {
      sTheta = -sTheta;   // Choose euler angles such that cos(psi) >= 0
   }

   if( sTheta > PR_QUATERNION_EPSILON )
   {
      // theta > 0
      theta = (Float) atan2( sTheta, cTheta );
      psi = (Float) atan2( rot[0][2], -rot[1][2] );
      phi = (Float) atan2( rot[2][0],  rot[2][1] );
   }
   else if( sTheta < -PR_QUATERNION_EPSILON )
   {
      // theta < 0
      theta = (Float) atan2( sTheta, cTheta );
      psi = (Float) atan2( -rot[0][2],  rot[1][2] );
      phi = (Float) atan2( -rot[2][0], -rot[2][1] );
   }
   else if( cTheta > 0 )
   {
      // theta = 0, which means that we cannot find psi and phi, but
      // only psi+phi.  Split the difference between them.
      theta = 0;
      psi = (Float) atan2( rot[1][0], rot[0][0] ) / 2;
      phi = psi;
   }
   else
   {
      // theta = pi, which means that we cannot find psi and phi, but
      // only psi-phi.  Split the difference between them.
      theta = M_PI;
      psi = (Float) atan2( rot[1][0], rot[0][0] ) / 2;
      phi = -psi;
   }
}

// ===================================================================
// loadFromEulerAngles(): Load the quaternion from Euler angles,
// using the x-convention.
// ===================================================================
void PrQuaternion::loadFromEulerAngles( Float psi, Float theta, Float phi )
{
   PrQuaternion psiRotation(   (Float) cos( psi/2 ),   0, 0,   (Float) sin( psi/2 ) );
   PrQuaternion thetaRotation( (Float) cos( theta/2 ), (Float) sin( theta/2 ), 0, 0 );
   PrQuaternion phiRotation(   (Float) cos( phi/2 ),   0, 0,   (Float) sin( phi/2 ) );

   *this = psiRotation;
   *this *= thetaRotation;
   *this *= phiRotation;
}

// ===================================================================
// eulerAnglesY(): Convert to Euler angles, using the y-convention
// (rotate around z-axis, then x-axis, then z-axis).
// ===================================================================
void PrQuaternion::eulerAnglesY( Float& psi, Float& theta, Float& phi ) const
{
   PrMatrix3 rot;  // Rotation matrix
   rotationMatrix( rot );

   Float cTheta = rot[2][2];
   Float sTheta = (Float) sqrt( rot[0][2] * rot[0][2] + rot[1][2] * rot[1][2] );
   if( rot[0][2] < 0 )
   {
      sTheta = -sTheta;   // Choose euler angles such that cos(psi) >= 0
   }

   if( sTheta > PR_QUATERNION_EPSILON )
   {
      // theta > 0
      theta = (Float) atan2( sTheta, cTheta );
      psi = (Float) atan2( rot[1][2],  rot[0][2] );
      phi = (Float) atan2( rot[2][1], -rot[2][0] );
   }
   else if( sTheta < -PR_QUATERNION_EPSILON )
   {
      // theta < 0
      theta = (Float) atan2( sTheta, cTheta );
      psi = (Float) atan2( -rot[1][2], -rot[0][2] );
      phi = (Float) atan2( -rot[2][1],  rot[2][0] );
   }
   else if( cTheta > 0 )
   {
      // theta = 0, which means that we cannot find psi and phi, but
      // only psi+phi.  Split the difference between them.
      theta = 0;
      psi = (Float) atan2( rot[1][0], rot[0][0] ) / 2;
      phi = psi;
   }
   else
   {
      // theta = pi, which means that we cannot find psi and phi, but
      // only psi-phi.  Split the difference between them.
      theta = M_PI;
      psi = (Float) atan2( -rot[1][0], -rot[0][0] ) / 2;
      phi = -psi;
   }
}

// ===================================================================
// loadFromEulerAnglesY(): Load the quaternion from Euler angles,
// using the y-convention.
// ===================================================================
void PrQuaternion::loadFromEulerAnglesY( Float psi, Float theta, Float phi )
{
   PrQuaternion psiRotation(   (Float) cos( psi/2 ),   0, 0,   (Float) sin( psi/2 ) );
   PrQuaternion thetaRotation( (Float) cos( theta/2 ), 0, (Float) sin( theta/2 ), 0 );
   PrQuaternion phiRotation(   (Float) cos( phi/2 ),   0, 0,   (Float) sin( phi/2 ) );

   *this = psiRotation;
   *this *= thetaRotation;
   *this *= phiRotation;
}

// ===================================================================
// rotationMatrix(): Convert unit quaternion to a 3x3 rotation matrix.
// Requires 27 mult + 12 add operations.
// ===================================================================
void PrQuaternion::rotationMatrix( PrMatrix& dest ) const
{
  dest.setSize( 3, 3 );

  // Column 0
  dest[0][0]=(Float) (1.0 - 2*( m_vector[1] * m_vector[1] + m_vector[2] * m_vector[2] ));
  dest[1][0]=2*( m_vector[0] * m_vector[1] + m_scalar*m_vector[2] );
  dest[2][0]=2*( m_vector[0] * m_vector[2] - m_scalar*m_vector[1] );

  // Column 1
  dest[0][1]=2*( m_vector[0] * m_vector[1] - m_scalar*m_vector[2] );
  dest[1][1]=(Float) (1.0 - 2*( m_vector[0] * m_vector[0] + m_vector[2] * m_vector[2] ));
  dest[2][1]=2*( m_vector[1] * m_vector[2] + m_scalar*m_vector[0] );

  // Column 2
  dest[0][2]=2*( m_vector[0] * m_vector[2] + m_scalar*m_vector[1] );
  dest[1][2]=2*( m_vector[1] * m_vector[2] - m_scalar*m_vector[0] );
  dest[2][2]=(Float) (1.0 - 2*( m_vector[0] * m_vector[0] + m_vector[1] * m_vector[1] ));
}

// ===================================================================
// column(): Similar to rotationMatrix(), but only returns one column.
// ===================================================================
void PrQuaternion::column( int col, PrVector& dest ) const
{
  SAIAssert( col >= 0 && col < 3 );
  dest.setSize( 3 );

  if( col == 0 )
  {
    dest[0]=(Float) (1.0 - 2*( m_vector[1] * m_vector[1] + m_vector[2] * m_vector[2] ));
    dest[1]=2*( m_vector[0] * m_vector[1] + m_scalar*m_vector[2] );
    dest[2]=2*( m_vector[0] * m_vector[2] - m_scalar*m_vector[1] );
  }
  else if( col == 1 )
  {
    dest[0]=2*( m_vector[0] * m_vector[1] - m_scalar*m_vector[2] );
    dest[1]=(Float) (1.0 - 2*( m_vector[0] * m_vector[0] + m_vector[2] * m_vector[2] ));
    dest[2]=2*( m_vector[1] * m_vector[2] + m_scalar*m_vector[0] );
  }
  else // if( col == 2 )
  {
    dest[0]=2*( m_vector[0] * m_vector[2] + m_scalar*m_vector[1] );
    dest[1]=2*( m_vector[1] * m_vector[2] - m_scalar*m_vector[0] );
    dest[2]=(Float) (1.0 - 2*( m_vector[0] * m_vector[0] + m_vector[1] * m_vector[1] ));
  }
}

// ===================================================================
// operator==(): Compare two unit quaternions to see if they create
// the same rotation, with a roundoff tolerance of
// PR_QUATERNION_EPSILON.
//
// Note that -q produces the same rotation as +q, so this method will
// say they are identical.
// ===================================================================
int PrQuaternion::operator==( const PrQuaternion& rhs )
{
  Float mag2 = dot( rhs );
  return (mag2 > PR_QUATERNION_COS_THRESHHOLD
    || mag2 < -PR_QUATERNION_COS_THRESHHOLD );
}

// ===================================================================
// multiply(): Multiply two quaternions.
// Requires 16 mult + 8 add operations.
// ===================================================================
void PrQuaternion::multiply( const PrQuaternion& rhs,
                             PrQuaternion& dest ) const
{
  PrVector3 tmpV;

  dest.m_scalar = m_scalar * rhs.m_scalar - m_vector.dot( rhs.m_vector );

  rhs.m_vector.multiply( m_scalar, dest.m_vector );
  m_vector.multiply( rhs.m_scalar, tmpV );
  dest.m_vector += tmpV;
  m_vector.cross( rhs.m_vector, tmpV );
  dest.m_vector += tmpV;
}

// ===================================================================
// multiply(): Apply a unit quaternion to a 3x1 vector, to rotate the
// vector.
//
// If the quaternion is (w,v) and the vector is x, then this operation
// takes the quaternion product (w,v) * (0,x) * (w,-v), and discards
// the scalar part of the result.  If you do the math, this reduces to
// x * (w^2 - v.v) + 2w * (v X x) + 2v * (v . x).
//
// Requires 23 mult + 15 add operations
// ===================================================================
void PrQuaternion::multiply( const PrVector& rhs, PrVector& dest ) const
{
  SAIAssert( rhs.size() == 3 );
  dest.setSize( 3 );

  PrVector3 tmpV;
  rhs.multiply( m_scalar * m_scalar - m_vector.dot( m_vector ), dest );
  m_vector.cross( rhs, tmpV );
  tmpV *= (m_scalar + m_scalar);
  dest += tmpV;
  m_vector.multiply( 2 * m_vector.dot( rhs ), tmpV );
  dest += tmpV;
}

// ===================================================================
// angularError(): Find the instantaneous angular error between this
// orientation and a desired orientation.  dPhi = E_inv *( q - qd )
//
// this and qd should both be unit quaternions.  This operation is the
// following equation, where E(q) and q_tilde are 4x3 matrices such
// that q_tilde * q = 0:
//
//    dPhi = E(q)^-1 * (q - qd) = (2 * q_tilde^T) * (q - qd)
//         = -2 * q_tilde^T * qd
//    dPhi = -0.5 * ( R[x] X Rd[x]  +  R[y] X Rd[y]  +  R[z] X Rd[z] )
//
// The value of q_tilde depends on who you ask.  According to
// Featherstone & Mirtich, it is:
//
//     q_tilde = [ -x -y -z
//                  w -z  y
//                  z  w -x
//                 -y  x  w ]
//
// But according to Khatib, it is:
//     q_tilde = [ -x -y -z
//                  w  z -y
//                 -z  w  x
//                  y -x  w ]
//
// This routine uses the Khatib definition.
// ===================================================================
void
PrQuaternion::angularError( const PrQuaternion& qd, PrVector& dPhi ) const
{
  dPhi.setSize( 3 );

  dPhi[0] = -2 * ( - m_vector[0] * qd.m_scalar
                   + m_scalar    * qd.m_vector[0]
                   - m_vector[2] * qd.m_vector[1]
                   + m_vector[1] * qd.m_vector[2] );
  dPhi[1] = -2 * ( - m_vector[1] * qd.m_scalar
                   + m_vector[2] * qd.m_vector[0]
                   + m_scalar    * qd.m_vector[1]
                   - m_vector[0] * qd.m_vector[2] );
  dPhi[2] = -2 * ( - m_vector[2] * qd.m_scalar
                   - m_vector[1] * qd.m_vector[0]
                   + m_vector[0] * qd.m_vector[1]
                   + m_scalar    * qd.m_vector[2] );
}

// ===================================================================
// velocity(): Convert an angular veloctity (omega) into the
// derivative of the orientation quaternion (dq).
//
// This operation is the following equation, where E(q) and q_tilde
// are defined as in angularError():
//
//     dq = E(q) * omega
//        = 0.5 * q_tilde * omega (Khatib's)
// ===================================================================
void PrQuaternion::velocity( const PrVector& omega, PrQuaternion& dq ) const
{
  SAIAssert( omega.size() == 3 );

  dq.values((Float) ((- m_vector[0] * omega[0]
             - m_vector[1] * omega[1]
             - m_vector[2] * omega[2]) * .5),

            (Float) ((+ m_scalar    * omega[0]
             + m_vector[2] * omega[1]  // -
             - m_vector[1] * omega[2]) * .5), // +

            (Float) ((- m_vector[2] * omega[0]  // +
             + m_scalar    * omega[1]  
             + m_vector[0] * omega[2]) * .5), // -

            (Float) ((+ m_vector[1] * omega[0]  // -
             - m_vector[0] * omega[1]  // +
             + m_scalar    * omega[2]) * .5) );
}

// ===================================================================
// display(): Display the quaternion
// ===================================================================
void PrQuaternion::display( const char* name ) const
{
  if( name != NULL )
  {
    printf( "%s =\n", name );
  }

  printf( "[%.6f, (%.6f %.6f %.6f)]",
          m_scalar, m_vector[0], m_vector[1], m_vector[2] );

  if( name != NULL )
  {
    printf( "\n" );
  }
}


// ============================================================================
// interpolate(): interpolates between q1 and q2 to the specified fraction
// ============================================================================
PrQuaternion PrQuaternion::interpolate( const PrQuaternion& q10, const PrQuaternion& q20, double fraction )
{
  Float w2,x2,y2,z2;

  Float fCosTheta = q10.dot( q20 );
  if( fCosTheta >= 0 )
  {  w2 = q20.m_scalar; x2 = q20.m_vector[0]; y2 = q20.m_vector[1]; z2 = q20.m_vector[2]; }
  else
  {  w2 = -q20.m_scalar; x2 = -q20.m_vector[0]; y2 = -q20.m_vector[1]; z2 = -q20.m_vector[2]; fCosTheta = -fCosTheta; }


  Float r, s;
  // calculate interpolation factors
  if( fCosTheta > PR_QUATERNION_COS_THRESHHOLD )
  { // use linear interpolation
    r = (Float) (1 - fraction);
    s = (Float) fraction;
  }
  else
  { // calculate spherical factors
    Float alpha = (Float) acos( fCosTheta );
    Float phi = 1/alpha;
    r = (Float) sin( (1-fraction )*alpha ) * phi;
    s = (Float) sin( fraction*alpha ) * phi;
  }


  // set the interpolated quaternion
  PrQuaternion q;
  q.m_scalar    = r*q10.m_scalar    + s*w2;
  q.m_vector[0] = r*q10.m_vector[0] + s*x2;
  q.m_vector[1] = r*q10.m_vector[1] + s*y2;
  q.m_vector[2] = r*q10.m_vector[2] + s*z2;

  // normalize the result
  return q.unit(); 
}

