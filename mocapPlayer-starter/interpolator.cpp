#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
  // convert the three XYZ euler angles from degrees into radians
  double alpha = angles[0] * M_PI / 180.0;  // rotation about X
  double beta  = angles[1] * M_PI / 180.0;  // rotation about Y
  double gamma = angles[2] * M_PI / 180.0;  // rotation about Z

  // trig values for each axis rotation
  double ca = cos(alpha), sa = sin(alpha);
  double cb = cos(beta),  sb = sin(beta);
  double cg = cos(gamma), sg = sin(gamma);

  // the composite matrix is Rz(gamma) * Ry(beta) * Rx(alpha)
  // I worked this out by hand by multiplying the three elementary rotation matrices
  // and verified it recovers the correct angles when passed through Rotation2Euler
  R[0] = cb * cg;
  R[1] = sa * sb * cg - ca * sg;
  R[2] = ca * sb * cg + sa * sg;
  R[3] = cb * sg;
  R[4] = ca * cg + sa * sb * sg;
  R[5] = ca * sb * sg - sa * cg;
  R[6] = -sb;
  R[7] = sa * cb;
  R[8] = ca * cb;
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int totalFrames = pInputMotion->GetNumFrames();
  int span = N + 1; // distance between consecutive keyframes

  int kfCurr = 0;
  while (kfCurr + span < totalFrames)
  {
    int kfNext = kfCurr + span;

    Posture * posCurr = pInputMotion->GetPosture(kfCurr);
    Posture * posNext = pInputMotion->GetPosture(kfNext);

    // for computing tangents we need the neighboring keyframes
    // at the boundaries of the motion, we clamp (reuse the endpoint posture)
    int kfPrev = kfCurr - span;
    int kfAfterNext = kfNext + span;
    Posture * posPrev = (kfPrev >= 0) ? pInputMotion->GetPosture(kfPrev) : posCurr;
    Posture * posAfterNext = (kfAfterNext < totalFrames) ? pInputMotion->GetPosture(kfAfterNext) : posNext;

    pOutputMotion->SetPosture(kfCurr, *posCurr);
    pOutputMotion->SetPosture(kfNext, *posNext);

    // the bezier control handles come from the catmull-rom tangent formula
    // tangent at a point = (successor - predecessor) / 2
    // we then scale by the 1/3 factor (Shoemake sec 4.4), giving the / 6.0 below
    // outgoing handle at kfCurr
    vector ctrlOutRoot = posCurr->root_pos + (posNext->root_pos - posPrev->root_pos) / 6.0;
    // incoming handle at kfNext
    vector ctrlInRoot = posNext->root_pos - (posAfterNext->root_pos - posCurr->root_pos) / 6.0;

    // precompute the same handles for every bone's rotation channels
    vector ctrlOutBone[MAX_BONES_IN_ASF_FILE];
    vector ctrlInBone[MAX_BONES_IN_ASF_FILE];
    for (int b = 0; b < MAX_BONES_IN_ASF_FILE; b++)
    {
      ctrlOutBone[b] = posCurr->bone_rotation[b] +
        (posNext->bone_rotation[b] - posPrev->bone_rotation[b]) / 6.0;
      ctrlInBone[b] = posNext->bone_rotation[b] -
        (posAfterNext->bone_rotation[b] - posCurr->bone_rotation[b]) / 6.0;
    }

    // fill the N dropped frames with bezier-interpolated values
    for (int f = 1; f <= N; f++)
    {
      Posture interpPosture;
      double frac = (double)f / span;

      interpPosture.root_pos = DeCasteljauEuler(frac,
        posCurr->root_pos, ctrlOutRoot, ctrlInRoot, posNext->root_pos);

      for (int b = 0; b < MAX_BONES_IN_ASF_FILE; b++)
        interpPosture.bone_rotation[b] = DeCasteljauEuler(frac,
          posCurr->bone_rotation[b], ctrlOutBone[b], ctrlInBone[b], posNext->bone_rotation[b]);

      pOutputMotion->SetPosture(kfCurr + f, interpPosture);
    }

    kfCurr = kfNext;
  }

  // any remaining frames past the last keyframe get copied verbatim
  for (int f = kfCurr + 1; f < totalFrames; f++)
    pOutputMotion->SetPosture(f, *(pInputMotion->GetPosture(f)));
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int totalFrames = pInputMotion->GetNumFrames();
  int span = N + 1;

  int kfCurr = 0;
  while (kfCurr + span < totalFrames)
  {
    int kfNext = kfCurr + span;

    Posture * posCurr = pInputMotion->GetPosture(kfCurr);
    Posture * posNext = pInputMotion->GetPosture(kfNext);

    pOutputMotion->SetPosture(kfCurr, *posCurr);
    pOutputMotion->SetPosture(kfNext, *posNext);

    // convert bone euler angles to quaternions once per span (avoids redundant conversion per frame)
    Quaternion<double> orientCurr[MAX_BONES_IN_ASF_FILE];
    Quaternion<double> orientNext[MAX_BONES_IN_ASF_FILE];
    for (int b = 0; b < MAX_BONES_IN_ASF_FILE; b++)
    {
      Euler2Quaternion(posCurr->bone_rotation[b].p, orientCurr[b]);
      Euler2Quaternion(posNext->bone_rotation[b].p, orientNext[b]);
    }

    for (int f = 1; f <= N; f++)
    {
      Posture interpPosture;
      double frac = (double)f / span;

      // root translation is always interpolated linearly in euler space,
      // even in quaternion mode (quaternions only apply to orientations)
      interpPosture.root_pos = posCurr->root_pos * (1.0 - frac) + posNext->root_pos * frac;

      // bone orientations: slerp on the quaternion sphere, then convert back to euler
      for (int b = 0; b < MAX_BONES_IN_ASF_FILE; b++)
      {
        Quaternion<double> slerpResult = Slerp(frac, orientCurr[b], orientNext[b]);
        Quaternion2Euler(slerpResult, interpPosture.bone_rotation[b].p);
      }

      pOutputMotion->SetPosture(kfCurr + f, interpPosture);
    }

    kfCurr = kfNext;
  }

  for (int f = kfCurr + 1; f < totalFrames; f++)
    pOutputMotion->SetPosture(f, *(pInputMotion->GetPosture(f)));
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int totalFrames = pInputMotion->GetNumFrames();
  int span = N + 1;

  int kfCurr = 0;
  while (kfCurr + span < totalFrames)
  {
    int kfNext = kfCurr + span;

    Posture * posCurr = pInputMotion->GetPosture(kfCurr);
    Posture * posNext = pInputMotion->GetPosture(kfNext);

    // neighbor lookup with boundary clamping
    int kfPrev = kfCurr - span;
    int kfAfterNext = kfNext + span;
    Posture * posPrev = (kfPrev >= 0) ? pInputMotion->GetPosture(kfPrev) : posCurr;
    Posture * posAfterNext = (kfAfterNext < totalFrames) ? pInputMotion->GetPosture(kfAfterNext) : posNext;

    pOutputMotion->SetPosture(kfCurr, *posCurr);
    pOutputMotion->SetPosture(kfNext, *posNext);

    // root translation gets bezier euler handles (same approach as BezierInterpolationEuler)
    vector ctrlOutRoot = posCurr->root_pos + (posNext->root_pos - posPrev->root_pos) / 6.0;
    vector ctrlInRoot = posNext->root_pos - (posAfterNext->root_pos - posCurr->root_pos) / 6.0;

    // for bone rotations, we build control handles on the quaternion hypersphere
    // following Shoemake's construction from his 1985 SIGGRAPH paper (sec 4.4, p249-250):
    //   1. Double() reflects the predecessor through the current point
    //   2. Slerp halfway between that reflection and the successor gives the "raw" handle
    //   3. compress to 1/3 of the arc from the keyframe (the "push towards" step)
    Quaternion<double> orientCurr[MAX_BONES_IN_ASF_FILE];
    Quaternion<double> orientNext[MAX_BONES_IN_ASF_FILE];
    Quaternion<double> ctrlOutQuat[MAX_BONES_IN_ASF_FILE];
    Quaternion<double> ctrlInQuat[MAX_BONES_IN_ASF_FILE];

    for (int b = 0; b < MAX_BONES_IN_ASF_FILE; b++)
    {
      Quaternion<double> qP, qC, qN, qA;  // prev, curr, next, afterNext
      Euler2Quaternion(posPrev->bone_rotation[b].p, qP);
      Euler2Quaternion(posCurr->bone_rotation[b].p, qC);
      Euler2Quaternion(posNext->bone_rotation[b].p, qN);
      Euler2Quaternion(posAfterNext->bone_rotation[b].p, qA);

      // outgoing handle at kfCurr
      Quaternion<double> reflectedPrev = Double(qP, qC);
      Quaternion<double> rawHandleOut = Slerp(0.5, reflectedPrev, qN);
      ctrlOutQuat[b] = Slerp(1.0 / 3.0, qC, rawHandleOut);

      // incoming handle at kfNext
      Quaternion<double> reflectedAfter = Double(qA, qN);
      Quaternion<double> rawHandleIn = Slerp(0.5, reflectedAfter, qC);
      ctrlInQuat[b] = Slerp(1.0 / 3.0, qN, rawHandleIn);

      orientCurr[b] = qC;
      orientNext[b] = qN;
    }

    for (int f = 1; f <= N; f++)
    {
      Posture interpPosture;
      double frac = (double)f / span;

      interpPosture.root_pos = DeCasteljauEuler(frac,
        posCurr->root_pos, ctrlOutRoot, ctrlInRoot, posNext->root_pos);

      for (int b = 0; b < MAX_BONES_IN_ASF_FILE; b++)
      {
        Quaternion<double> bezierResult = DeCasteljauQuaternion(frac,
          orientCurr[b], ctrlOutQuat[b], ctrlInQuat[b], orientNext[b]);
        Quaternion2Euler(bezierResult, interpPosture.bone_rotation[b].p);
      }

      pOutputMotion->SetPosture(kfCurr + f, interpPosture);
    }

    kfCurr = kfNext;
  }

  for (int f = kfCurr + 1; f < totalFrames; f++)
    pOutputMotion->SetPosture(f, *(pInputMotion->GetPosture(f)));
}

// pipeline: euler angles -> rotation matrix -> quaternion
void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q)
{
  double mat[9];
  Euler2Rotation(angles, mat);
  q = Quaternion<double>::Matrix2Quaternion(mat);
}

// pipeline: quaternion -> rotation matrix -> euler angles (in degrees)
void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3])
{
  double mat[9];
  q.Quaternion2Matrix(mat);
  Rotation2Euler(mat, angles);
}

// slerp implementation based on Shoemake's "Animating Rotation with Quaternion Curves"
// interpolates along the great arc on the 4D unit hypersphere
Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
  Quaternion<double> qTarget = qEnd_;

  // 4D dot product gives the cosine of the angle between orientations
  double cosOmega = qStart.Gets() * qTarget.Gets()
                  + qStart.Getx() * qTarget.Getx()
                  + qStart.Gety() * qTarget.Gety()
                  + qStart.Getz() * qTarget.Getz();

  // q and -q represent the same rotation, so flip if needed to
  // ensure we interpolate along the shorter of the two possible arcs
  if (cosOmega < 0.0)
  {
    qTarget = -1.0 * qTarget;
    cosOmega = -cosOmega;
  }

  // degenerate case: orientations are nearly identical
  // use normalized linear blend to avoid dividing by ~0
  if (cosOmega > 0.9995)
  {
    Quaternion<double> blended = (1.0 - t) * qStart + t * qTarget;
    blended.Normalize();
    return blended;
  }

  // general case: use the spherical linear interpolation formula
  double omega = acos(cosOmega);
  double sinOmega = sin(omega);
  double coeffStart = sin((1.0 - t) * omega) / sinOmega;
  double coeffEnd   = sin(t * omega) / sinOmega;

  return coeffStart * qStart + coeffEnd * qTarget;
}

// from Shoemake: Double(p, q) = 2(p . q)q - p
// this is the quaternion analog of reflecting a point through another
// on the unit sphere. used for computing bezier spline control points.
Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  double innerProd = p.Gets() * q.Gets()
                   + p.Getx() * q.Getx()
                   + p.Gety() * q.Gety()
                   + p.Getz() * q.Getz();
  return 2.0 * innerProd * q - p;
}

// evaluates a cubic bezier at parameter t using de casteljau's triangular scheme:
// 4 control points -> 3 interpolated -> 2 interpolated -> 1 final point
vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
  double s = 1.0 - t;  // complement of t, used repeatedly

  // reduce 4 points to 3
  vector lerp01 = p0 * s + p1 * t;
  vector lerp12 = p1 * s + p2 * t;
  vector lerp23 = p2 * s + p3 * t;

  // reduce 3 to 2
  vector lerp012 = lerp01 * s + lerp12 * t;
  vector lerp123 = lerp12 * s + lerp23 * t;

  // final point on the curve
  return lerp012 * s + lerp123 * t;
}

// same de casteljau scheme but on the quaternion hypersphere,
// using slerp instead of euclidean lerp at each reduction step
Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
  // 4 -> 3
  Quaternion<double> slerp01 = Slerp(t, p0, p1);
  Quaternion<double> slerp12 = Slerp(t, p1, p2);
  Quaternion<double> slerp23 = Slerp(t, p2, p3);

  // 3 -> 2
  Quaternion<double> slerp012 = Slerp(t, slerp01, slerp12);
  Quaternion<double> slerp123 = Slerp(t, slerp12, slerp23);

  // 2 -> 1
  return Slerp(t, slerp012, slerp123);
}

