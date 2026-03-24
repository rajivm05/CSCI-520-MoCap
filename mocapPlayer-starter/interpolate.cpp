#ifdef WIN32
  #define _CRT_SECURE_NO_WARNINGS
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>
#include "interpolator.h"
#include "motion.h"

// reads a keyframe file: one integer per line (frame indices, 0-based)
static std::vector<int> LoadKeyframeFile(const char * filepath)
{
  std::vector<int> kfList;
  std::ifstream ifs(filepath);
  if (!ifs.is_open())
  {
    printf("Error: cannot open keyframe file %s\n", filepath);
    exit(1);
  }
  int idx;
  while (ifs >> idx)
    kfList.push_back(idx);
  ifs.close();
  printf("Loaded %d keyframes from %s\n", (int)kfList.size(), filepath);
  return kfList;
}

int main(int argc, char **argv)
{
  // two usage modes:
  //   uniform:     interpolate <asf> <amc> <l|b> <e|q> <N> <output.amc>
  //   non-uniform: interpolate <asf> <amc> <l|b> <e|q> -k <keyframe_file> <output.amc>
  if (argc != 7 && argc != 8)
  {
    printf("Interpolates motion capture data.\n");
    printf("Uniform usage:     %s <skeleton.asf> <motion.amc> <l|b> <e|q> <N> <output.amc>\n", argv[0]);
    printf("Non-uniform usage: %s <skeleton.asf> <motion.amc> <l|b> <e|q> -k <keyframes.txt> <output.amc>\n", argv[0]);
    printf("  interpolation method:  l = linear, b = Bezier\n");
    printf("  angle representation:  e = Euler, q = quaternion\n");
    printf("  N: number of skipped frames (uniform mode)\n");
    printf("  -k <file>: keyframe file with one frame index per line (non-uniform mode)\n");
    printf("Example (uniform):     %s skeleton.asf motion.amc l e 5 output.amc\n", argv[0]);
    printf("Example (non-uniform): %s skeleton.asf motion.amc b q -k keyframes.txt output.amc\n", argv[0]);
    return -1;
  }

  char * inputSkeletonFile = argv[1];
  char * inputMotionCaptureFile = argv[2];
  char * interpolationTypeString = argv[3];
  char * angleRepresentationString = argv[4];

  // detect whether we are in non-uniform keyframe mode
  bool nonUniformMode = false;
  int N = 0;
  std::vector<int> keyframeList;
  char * outputMotionCaptureFile = NULL;

  if (strcmp(argv[5], "-k") == 0)
  {
    // non-uniform mode: argv[5]="-k", argv[6]=keyframe file, argv[7]=output
    if (argc != 8)
    {
      printf("Error: non-uniform mode requires 8 arguments.\n");
      return -1;
    }
    nonUniformMode = true;
    keyframeList = LoadKeyframeFile(argv[6]);
    outputMotionCaptureFile = argv[7];
    printf("Non-uniform keyframe mode with %d keyframes.\n", (int)keyframeList.size());
  }
  else
  {
    // uniform mode: argv[5]=N, argv[6]=output
    N = strtol(argv[5], NULL, 10);
    if (N < 0)
    {
      printf("Error: invalid N value (%d).\n", N);
      exit(1);
    }
    outputMotionCaptureFile = argv[6];
    printf("N=%d\n", N);
  }

  Skeleton * pSkeleton = NULL;	// skeleton as read from an ASF file (input)
  Motion * pInputMotion = NULL; // motion as read from an AMC file (input)

  printf("Loading skeleton from %s...\n", inputSkeletonFile);
  try
  {
    pSkeleton = new Skeleton(inputSkeletonFile, MOCAP_SCALE);
  }
  catch(int exceptionCode)
  {
    printf("Error: failed to load skeleton from %s. Code: %d\n", inputSkeletonFile, exceptionCode);
    exit(1);
  }

  printf("Loading input motion from %s...\n", inputMotionCaptureFile);
  try
  {
    pInputMotion = new Motion(inputMotionCaptureFile, MOCAP_SCALE, pSkeleton);
  }
  catch(int exceptionCode)
  {
    printf("Error: failed to load motion from %s. Code: %d\n", inputMotionCaptureFile, exceptionCode);
    exit(1);
  }

  pSkeleton->enableAllRotationalDOFs();

  InterpolationType interpolationType;
  if (interpolationTypeString[0] == 'l')
    interpolationType = LINEAR;
  else if (interpolationTypeString[0] == 'b')
    interpolationType = BEZIER;
  else
  {
    printf("Error: unknown interpolation type: %s\n", interpolationTypeString);
    exit(1);
  }
  printf("Interpolation type is: %s\n", (interpolationType == LINEAR) ? "LINEAR" : "BEZIER");

  AngleRepresentation angleRepresentation;
  if (angleRepresentationString[0] == 'e')
    angleRepresentation = EULER;
  else if (angleRepresentationString[0] == 'q')
    angleRepresentation = QUATERNION;
  else
  {
    printf("Error: unknown angle representation: %s\n", angleRepresentationString);
    exit(1);
  }
  printf("Angle representation for interpolation is: %s\n", (angleRepresentation == EULER) ? "EULER" : "QUATERNION");

  Interpolator interpolator;
  interpolator.SetInterpolationType(interpolationType);
  interpolator.SetAngleRepresentation(angleRepresentation);

  printf("Interpolating...\n");
  Motion * pOutputMotion; // interpolated motion (output)

  if (nonUniformMode)
    interpolator.InterpolateNonUniform(pInputMotion, &pOutputMotion, keyframeList);
  else
    interpolator.Interpolate(pInputMotion, &pOutputMotion, N);

  if (pOutputMotion == NULL)
  {
    printf("Error: interpolation failed. No output generated.\n");
    exit(1);
  }
  printf("Interpolation completed.\n");

  printf("Writing output motion capture file to %s...\n", outputMotionCaptureFile);
  int forceAllJointsBe3DOF = 1;
  pOutputMotion->writeAMCfile(outputMotionCaptureFile, 0.06, forceAllJointsBe3DOF);

  return 0;
}

