//
// File: minesweeper_controller.cpp
//
// Code generated for Simulink model 'minesweeper_controller'.
//
// Model version                  : 1.0
// Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
// C/C++ source code generated on : Fri Dec 19 12:10:28 2025
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "minesweeper_controller.h"
#include "minesweeper_controller_private.h"
#include "minesweeper_controller_dt.h"

// Real-time model
RT_MODEL_minesweeper_controll_T minesweeper_controller_M_ =
  RT_MODEL_minesweeper_controll_T();
RT_MODEL_minesweeper_controll_T *const minesweeper_controller_M =
  &minesweeper_controller_M_;

// Model step function
void minesweeper_controller_step(void)
{
  // External mode
  rtExtModeUploadCheckTrigger(1);

  {                                    // Sample time: [0.1s, 0.0s]
    rtExtModeUpload(0, (real_T)minesweeper_controller_M->Timing.taskTime0);
  }

  // signal main to stop simulation
  {                                    // Sample time: [0.1s, 0.0s]
    if ((rtmGetTFinal(minesweeper_controller_M)!=-1) &&
        !((rtmGetTFinal(minesweeper_controller_M)-
           minesweeper_controller_M->Timing.taskTime0) >
          minesweeper_controller_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(minesweeper_controller_M, "Simulation finished");
    }

    if (rtmGetStopRequested(minesweeper_controller_M)) {
      rtmSetErrorStatus(minesweeper_controller_M, "Simulation finished");
    }
  }

  // Update absolute time for base rate
  // The "clockTick0" counts the number of times the code of this task has
  //  been executed. The absolute time is the multiplication of "clockTick0"
  //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
  //  overflow during the application lifespan selected.

  minesweeper_controller_M->Timing.taskTime0 =
    ((time_T)(++minesweeper_controller_M->Timing.clockTick0)) *
    minesweeper_controller_M->Timing.stepSize0;
}

// Model initialize function
void minesweeper_controller_initialize(void)
{
  // Registration code
  rtmSetTFinal(minesweeper_controller_M, 300.0);
  minesweeper_controller_M->Timing.stepSize0 = 0.1;

  // External mode info
  minesweeper_controller_M->Sizes.checksums[0] = (1531930382U);
  minesweeper_controller_M->Sizes.checksums[1] = (2732830836U);
  minesweeper_controller_M->Sizes.checksums[2] = (2016237158U);
  minesweeper_controller_M->Sizes.checksums[3] = (4216256098U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[2];
    minesweeper_controller_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(minesweeper_controller_M->extModeInfo,
      &minesweeper_controller_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(minesweeper_controller_M->extModeInfo,
                        minesweeper_controller_M->Sizes.checksums);
    rteiSetTPtr(minesweeper_controller_M->extModeInfo, rtmGetTPtr
                (minesweeper_controller_M));
  }

  // data type transition information
  {
    static DataTypeTransInfo dtInfo;
    minesweeper_controller_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 19;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    // Block I/O transition table
    dtInfo.BTransTable = &rtBTransTable;

    // Parameters transition table
    dtInfo.PTransTable = &rtPTransTable;
  }
}

// Model terminate function
void minesweeper_controller_terminate(void)
{
  // (no terminate code required)
}

//
// File trailer for generated code.
//
// [EOF]
//
