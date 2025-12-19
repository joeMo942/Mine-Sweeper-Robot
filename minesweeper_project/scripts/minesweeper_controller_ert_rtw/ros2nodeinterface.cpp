
//
// File ros2nodeinterface.cpp
//
// Code generated for Simulink model 'minesweeper_controller'.
//
// Model version                  : 1.0
// Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
// C/C++ source code generated on : Fri Dec 19 12:10:41 2025
//
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4244)
#pragma warning(disable : 4265)
#pragma warning(disable : 4458)
#pragma warning(disable : 4100)
#pragma comment(lib, "Ws2_32.lib")
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wshadow"
#endif //_MSC_VER
#include "rclcpp/rclcpp.hpp"
#include "minesweeper_controller.h"
#include "ros2nodeinterface.h"
#include <thread>
#include <chrono>
#include <utility>
const std::string SLROSNodeName("minesweeper_controller");
extern rclcpp::Node::SharedPtr SLROSNodePtr;
#ifndef RT_MEMORY_ALLOCATION_ERROR_DEF
#define RT_MEMORY_ALLOCATION_ERROR_DEF
const char *RT_MEMORY_ALLOCATION_ERROR = "memory allocation error";
#endif
namespace ros2 {
namespace matlab {
NodeInterface::NodeInterface()
    : mExec()
    , mBaseRateSem()
    , mBaseRateThread()
    , mSchedulerTimer()
    , mExtModeThread()
    , mStopSem()
    , mRunModel(true){
  }
NodeInterface::~NodeInterface() {
    terminate();
  }
void NodeInterface::initialize(int argc, char * const argv[]) {
    try {
        //initialize ros2
        std::vector<char *> args(argv, argv + argc);
        rclcpp::init(static_cast<int>(args.size()), args.data());
        //create the Node specified in Model
        std::string NodeName("minesweeper_controller");
        SLROSNodePtr = std::make_shared<rclcpp::Node>(NodeName);
        RCLCPP_INFO(SLROSNodePtr->get_logger(),"** Starting the model \"minesweeper_controller\" **\n");
        mExec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        mExec->add_node(SLROSNodePtr);
		{
			char* extmodeArg[] = {"-port","17725","-blocking","1","-verbose","0"};
			rtExtModeParseArgs(6, (const char_T **)extmodeArg, NULL);
		}
        //initialize the model which will initialize the publishers and subscribers
		rtmSetErrorStatus(minesweeper_controller_M, (NULL));
        minesweeper_controller_initialize();
		/* External mode */
		rtSetTFinalForExtMode(&rtmGetTFinal(minesweeper_controller_M));
		rtExtModeCheckInit(1);
		{
			boolean_T rtmStopReq = false;
			rtExtModeWaitForStartPkt(minesweeper_controller_M->extModeInfo, 1,
				&rtmStopReq);
			if (rtmStopReq) {
				rtmSetStopRequested(minesweeper_controller_M, true);
			}
		}
		rtERTExtModeStartMsg();
		mExtModeThread = std::make_shared<std::thread>(&NodeInterface::extmodeBackgroundTask, this);
        //create the threads for the rates in the Model
        mBaseRateThread = std::make_shared<std::thread>(&NodeInterface::baseRateTask, this);
        // Create "MutuallyExclusive" callback group for callback associated with mSchedulerTimer to prevent
        // it from being executed in parallel.
		mSchedulerGroup = SLROSNodePtr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		mSchedulerTimer = SLROSNodePtr->create_wall_timer(std::chrono::nanoseconds(100000000),std::bind(&NodeInterface::schedulerThreadCallback,this),mSchedulerGroup);
    }
    catch (std::exception& ex) {
        std::cout << ex.what() << std::endl;
        throw ex;
    }
    catch (...) {
        std::cout << "Unknown exception" << std::endl;
        throw;
    }
}
int NodeInterface::run() {
  if (mExec) {
    mExec->spin();
  }
  mRunModel = false;
  return 0;
}
boolean_T NodeInterface::getStopRequestedFlag(void) {
    #ifndef rtmGetStopRequested
    return (!(rtmGetErrorStatus(minesweeper_controller_M) 
        == (NULL)));
    #else
    return (!(rtmGetErrorStatus(minesweeper_controller_M) 
        == (NULL)) || rtmGetStopRequested(minesweeper_controller_M));
    #endif
}
void NodeInterface::stop(void) {
  if (mExec.get()) {
    mExec->cancel();
    if (SLROSNodePtr) {
      mExec->remove_node(SLROSNodePtr);
    }
    while (mExec.use_count() > 1);
  }
}
void NodeInterface::terminate(void) {
    if (mBaseRateThread.get()) {
        mRunModel = false;
        mBaseRateSem.notify();
        mBaseRateThread->join();
		if (mSchedulerTimer.get()) {
        	mSchedulerTimer->cancel();
        	mSchedulerTimer->reset();
		}
        mBaseRateThread.reset();
        minesweeper_controller_terminate();
        rtExtModeShutdown(1);
        mExec.reset();
        SLROSNodePtr.reset();
        rclcpp::shutdown();
    }
}
//
// Scheduler Task using clock timer to run base-rate
//
void NodeInterface::schedulerThreadCallback(void)
{
	if(mRunModel) {
        mBaseRateSem.notify();
    }
}
//
//Model specific
// Base-rate task
void NodeInterface::baseRateTask(void) {
  mRunModel = (rtmGetErrorStatus(minesweeper_controller_M) ==
              (NULL));
  while (mRunModel) {
    mBaseRateSem.wait();
#ifdef MW_DEBUG_LOG
    RCLCPP_INFO(SLROSNodePtr->get_logger(),"** Base rate task semaphore received\n");
#endif
    if (!mRunModel) break;
	/* External mode */
    {
      boolean_T rtmStopReq = false;
      rtExtModePauseIfNeeded(minesweeper_controller_M->extModeInfo, 1,
        &rtmStopReq);
      if (rtmStopReq) {
        rtmSetStopRequested(minesweeper_controller_M, true);
      }
      if (rtmGetStopRequested(minesweeper_controller_M) == true) {
        rtmSetErrorStatus(minesweeper_controller_M, "Simulation finished");
		mRunModel = false;
        break;
      }
    }
    minesweeper_controller_step(
	);
    rtExtModeCheckEndTrigger();
    mRunModel = !NodeInterface::getStopRequestedFlag();
  }
  NodeInterface::stop();
}
void NodeInterface::extmodeBackgroundTask(void)
{
  while (mRunModel) {
    /* External mode */
    {
      boolean_T rtmStopReq = false;
      rtExtModeOneStep(minesweeper_controller_M->extModeInfo, 1,
                       &rtmStopReq);
      if (rtmStopReq) {
        rtmSetStopRequested(minesweeper_controller_M, true);
      }
    }
  }
}
}//namespace matlab
}//namespace ros2
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
