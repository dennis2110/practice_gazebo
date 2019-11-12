#ifndef MMC_SUCCESS
	#define MMC_SUCCESS 1
#endif

#ifndef MMC_FAILED
	#define MMC_FAILED 0
#endif

#include "Definitions.h"
#include <ros/ros.h>
#include <iostream>
#include <sstream>



namespace TabletennisRobot
{
    class EposCommunication
    {
    public:
        EposCommunication();
        ~EposCommunication();
        int initialization();
        bool deviceOpenedCheck();
        bool homingCheck();


        int closeDevice();

    private:
        void  LogError(std::string functionName, int p_lResult, unsigned int p_ulErrorCode);
        void  LogInfo(std::string message);
        void  SeparatorLine();
        void  PrintHeader();
        void  PrintSettings();
        void  SetDefaultParameters();

        int   OpenDevice(unsigned int* p_pErrorCode);
        int   CloseDevice(unsigned int* p_pErrorCode);



    public:

    private:
        typedef void* HANDLE;
        typedef int BOOL;

        //Variables:
        bool deviceOpenedCheckStatus = MMC_FAILED;
        bool homingCompletedStatus = MMC_FAILED;
        void* g_pKeyHandle;
        unsigned short g_usNodeId;
        int g_baudrate;
        std::string g_deviceName;
        std::string g_protocolStackName;
        std::string g_interfaceName;
        std::string g_portName;
        const std::string g_programName = "EPOS2 Controller";

    };
}