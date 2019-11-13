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
        bool motorEnableCheck();
        bool homingCheck();
        bool PPMCheck();

        int openDevice(unsigned short p_usNodeId, std::string p_deviceName, std::string p_protocolStackName, std::string p_interfaceName, std::string p_portName, int p_baudrate);
        int closeDevice();
        int enableMotor();
        int disableMotor();
        int autoHoming();
        int	startProfilePositionMode(unsigned int p_profile_velocity, unsigned int p_profile_acceleration,unsigned int p_profile_deceleration);
        int	setPosition(float position_setpoint);
        int	getPosition(float* pPositionIs);
        int	getVelocity(float* pVelocityIs);

    private:
        typedef void* HANDLE;
        typedef int BOOL;

    private:
        void  LogError(std::string functionName, int p_lResult, unsigned int p_ulErrorCode);
        void  LogInfo(std::string message);
        void  SeparatorLine();
        void  PrintHeader();
        void  PrintSettings();
        void  SetDefaultParameters();

        int   OpenDevice(unsigned int* p_pErrorCode);
        int   CloseDevice(unsigned int* p_pErrorCode);
        int   PrepareEpos(unsigned int* p_pErrorCode);
        int   DisableEpos(unsigned int* p_pErrorCode);
        //autuhome private fun
        int	ActivateProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode);
        int	SetPositionProfile(unsigned int* p_pErrorCode);
        int	SetPosition(long position_setpoint, unsigned int* p_pErrorCode);
        int	GetPosition(int* pPositionIsCounts, unsigned int* p_pErrorCode);
        int	GetVelocity(int* pVelocityIsCounts, unsigned int* p_pErrorCode);
        float countsTomm(int* counts);
        int mmToCounts(float mm);




    public:

    private:

        //Variables:
        bool deviceOpenedCheckStatus = MMC_FAILED;
        bool motorEnableCheckStatus = MMC_FAILED;
        bool homingCompletedStatus = MMC_FAILED;
        bool PPMStatus = MMC_FAILED;
        
        void* g_pKeyHandle;
        unsigned short g_usNodeId;
        int g_baudrate;
        std::string g_deviceName;
        std::string g_protocolStackName;
        std::string g_interfaceName;
        std::string g_portName;
        const std::string g_programName = "EPOS2 Controller";

        unsigned int g_profile_velocity;
        unsigned int g_profile_acceleration;
        unsigned int g_profile_deceleration;
    };
}