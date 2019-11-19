#ifndef MMC_SUCCESS
	#define MMC_SUCCESS 1
#endif

#ifndef MMC_FAILED
	#define MMC_FAILED 0
#endif

//#define UseSubDevice

#include "arduserial.h"
#include "Definitions.h"
#include <ros/ros.h>
#include <iostream>
#include <sstream>


namespace TabletennisRobot
{
    class EposCommunication
    {
    private:
        typedef void* HANDLE;
        typedef int BOOL;
    public:
        EposCommunication();
        ~EposCommunication();
        int initialization();
        bool deviceOpenedCheck();
        bool motorEnableCheck();
        bool homingCheck();
        bool PPMCheck();

        int openDevice(std::string p_deviceName, std::string p_protocolStackName, std::string p_interfaceName, std::string p_portName, int p_baudrate);
        int closeDevice();
        int enableMotor();
        int disableMotor();
        int autoHoming();
        int	startProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int p_profile_velocity, unsigned int p_profile_acceleration,unsigned int p_profile_deceleration);
        int	setPosition(HANDLE p_DeviceHandle, unsigned short p_usNodeId, float position_setpoint);
        int stopPosition(HANDLE p_DeviceHandle, unsigned short p_usNodeId);
        int	getPosition(unsigned short p_usNodeId, float* pPositionIs);
        int	getVelocity(unsigned short p_usNodeId, float* pVelocityIs);

    

    private:
        void  LogError(std::string functionName, int p_lResult, unsigned int p_ulErrorCode);
        void  LogInfo(std::string message);
        void  SeparatorLine();
        void  PrintHeader();
        void  PrintSettings();
        void  SetDefaultParameters();

        int   OpenDevice(unsigned int* p_pErrorCode);
        int   CloseDevice(unsigned int* p_pErrorCode);
        int   PrepareEpos(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode);
        int   DisableEpos(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode);
        //autuhome private fun
        int	ActivateProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode);
        int	SetPositionProfile(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode);
        int	SetPosition(HANDLE p_DeviceHandle, unsigned short p_usNodeId, long position_setpoint, unsigned int* p_pErrorCode);
        int StopPosition(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode);
        int	GetPosition(unsigned short p_usNodeId, int* pPositionIsCounts, unsigned int* p_pErrorCode);
        int	GetVelocity(unsigned short p_usNodeId, int* pVelocityIsCounts, unsigned int* p_pErrorCode);

        long mToQC(float m);
        float QCTom(int* QC);

        float countsTomm(int* counts);
        int mmToCounts(float mm);




    public:
        const unsigned short g_usNodeId1 = 1;
        const unsigned short g_usNodeId2 = 2;
        const unsigned short g_usNodeId3 = 3;
        const unsigned short g_usNodeId4 = 4;
        const unsigned short g_usNodeId5 = 5;

        void* g_pKeyHandle;
        // subdevice
        void* subKeyHandle;


    private:

        //Variables:
        bool deviceOpenedCheckStatus = MMC_FAILED;
        bool motorEnableCheckStatus = MMC_FAILED;
        bool homingCompletedStatus = MMC_FAILED;
        bool PPMStatus = MMC_FAILED;
        
        
        

        int g_baudrate;
        std::string g_deviceName;
        std::string g_protocolStackName;
        std::string g_interfaceName;
        std::string g_portName;
        const std::string g_programName = "EPOS2 Controller";
        const std::string subDeviceName = "EPOS2";
        const std::string subProtocolStackName = "CANopen";

        unsigned int g_profile_velocity;
        unsigned int g_profile_acceleration;
        unsigned int g_profile_deceleration;

        ArduSerial homeArdu;
        
    };
}