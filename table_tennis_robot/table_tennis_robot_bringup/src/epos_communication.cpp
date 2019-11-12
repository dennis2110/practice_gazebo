#include "epos_communication.h"

namespace TabletennisRobot
{
    EposCommunication::EposCommunication(){
        g_pKeyHandle = 0;
        g_usNodeId = 1;
        g_baudrate = 0;
    }

    EposCommunication::~EposCommunication(){

    }

    int EposCommunication::initialization(){
        int lResult = MMC_SUCCESS;
        unsigned int ulErrorCode = 0;

        PrintHeader();
        SetDefaultParameters();
        PrintSettings();

        //Open device:
        if((lResult = OpenDevice(&ulErrorCode))==MMC_FAILED)
        {
            LogError("OpenDevice", lResult, ulErrorCode);
        }
        else {
            deviceOpenedCheckStatus = MMC_SUCCESS; //used to forbid other functions as getPosition and getVelocity if device is not opened
        }




        ///////////////////////////////
        return lResult;
    }

    bool EposCommunication::deviceOpenedCheck(){
        return deviceOpenedCheckStatus;
    }
    
    bool EposCommunication::homingCheck(){
        return homingCompletedStatus;
    }

    int EposCommunication::closeDevice()
    {
        //Close device:
        int lResult = MMC_FAILED;
        unsigned int ulErrorCode = 0;
        if((lResult = CloseDevice(&ulErrorCode))==MMC_FAILED)
        {
            LogError("CloseDevice", lResult, ulErrorCode);
            return lResult;
        }
        return lResult;
    }

    ////////////////////////////
    ///// private function /////
    ////////////////////////////
    void EposCommunication::LogError(std::string functionName, int p_lResult, unsigned int p_ulErrorCode)
    {   
	    std::cerr << g_programName << ": " << functionName << " failed (result=" << p_lResult << ", errorCode=0x" << std::hex << p_ulErrorCode << ")"<< std::endl;
    }

    void EposCommunication::LogInfo(std::string message)
    {
	    std::cout << message << std::endl;
    }

    void EposCommunication::SeparatorLine()
    {
        const int lineLength = 65;
        for(int i=0; i<lineLength; i++)
        {
            std::cout << "-";
        }
        std::cout << std::endl;
    }

    void EposCommunication::PrintHeader()
    {
        SeparatorLine();

        LogInfo("Initializing EPOS2 Communication Library");

        SeparatorLine();
    }

    void EposCommunication::PrintSettings()
    {
        std::stringstream msg;

        msg << "default settings:" << std::endl;
        msg << "node id             = " << g_usNodeId << std::endl;
        msg << "device name         = '" << g_deviceName << "'" << std::endl;
        msg << "protocal stack name = '" << g_protocolStackName << "'" << std::endl;
        msg << "interface name      = '" << g_interfaceName << "'" << std::endl;
        msg << "port name           = '" << g_portName << "'"<< std::endl;
        msg << "baudrate            = " << g_baudrate;

        LogInfo(msg.str());

        SeparatorLine();
    }

    void EposCommunication::SetDefaultParameters()
    {

        /* Options:
        * node id: default 1 (not ROS node!)
        * device name: EPOS2, EPOS4, default: EPOS4
        * protocol stack name: MAXON_RS232, CANopen, MAXON SERIAL V2, default: MAXON SERIAL V2
        * interface name: RS232, USB, CAN_ixx_usb 0, CAN_kvaser_usb 0,... default: USB
        * port name: COM1, USB0, CAN0,... default: USB0
        * baudrate: 115200, 1000000,... default: 1000000
        */

        //USB
        g_usNodeId = 1;
        g_deviceName = "EPOS2"; 
        g_protocolStackName = "MAXON SERIAL V2"; 
        g_interfaceName = "USB"; 
        g_baudrate = 1000000; 

        //get the port name:
        /*int lStartOfSelection = 1;
        int lMaxStrSize = 255;
        char* pPortNameSel = new char[lMaxStrSize];
        int lEndOfSelection = 0;
        unsigned int ulErrorCode = 0;
        VCS_GetPortNameSelection((char*)g_deviceName.c_str(), (char*)g_protocolStackName.c_str(), (char*)g_interfaceName.c_str(), lStartOfSelection, pPortNameSel, lMaxStrSize, &lEndOfSelection, &ulErrorCode);
        g_portName = pPortNameSel;
        ROS_INFO_STREAM("Port Name: " << g_portName);*/
        
        g_portName = "USB0";
    }

    int EposCommunication::OpenDevice(unsigned int* p_pErrorCode)
    {
        int lResult = MMC_FAILED;

        char* pDeviceName = new char[255];
        char* pProtocolStackName = new char[255];
        char* pInterfaceName = new char[255];
        char* pPortName = new char[255];

        strcpy(pDeviceName, g_deviceName.c_str());
        strcpy(pProtocolStackName, g_protocolStackName.c_str());
        strcpy(pInterfaceName, g_interfaceName.c_str());
        strcpy(pPortName, g_portName.c_str());

        LogInfo("Open device...");

        g_pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);

        if(g_pKeyHandle!=0 && *p_pErrorCode == 0)
        {
            unsigned int lBaudrate = 0;
            unsigned int lTimeout = 0;

            if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=MMC_FAILED)
            {
                if(VCS_SetProtocolStackSettings(g_pKeyHandle, g_baudrate, lTimeout, p_pErrorCode)!=MMC_FAILED)
                {
                    if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=MMC_FAILED)
                    {
                        if(g_baudrate==(int)lBaudrate)
                        {
                            lResult = MMC_SUCCESS;
                        }
                    }
                }
            }
        }
        else
        {
            g_pKeyHandle = 0;
            ROS_ERROR("Opening device failed.");
        }

        delete []pDeviceName;
        delete []pProtocolStackName;
        delete []pInterfaceName;
        delete []pPortName;

        return lResult;
    }

    int EposCommunication::CloseDevice(unsigned int* p_pErrorCode)
    {
        int lResult = MMC_FAILED;

        *p_pErrorCode = 0;

        /*if(VCS_SetDisableState(g_pKeyHandle, g_usNodeId, p_pErrorCode) == MMC_FAILED)
        {
            lResult = MMC_FAILED;
        }*/

        LogInfo("Close device");

        if(VCS_CloseDevice(g_pKeyHandle, p_pErrorCode)!=MMC_FAILED && *p_pErrorCode == 0)
        {
            lResult = MMC_SUCCESS;
        }

        return lResult;
    }

}