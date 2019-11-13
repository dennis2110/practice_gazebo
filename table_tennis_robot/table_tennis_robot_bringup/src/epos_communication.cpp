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
            return false;
        }
        else {
            deviceOpenedCheckStatus = MMC_SUCCESS; //used to forbid other functions as getPosition and getVelocity if device is not opened
        }

        //Prepare EPOS controller:
        if((lResult = PrepareEpos(&ulErrorCode))==MMC_FAILED)
        {
            LogError("PrepareEpos", lResult, ulErrorCode);
            return false;
        }
        else
        {
            motorEnableCheckStatus = MMC_SUCCESS;
        }
        


        ///////////////////////////////
        return lResult;
    }

    bool EposCommunication::deviceOpenedCheck(){
        return deviceOpenedCheckStatus;
    }

    bool EposCommunication::motorEnableCheck(){
        return motorEnableCheckStatus;
    }
    
    bool EposCommunication::homingCheck(){
        return homingCompletedStatus;
    }

    bool EposCommunication::PPMCheck(){
        return PPMStatus;
    }

    int EposCommunication::openDevice(unsigned short p_usNodeId, std::string p_deviceName, std::string p_protocolStackName, std::string p_interfaceName, std::string p_portName, int p_baudrate){
        int lResult = MMC_SUCCESS;
        unsigned int ulErrorCode = 0;

        g_usNodeId = p_usNodeId;
        g_deviceName = p_deviceName;
        g_protocolStackName = p_protocolStackName;
        g_interfaceName = p_interfaceName;
        g_portName = p_portName;
        g_baudrate = p_baudrate;

        //Open device:
        if((lResult = OpenDevice(&ulErrorCode))==MMC_FAILED)
        {
            LogError("OpenDevice", lResult, ulErrorCode);
        }
        else {
            deviceOpenedCheckStatus = MMC_SUCCESS; //used to forbid other functions as getPosition and getVelocity if device is not opened
        }

        return lResult;
    }

    int EposCommunication::closeDevice()
    {
        //Close device:
        int lResult = MMC_FAILED;
        unsigned int ulErrorCode = 0;
        if((lResult = CloseDevice(&ulErrorCode))==MMC_FAILED)
        {
            LogError("CloseDevice", lResult, ulErrorCode);
        }
        else {
            deviceOpenedCheckStatus = MMC_FAILED; //used to forbid other functions as getPosition and getVelocity if device is not opened
        }
        return lResult;
    }

    int EposCommunication::enableMotor(){
        //Prepare EPOS controller:
        int lResult = MMC_FAILED;
        unsigned int ulErrorCode = 0;
        if((lResult = PrepareEpos(&ulErrorCode))==MMC_FAILED)
        {
            LogError("PrepareEpos", lResult, ulErrorCode);
        }
        else
        {
            motorEnableCheckStatus = MMC_SUCCESS;
        }
        return lResult;
    }

    int  EposCommunication::disableMotor(){
        int lResult = MMC_FAILED;
        unsigned int ulErrorCode = 0;
        if((lResult = DisableEpos(&ulErrorCode)) == MMC_FAILED)
        {
            LogError("DisableEpos", lResult, ulErrorCode);
        }else
        {
            motorEnableCheckStatus = MMC_FAILED;
        }
        return lResult;
    }

    int EposCommunication::autoHoming(){
        return MMC_FAILED;
    }

    int EposCommunication::startProfilePositionMode(unsigned int p_profile_velocity, unsigned int p_profile_acceleration,unsigned int p_profile_deceleration){
        int lResult = MMC_SUCCESS;
	    unsigned int ulErrorCode = 0;

        g_profile_velocity = p_profile_velocity;
        g_profile_acceleration = p_profile_acceleration;
        g_profile_deceleration = p_profile_deceleration;

        if(ActivateProfilePositionMode(g_pKeyHandle,g_usNodeId,&ulErrorCode) == MMC_FAILED){
            ROS_ERROR("activate PPM fail");
        }

        if(SetPositionProfile(&ulErrorCode) == MMC_FAILED){
            ROS_ERROR("set PPM fail");
        }else{
            PPMStatus = MMC_SUCCESS;
        }

        return lResult;
    }

    int EposCommunication::setPosition(float position_setpoint){
        int lResult = MMC_SUCCESS;
	    unsigned int ulErrorCode = 0;

        if(SetPosition((long)position_setpoint, &ulErrorCode) == MMC_FAILED){
            ROS_ERROR("set position fail");
        }else{
			ROS_INFO("SetPosition executed.");
		}

        return lResult;
    }

    int EposCommunication::getPosition(float* pPositionIs){
        int lResult = MMC_FAILED;
        unsigned int ulErrorCode = 0;
        int pPositionIsCounts = 0;

        if((lResult = GetPosition(&pPositionIsCounts, &ulErrorCode))==MMC_FAILED)
        {
            LogError("getPosition", lResult, ulErrorCode);
            return lResult;
        }
        //*pPositionIs = &(float)pPositionIsCounts;

        //only for Debugging
        //ROS_INFO_STREAM("!!! pPositionIs: " << *pPositionIs << " pPositionIsCounts: " << pPositionIsCounts);
        return lResult;
    }

    int EposCommunication::getVelocity(float* pVelocityIs){
        int lResult = MMC_FAILED;
        unsigned int ulErrorCode = 0;
        int pVelocityIsCounts;

        if((lResult = GetVelocity(&pVelocityIsCounts, &ulErrorCode))==MMC_FAILED)
        {
            LogError("getVelocity", lResult, ulErrorCode);
            return lResult;
        }
        //*pVelocityIs = countsTomm(&pVelocityIsCounts);
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

    int EposCommunication::PrepareEpos(unsigned int* p_pErrorCode){
        int lResult = MMC_SUCCESS;
        BOOL oIsFault = 0; //0 is not in fault state

        if(VCS_GetFaultState(g_pKeyHandle, g_usNodeId, &oIsFault, p_pErrorCode ) == MMC_FAILED)
        {
            LogError("VCS_GetFaultState", lResult, *p_pErrorCode);
            lResult = MMC_FAILED;
        }

        ROS_INFO_STREAM("Debug 1: FaultState:" << oIsFault);

        if(oIsFault)
        {
            std::stringstream msg;
            msg << "clear fault, node = '" << g_usNodeId << "'";
            LogInfo(msg.str());

            if(VCS_ClearFault(g_pKeyHandle, g_usNodeId, p_pErrorCode) == MMC_FAILED)
            {
                LogError("VCS_ClearFault", lResult, *p_pErrorCode);
                lResult = MMC_FAILED;
            }
        }

        BOOL oIsEnabled = 0;

        if(VCS_GetEnableState(g_pKeyHandle, g_usNodeId, &oIsEnabled, p_pErrorCode) == MMC_FAILED)
        {
            LogError("VCS_GetEnableState", lResult, *p_pErrorCode);
            lResult = MMC_FAILED;
        }

        if(!oIsEnabled)
        {
            if(VCS_SetEnableState(g_pKeyHandle, g_usNodeId, p_pErrorCode) == MMC_FAILED)
            {
                LogError("VCS_SetEnableState", lResult, *p_pErrorCode);
                lResult = MMC_FAILED;
            }
            else{
                VCS_GetEnableState(g_pKeyHandle, g_usNodeId, &oIsEnabled, p_pErrorCode);
                ROS_INFO_STREAM("SetEnableState should be 1:" <<  oIsEnabled);
            }
        }

        return lResult;
    }

    int EposCommunication::DisableEpos(unsigned int* p_pErrorCode){
        int lResult = MMC_SUCCESS;

        *p_pErrorCode = 0;

        if(VCS_SetDisableState(g_pKeyHandle, g_usNodeId, p_pErrorCode) == MMC_FAILED)
        {
            lResult = MMC_FAILED;
        }
        
        return lResult;
    }

    int EposCommunication::ActivateProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode)
    {
        int lResult = MMC_SUCCESS;
        std::stringstream msg;

        msg << "set profile position mode, node = " << p_usNodeId;
        LogInfo(msg.str());

        if(VCS_ActivateProfilePositionMode(p_DeviceHandle, p_usNodeId, p_pErrorCode) == MMC_FAILED)
        {
            LogError("VCS_ActivateProfilePositionMode", lResult, *p_pErrorCode);
            lResult = MMC_FAILED;
        }
        else {
            ROS_INFO("VCS_ActivateProfilePositionMode successfull.");
        }
        return lResult;
    }

    int EposCommunication::SetPositionProfile(unsigned int* p_pErrorCode)
    {
        //to use set variables below first!
        int lResult = MMC_SUCCESS;

        if(VCS_SetPositionProfile(g_pKeyHandle, g_usNodeId, g_profile_velocity, g_profile_acceleration, g_profile_deceleration, p_pErrorCode) == MMC_FAILED)
        {
            LogError("VCS_SetPositionProfile", lResult, *p_pErrorCode);
            lResult = MMC_FAILED;
        }

        return lResult;
    }

    int EposCommunication::SetPosition(long position_setpoint, unsigned int* p_pErrorCode){
        // absolute position, starts immediately
        int lResult = MMC_SUCCESS;
        std::stringstream msg;

        msg << "move to position = " << position_setpoint << ", node = " << g_usNodeId;
        LogInfo(msg.str());

        if(VCS_MoveToPosition(g_pKeyHandle, g_usNodeId, position_setpoint, 1, 1, p_pErrorCode) == MMC_FAILED)
            {
                LogError("VCS_MoveToPosition", lResult, *p_pErrorCode);
                lResult = MMC_FAILED;
            }
        else{
            ROS_INFO("Movement executed.");
        }

        return lResult;
    }

    int EposCommunication::GetPosition(int* pPositionIsCounts, unsigned int* p_pErrorCode){
        int lResult = MMC_SUCCESS;

        if(VCS_GetPositionIs(g_pKeyHandle, g_usNodeId, pPositionIsCounts, p_pErrorCode) == MMC_FAILED)
        {
            LogError("VCS_GetPositionIs", lResult, *p_pErrorCode);
            lResult = MMC_FAILED;
        }
        return lResult;
    }

    int EposCommunication::GetVelocity(int* pVelocityIsCounts, unsigned int* p_pErrorCode){
        int lResult = MMC_SUCCESS;

        if(VCS_GetVelocityIs(g_pKeyHandle, g_usNodeId, pVelocityIsCounts, p_pErrorCode) == MMC_FAILED)
        {
            LogError("VCS_GetVelocityIs", lResult, *p_pErrorCode);
            lResult = MMC_FAILED;
        }
        return lResult;
    }

    float EposCommunication::countsTomm(int* counts){
        //can't achieve now  (need actual motor reduction ratio)
        return 0.0;
    }

    int EposCommunication::mmToCounts(float mm){
        //can't achieve now  (need actual motor reduction ratio)
        return 0;
    }




}