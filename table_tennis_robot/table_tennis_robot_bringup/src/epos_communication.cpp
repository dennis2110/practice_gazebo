#include "epos_communication.h"

namespace TabletennisRobot
{
    EposCommunication::EposCommunication():homeArdu("/dev/ttyACM0",6){
        g_pKeyHandle = 0;
        subKeyHandle = 0;
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
        if(enableMotor()==MMC_FAILED){
            LogError("enableMotor", lResult, ulErrorCode);
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

    int EposCommunication::openDevice(std::string p_deviceName, std::string p_protocolStackName, std::string p_interfaceName, std::string p_portName, int p_baudrate){
        int lResult = MMC_SUCCESS;
        unsigned int ulErrorCode = 0;

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
            homingCompletedStatus = MMC_FAILED;
            PPMStatus = MMC_FAILED;
        }
        return lResult;
    }

    int EposCommunication::enableMotor(){
        //Prepare EPOS controller:
        int lResult = MMC_FAILED;
        unsigned int ulErrorCode = 0;
        
#ifdef UseSubDevice
        if((lResult = PrepareEpos(g_pKeyHandle, g_usNodeId1,&ulErrorCode))==MMC_FAILED)
        {
            LogError("PrepareEpos(ID1)", lResult, ulErrorCode);
        }
        else if((lResult = PrepareEpos(subKeyHandle, g_usNodeId2,&ulErrorCode))==MMC_FAILED)
        {
            LogError("PrepareEpos(ID2)", lResult, ulErrorCode);
        }
        else if((lResult = PrepareEpos(subKeyHandle, g_usNodeId3,&ulErrorCode))==MMC_FAILED)
        {
            LogError("PrepareEpos(ID3)", lResult, ulErrorCode);
        }
        else if((lResult = PrepareEpos(subKeyHandle, g_usNodeId4,&ulErrorCode))==MMC_FAILED)
        {
            LogError("PrepareEpos(ID4)", lResult, ulErrorCode);
        }
        else if((lResult = PrepareEpos(subKeyHandle, g_usNodeId5,&ulErrorCode))==MMC_FAILED)
        {
            LogError("PrepareEpos(ID5)", lResult, ulErrorCode);
        }
        else
        {
            motorEnableCheckStatus = MMC_SUCCESS;
        }
#else
        if((lResult = PrepareEpos(g_pKeyHandle, g_usNodeId1,&ulErrorCode))==MMC_FAILED)
        {
            LogError("PrepareEpos(ID1)", lResult, ulErrorCode);
        }else
        {
            motorEnableCheckStatus = MMC_SUCCESS;
        }
#endif
        return lResult;
    }

    int  EposCommunication::disableMotor(){
        int lResult = MMC_FAILED;
        unsigned int ulErrorCode = 0;
#ifdef UseSubDevice
        if((lResult = DisableEpos(g_pKeyHandle, g_usNodeId1,&ulErrorCode)) == MMC_FAILED)
        {
            LogError("DisableEpos (ID1)", lResult, ulErrorCode);
        }else if((lResult = DisableEpos(subKeyHandle, g_usNodeId2,&ulErrorCode)) == MMC_FAILED)
        {
            LogError("DisableEpos (ID2)", lResult, ulErrorCode);
        }
        else if((lResult = DisableEpos(subKeyHandle, g_usNodeId3,&ulErrorCode))==MMC_FAILED)
        {
            LogError("DisableEpos (ID3)", lResult, ulErrorCode);
        }
        else if((lResult = DisableEpos(subKeyHandle, g_usNodeId4,&ulErrorCode))==MMC_FAILED)
        {
            LogError("DisableEpos (ID4)", lResult, ulErrorCode);
        }
        else if((lResult = DisableEpos(subKeyHandle, g_usNodeId5,&ulErrorCode))==MMC_FAILED)
        {
            LogError("DisableEpos (ID5)", lResult, ulErrorCode);
        }
        else
        {
            motorEnableCheckStatus = MMC_FAILED;
        }
#else
        if((lResult = DisableEpos(g_pKeyHandle, g_usNodeId1,&ulErrorCode)) == MMC_FAILED)
        {
            LogError("DisableEpos (ID1)", lResult, ulErrorCode);
        }else
        {
            motorEnableCheckStatus = MMC_FAILED;
        }
#endif
        return lResult;
    }

    int EposCommunication::autoHoming(){
        int lResult = MMC_FAILED;
        unsigned int ulErrorCode = 0;
        bool is_joint0_zero = false;
        bool is_joint1_zero = false;
        std::cout << "set vel mode" << std::endl;

        if(VCS_ActivateProfileVelocityMode(g_pKeyHandle, g_usNodeId1, &ulErrorCode) == MMC_FAILED)
        {
            LogError("VCS_ActivateProfileVelocityMode j0", lResult, ulErrorCode);
            lResult = MMC_FAILED;
        }
        if(VCS_ActivateProfileVelocityMode(subKeyHandle, g_usNodeId2, &ulErrorCode) == MMC_FAILED)
        {
            LogError("VCS_ActivateProfileVelocityMode j1", lResult, ulErrorCode);
            lResult = MMC_FAILED;
        }
        long targetvelocity = -200;
        if(VCS_MoveWithVelocity(g_pKeyHandle, g_usNodeId1, targetvelocity, &ulErrorCode) == MMC_FAILED)
		{
			LogError("VCS_MoveWithVelocity j0", lResult, ulErrorCode);
            lResult = MMC_FAILED;
		}
        if(VCS_MoveWithVelocity(subKeyHandle, g_usNodeId2, targetvelocity, &ulErrorCode) == MMC_FAILED)
		{
			LogError("VCS_MoveWithVelocity j1", lResult, ulErrorCode);
            lResult = MMC_FAILED;
		}

        homeArdu.readdata[0] = 0;
        homeArdu.readdata[1] = 0;
        std::cout <<"move and wait for sensor...." <<std::endl;
        while (!lResult)
        {
            homeArdu.read(&homeArdu.readdata[0],6);
            if(homeArdu.readdata[0]==1 && !is_joint0_zero){
                std::cout <<"123456:" << homeArdu.readdata[0] <<":654321" <<std::endl;
                if(VCS_HaltVelocityMovement(g_pKeyHandle, g_usNodeId1, &ulErrorCode) == MMC_FAILED)
                {
                    LogError("VCS_HaltVelocityMovement", lResult, ulErrorCode);
                    lResult = MMC_FAILED;
                }
                std::cout << "set j0 home mode" << std::endl;

                /*if(VCS_ActivateProfilePositionMode(g_pKeyHandle, g_usNodeId1, &ulErrorCode) == MMC_FAILED)
                {
                    LogError("VCS_ActivateProfilePositionMode", lResult, ulErrorCode);
                    lResult = MMC_FAILED;
                }
                

                if(VCS_SetPositionProfile(g_pKeyHandle, g_usNodeId1, 1000, 200, 200, &ulErrorCode) == MMC_FAILED)
                {
                    LogError("VCS_SetPositionProfile", lResult, ulErrorCode);
                    lResult = MMC_FAILED;
                }

                if(VCS_MoveToPosition(g_pKeyHandle, g_usNodeId1, 42250, 0, 1, &ulErrorCode) == MMC_FAILED)
                {
                    LogError("VCS_MoveToPosition", lResult, ulErrorCode);
                    lResult = MMC_FAILED;
                } 

                sleep(10);*/

                if(VCS_ActivateHomingMode(g_pKeyHandle, g_usNodeId1, &ulErrorCode) == MMC_FAILED)
                {
                    LogError("VCS_ActivateHomingMode", lResult, ulErrorCode);
                    lResult = MMC_FAILED;
                }

                if(VCS_FindHome(g_pKeyHandle, g_usNodeId1, HM_ACTUAL_POSITION, &ulErrorCode) == MMC_FAILED)
                {
                    LogError("VCS_ActivateHomingMode", lResult, ulErrorCode);
                    lResult = MMC_FAILED;
                }

                is_joint0_zero = true;
            }
            if(homeArdu.readdata[1]==1 && !is_joint1_zero){
                std::cout <<"666666:" << homeArdu.readdata[1] <<":666666" <<std::endl;
                if(VCS_HaltVelocityMovement(subKeyHandle, g_usNodeId2, &ulErrorCode) == MMC_FAILED)
                {
                    LogError("VCS_HaltVelocityMovement", lResult, ulErrorCode);
                    lResult = MMC_FAILED;
                }



                if(VCS_ActivateHomingMode(subKeyHandle, g_usNodeId2, &ulErrorCode) == MMC_FAILED)
                {
                    LogError("VCS_ActivateHomingMode", lResult, ulErrorCode);
                    lResult = MMC_FAILED;
                }

                if(VCS_FindHome(subKeyHandle, g_usNodeId2, HM_ACTUAL_POSITION, &ulErrorCode) == MMC_FAILED)
                {
                    LogError("VCS_ActivateHomingMode", lResult, ulErrorCode);
                    lResult = MMC_FAILED;
                }
                
                std::cout << "set j1 home mode" << std::endl;
                is_joint1_zero = true;
            }
            
            if(is_joint0_zero && is_joint1_zero){
                lResult = MMC_SUCCESS;
            }
        }
        
        if(lResult == MMC_SUCCESS){
            homingCompletedStatus = MMC_SUCCESS;
        }
        std::cout << "end auto home" << std::endl;
        
        return lResult;
    }

    int EposCommunication::startProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int p_profile_velocity, unsigned int p_profile_acceleration,unsigned int p_profile_deceleration){
        int lResult = MMC_SUCCESS;
	    unsigned int ulErrorCode = 0;

        g_profile_velocity = p_profile_velocity;
        g_profile_acceleration = p_profile_acceleration;
        g_profile_deceleration = p_profile_deceleration;

        if(ActivateProfilePositionMode(p_DeviceHandle,p_usNodeId,&ulErrorCode) == MMC_FAILED){
            LogInfo("activate PPM fail");
        }

        if(SetPositionProfile(p_DeviceHandle, p_usNodeId, &ulErrorCode) == MMC_FAILED){
            LogInfo("set PPM fail");
        }else{
            PPMStatus = MMC_SUCCESS;
        }

        return lResult;
    }

    int EposCommunication::setPosition(HANDLE p_DeviceHandle, unsigned short p_usNodeId, float position_setpoint){
        int lResult = MMC_SUCCESS;
	    unsigned int ulErrorCode = 0;
        long position_QC = 0;

        if(p_usNodeId == g_usNodeId1){
            position_QC = mToQC(position_setpoint);
        }else{
            position_QC = radToQC(p_usNodeId, position_setpoint);
        }

        if(SetPosition(p_DeviceHandle, p_usNodeId, position_QC, &ulErrorCode) == MMC_FAILED){
            LogInfo("set position fail");
            lResult = MMC_FAILED;
        }else{
			//LogInfo("SetPosition executed.");
		}

        return lResult;
    }

    int EposCommunication::stopPosition(HANDLE p_DeviceHandle, unsigned short p_usNodeId){
        int lResult = MMC_SUCCESS;
	    unsigned int ulErrorCode = 0;

        if(StopPosition(p_DeviceHandle, p_usNodeId, &ulErrorCode) == MMC_FAILED){
            LogInfo("stop position fail");
            lResult = MMC_FAILED;
        }
        return lResult;
    }

    int EposCommunication::getPosition(unsigned short p_usNodeId, double* pPositionIs){
        int lResult = MMC_FAILED;
        unsigned int ulErrorCode = 0;
        int pPositionIsCounts = 0;

        if((lResult = GetPosition(p_usNodeId, &pPositionIsCounts, &ulErrorCode))==MMC_FAILED)
        {
            LogError("getPosition", lResult, ulErrorCode);
            return lResult;
        }
        if(p_usNodeId == g_usNodeId1){
            *pPositionIs = QCTom(&pPositionIsCounts);
        }else{
            *pPositionIs = QCTorad(p_usNodeId, &pPositionIsCounts);
        }
        
        

        //only for Debugging
        //ROS_INFO_STREAM("!!! pPositionIs: " << *pPositionIs << " pPositionIsCounts: " << pPositionIsCounts);
        return lResult;
    }

    int EposCommunication::getVelocity(unsigned short p_usNodeId, double* pVelocityIs){
        int lResult = MMC_FAILED;
        unsigned int ulErrorCode = 0;
        int pVelocityIsCounts;

        if((lResult = GetVelocity(p_usNodeId, &pVelocityIsCounts, &ulErrorCode))==MMC_FAILED)
        {
            LogError("getVelocity", lResult, ulErrorCode);
            return lResult;
        }

        if(p_usNodeId == g_usNodeId1){
            *pVelocityIs = RPMTom_s(&pVelocityIsCounts);
        }else{
            *pVelocityIs = RPMTorad_s(p_usNodeId, &pVelocityIsCounts);
        }


        return lResult;
    }

    int EposCommunication::getVelocityUnit(){
        int lResult = MMC_SUCCESS;
        unsigned char dimension;
        char notation;
        unsigned int ulErrorCode = 0;
        if(VCS_GetVelocityUnits(g_pKeyHandle,g_usNodeId1,&dimension,&notation,&ulErrorCode) == MMC_FAILED){
            LogError("VCS_GetVelocityUnits", lResult, ulErrorCode);
            lResult = MMC_FAILED;
        }
        std::cout << "dimension: " << int(dimension) << std::endl;
        std::cout << "notation: " << int(notation) << std::endl;

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
        msg << "node id             = " << g_usNodeId1 << std::endl;
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
        //g_usNodeId1 = 1;
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

#ifdef UseSubDevice
        char* psubDeviceName = new char[255];
	    char* psubProtocolStackName = new char[255];
	    strcpy(psubDeviceName, subDeviceName.c_str());
	    strcpy(psubProtocolStackName, subProtocolStackName.c_str());
#endif

        strcpy(pDeviceName, g_deviceName.c_str());
        strcpy(pProtocolStackName, g_protocolStackName.c_str());
        strcpy(pInterfaceName, g_interfaceName.c_str());
        strcpy(pPortName, g_portName.c_str());

        LogInfo("Open device...");

        g_pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);

#ifdef UseSubDevice
        LogInfo("Open SUB device...");
        subKeyHandle = VCS_OpenSubDevice(g_pKeyHandle, psubDeviceName, psubProtocolStackName, p_pErrorCode);
        if(g_pKeyHandle!=0 && *p_pErrorCode == 0 && subKeyHandle !=0)
#else
        if(g_pKeyHandle!=0 && *p_pErrorCode == 0)
#endif
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
            subKeyHandle = 0;
            std::cout << "############ device open fail ############" << std::endl;
        }

        delete []pDeviceName;
        delete []pProtocolStackName;
        delete []pInterfaceName;
        delete []pPortName;

#ifdef UseSubDevice
        delete []psubDeviceName;
    	delete []psubProtocolStackName;
#endif

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
#ifdef UseSubDevice
        LogInfo("Close SUB device");

	    if(VCS_CloseSubDevice(subKeyHandle, p_pErrorCode)!=0 && *p_pErrorCode == 0)
	    {
	    	lResult = MMC_SUCCESS;
	    }
#endif
        LogInfo("Close device");

        if(VCS_CloseDevice(g_pKeyHandle, p_pErrorCode)!=MMC_FAILED && *p_pErrorCode == 0)
        {
            lResult = MMC_SUCCESS;
        }

        return lResult;
    }

    int EposCommunication::PrepareEpos(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode){
        int lResult = MMC_SUCCESS;
        BOOL oIsFault = 0; //0 is not in fault state

        if(VCS_GetFaultState(p_DeviceHandle, p_usNodeId, &oIsFault, p_pErrorCode ) == MMC_FAILED)
        {
            LogError("VCS_GetFaultState", lResult, *p_pErrorCode);
            lResult = MMC_FAILED;
        }

        std::cout << "Debug 1: FaultState:" << oIsFault << std::endl;

        if(oIsFault)
        {
            std::stringstream msg;
            msg << "clear fault, node = '" << p_usNodeId << "'";
            LogInfo(msg.str());

            if(VCS_ClearFault(p_DeviceHandle, p_usNodeId, p_pErrorCode) == MMC_FAILED)
            {
                LogError("VCS_ClearFault", lResult, *p_pErrorCode);
                lResult = MMC_FAILED;
            }
        }

        BOOL oIsEnabled = 0;

        if(VCS_GetEnableState(p_DeviceHandle, p_usNodeId, &oIsEnabled, p_pErrorCode) == MMC_FAILED)
        {
            LogError("VCS_GetEnableState", lResult, *p_pErrorCode);
            lResult = MMC_FAILED;
        }

        if(!oIsEnabled)
        {
            if(VCS_SetEnableState(p_DeviceHandle, p_usNodeId, p_pErrorCode) == MMC_FAILED)
            {
                LogError("VCS_SetEnableState", lResult, *p_pErrorCode);
                lResult = MMC_FAILED;
            }
            else{
                VCS_GetEnableState(p_DeviceHandle, p_usNodeId, &oIsEnabled, p_pErrorCode);
                std::cout << "SetEnableState should be 1:" <<  oIsEnabled << std::endl;
            }
        }

        return lResult;
    }

    int EposCommunication::DisableEpos(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode){
        int lResult = MMC_SUCCESS;

        *p_pErrorCode = 0;

        if(VCS_SetDisableState(p_DeviceHandle, p_usNodeId, p_pErrorCode) == MMC_FAILED)
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
            LogInfo("VCS_ActivateProfilePositionMode successfull.");
        }
        return lResult;
    }

    int EposCommunication::SetPositionProfile(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode)
    {
        //to use set variables below first!
        int lResult = MMC_SUCCESS;

        if(VCS_SetPositionProfile(p_DeviceHandle, p_usNodeId, g_profile_velocity, g_profile_acceleration, g_profile_deceleration, p_pErrorCode) == MMC_FAILED)
        {
            LogError("VCS_SetPositionProfile", lResult, *p_pErrorCode);
            lResult = MMC_FAILED;
        }

        return lResult;
    }

    int EposCommunication::SetPosition(HANDLE p_DeviceHandle, unsigned short p_usNodeId, long position_setpoint, unsigned int* p_pErrorCode){
        // absolute position, starts immediately
        int lResult = MMC_SUCCESS;
        //std::stringstream msg;

        //msg << "move to position = " << position_setpoint << ", node = " << p_usNodeId;
        //LogInfo(msg.str());

        if(VCS_MoveToPosition(p_DeviceHandle, p_usNodeId, position_setpoint, 1, 1, p_pErrorCode) == MMC_FAILED)
            {
                LogError("VCS_MoveToPosition", lResult, *p_pErrorCode);
                lResult = MMC_FAILED;
            } 
        else{
            //LogInfo("Movement executed.");
        }

        return lResult;
    }

    int EposCommunication::StopPosition(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode){
        int lResult = MMC_SUCCESS;
        if(VCS_HaltPositionMovement(p_DeviceHandle, p_usNodeId, p_pErrorCode) == MMC_FAILED){
            LogError("VCS_HaltPositionMovement", lResult, *p_pErrorCode);
            lResult = MMC_FAILED;
        }
        return lResult;
    }

    int EposCommunication::GetPosition(unsigned short p_usNodeId, int* pPositionIsCounts, unsigned int* p_pErrorCode){
        int lResult = MMC_SUCCESS;
        if(p_usNodeId == g_usNodeId1){
            if(VCS_GetPositionIs(g_pKeyHandle, p_usNodeId, pPositionIsCounts, p_pErrorCode) == MMC_FAILED)
            {
                LogError("VCS_GetPositionIs", lResult, *p_pErrorCode);
                lResult = MMC_FAILED;
            }
        }else{
            if(VCS_GetPositionIs(subKeyHandle, p_usNodeId, pPositionIsCounts, p_pErrorCode) == MMC_FAILED)
            {
                LogError("VCS_GetPositionIs", lResult, *p_pErrorCode);
                lResult = MMC_FAILED;
            }
        }
        return lResult;
    }

    int EposCommunication::GetVelocity(unsigned short p_usNodeId, int* pVelocityIsCounts, unsigned int* p_pErrorCode){
        int lResult = MMC_SUCCESS;
        if(p_usNodeId == g_usNodeId1){
            if(VCS_GetVelocityIs(g_pKeyHandle, p_usNodeId, pVelocityIsCounts, p_pErrorCode) == MMC_FAILED)
            {
                LogError("VCS_GetVelocityIs", lResult, *p_pErrorCode);
                lResult = MMC_FAILED;
            }
        }
        else{
            if(VCS_GetVelocityIs(subKeyHandle, p_usNodeId, pVelocityIsCounts, p_pErrorCode) == MMC_FAILED)
            {
                LogError("VCS_GetVelocityIs", lResult, *p_pErrorCode);
                lResult = MMC_FAILED;
            }
        }
        return lResult;
    }

    long EposCommunication::mToQC(double m){
        return (long)(m*90000);//slide rail (90000 QC/m)
    }

    double EposCommunication::QCTom(int* QC){
        double top = (double)*QC;
        return top/90000.0;
    }

    double EposCommunication::RPMTom_s(int* RPM){
        double top = (double)*RPM;
        return top*2048.0/(90000.0 * 60.0);
    }

    long EposCommunication::radToQC(unsigned short p_usNodeId, double rad){
        if(p_usNodeId == g_usNodeId2){
            return (long)(rad*17303);//joint 1 (17303 QC/rad)
        }else if(p_usNodeId == g_usNodeId3){
            return (long)(rad*17360);//joint 2 (17360 QC/rad)
        }else if(p_usNodeId == g_usNodeId4){
            return (long)(rad*17303);//joint 3 (17303 QC/rad)
        }else if(p_usNodeId == g_usNodeId5){
            return (long)(rad*45034);//joint 4 (45034 QC/rad)
        }else{
            return 0;
        }
    }
    
    double EposCommunication::QCTorad(unsigned short p_usNodeId, int* QC){
        double top = (double)*QC;
        if(p_usNodeId == g_usNodeId2){
            return top/17303.0;
        }else if(p_usNodeId == g_usNodeId3){
            return top/17360.0;
        }else if(p_usNodeId == g_usNodeId4){
            return top/17303.0;
        }else if(p_usNodeId == g_usNodeId5){
            return top/45034.0;
        }else{
            return 0.0;
        }
    }

    double EposCommunication::RPMTorad_s(unsigned short p_usNodeId, int* RPM){
        double top = (double)*RPM;
        if(p_usNodeId == g_usNodeId2){
            return top*2048.0/(17303.0 * 60.0);
        }else if(p_usNodeId == g_usNodeId3){
            return top*2048.0/(17360.0 * 60.0);
        }else if(p_usNodeId == g_usNodeId4){
            return top*2048.0/(17303.0 * 60.0);
        }else if(p_usNodeId == g_usNodeId5){
            return top*2048.0/(45034.0 * 60.0);
        }else{
            return 0.0;
        }
    }


}