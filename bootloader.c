/***********includes*************/

#include "bootloader.h"
/***********static functions decleration*************/

static void BootLoader_Get_Version (uint8_t *HostBuffer);
static void BootLoader_Get_Help (uint8_t *HostBuffer);
static void BootLoader_Get_chip_identification_number (uint8_t *HostBuffer);
static void BootLoader_Read_Protection_Level (uint8_t *HostBuffer);
static void BootLoader_Jump_To_Address (uint8_t *HostBuffer);
static void BootLoader_Erase_Flash (uint8_t *HostBuffer);
static void BootLoader_Memory_Write (uint8_t *HostBuffer);
static void BootLoader_Enable_RW_Protection (uint8_t *HostBuffer);
static void BootLoader_Memory_Read (uint8_t *HostBuffer);
static void BootLoader_Get_Sector_Protection_Status (uint8_t *HostBuffer);
static void BootLoader_Read_OTP (uint8_t *HostBuffer);
static void BootLoader_Disable_RW_Protection (uint8_t *HostBuffer);
static void BootLoader_Change_Read_Protection_Level (uint8_t *HostBuffer);

static uint8_t BL_CRC_Verify (uint8_t *BData , uint32_t dataLen , uint32_t Host_CRC);
static void BL_Send_Ack (uint8_t message_len);
static void BL_Send_Nack ();
static uint8_t Host_Jump_Address_Verification (uint32_t);
static void jump_to_user_app();
static uint8_t flash_memory_write_cmd (uint8_t * host_payload , uint32_t address , uint16_t payload_len );
static uint8_t CBL_stm32f407_Read_protection_level();
static uint8_t Change_ROP_Level (uint32_t ROP_Level);
/***********global variables definitions*************/

static uint8_t BL_HOST_BUFFER [BL_HOST_BUFFER_RX_LENGH];  // bootloader Bufer 

static uint8_t BL_Supported_CMDS[12] = {
 CBL_GET_VER_CMD,               
 CBL_GET_HELP_CMD,						
 CBL_GET_CID_CMD,								
 CBL_GET_RDP_STATUS_CMD,		  	
 CBL_GO_TO_ADDR_CMD,					  
 CBL_FLASH_ERASE_CMD,						
 CBL_MEM_WR_CMD,							
 CBL_EN_R_W_PROTECT_CMD,			
 CBL_MEM_READ_CMD,						
 CBL_READ_SECTOR_STATUS_CMD,	
 CBL_OTP_READ_CMD,						
 CBL_CHNG_RPL_CMD,			
};
/********** software interfaces definitions****************/

BL_STATUS BL_UART_Host_Fetch_Command (void)
{
	BL_STATUS status = BL_NACK;
	HAL_StatusTypeDef Status= HAL_ERROR;
	uint8_t dataLength = 0;
	
	/*clear the buffer every new host command*/
	memset(BL_HOST_BUFFER , 0 ,BL_HOST_BUFFER_RX_LENGH);
	/*Receive the first host command packet (byte)*/
	Status = HAL_UART_Receive(BL_RX_HOST_PACKET ,BL_HOST_BUFFER,1,HAL_MAX_DELAY );
	
	if(Status != HAL_OK){
		status = BL_NACK;
	}
	else{
		/*Receive the host command packet */
		dataLength = BL_HOST_BUFFER[0];
	  Status = HAL_UART_Receive(BL_RX_HOST_PACKET ,&BL_HOST_BUFFER[1],dataLength,HAL_MAX_DELAY );
		if(Status != HAL_OK){
		status = BL_NACK;
	}
	else{
		switch(BL_HOST_BUFFER[1]){
			case CBL_GET_VER_CMD:
				BootLoader_Get_Version(BL_HOST_BUFFER);
			break;
			case CBL_GET_HELP_CMD:
				BootLoader_Get_Help(BL_HOST_BUFFER);
			break;
			case CBL_GET_CID_CMD:
				BootLoader_Get_chip_identification_number(BL_HOST_BUFFER);
			break;
			case CBL_GET_RDP_STATUS_CMD:
				BootLoader_Read_Protection_Level(BL_HOST_BUFFER);
			break;	
			case CBL_GO_TO_ADDR_CMD:
				BootLoader_Jump_To_Address(BL_HOST_BUFFER);
			break;	
			case CBL_FLASH_ERASE_CMD:
				BootLoader_Erase_Flash(BL_HOST_BUFFER);
			break;	
			case CBL_MEM_WR_CMD:
				BootLoader_Memory_Write(BL_HOST_BUFFER);
			break;	
			case CBL_CHNG_RPL_CMD:
				BootLoader_Change_Read_Protection_Level(BL_HOST_BUFFER);
			break;	
			default:BL_print_message("invalid host command!!");break;
		}
	}
	}
	
	return status;
}

void BL_print_message (char *format , ...)
{
	 char message[100]= {0};
	 va_list args;
	 va_start (args,format);
	 vsprintf(message ,format , args );
#if (BL_DEBUG_METHOD == BL_ENABLE_UART_DEBUG_MESSAGE)	 
	 HAL_UART_Transmit(BL_Debug_UART ,(uint8_t *)message,sizeof(message),HAL_MAX_DELAY );
#elif (BL_DEBUG_METHOD == BL_ENABLE_SPI_DEBUG_MESSAGE)	

#elif (BL_DEBUG_METHOD == BL_ENABLE_CAN_DEBUG_MESSAGE)

	 
#endif	 
	 va_end(args);
}

/***********static functions definitions*************/
static void jump_to_user_app(){
/*value of msp of main application */
	uint32_t MSP_Value = *((volatile uint32_t *)(FLASH_SECTOR2_BASE_ADDRESS));
	
/*value of resetHandler of main application */	
	uint32_t mainappADD = *((volatile uint32_t *)(FLASH_SECTOR2_BASE_ADDRESS + 4));
	
	void(*PmainApp)(void) = (void *)mainappADD;
	
	/*Set Main Stack Pointer*/
	__set_MSP(MSP_Value);
	
	/*Deinitialization Of Modules*/
	HAL_RCC_DeInit(); /*Resets the RCC clock configuration to the default reset state.*/
	
	/*Jump To Application Reset Handler*/
	PmainApp();
}
static uint8_t BL_CRC_Verify (uint8_t *BData , uint32_t dataLen , uint32_t Host_CRC){
	uint8_t crc_status = CRC_FAILED;
	uint32_t CRC_MCU_Calculated =0;
	uint8_t dataCounter =0;
	uint32_t bufferData = 0;
	
	/*calculate CRC32*/
	for(dataCounter =0; dataCounter < dataLen; dataCounter++){
		bufferData = (uint32_t)BData[dataCounter];
		CRC_MCU_Calculated = HAL_CRC_Accumulate(BL_CRC_OBJ ,&bufferData,1);
	}
	/*Reset the CRC calculate unit*/
	__HAL_CRC_DR_RESET(BL_CRC_OBJ);
	
	/*compare the host CRC with the MCU CRC*/
	if(CRC_MCU_Calculated == Host_CRC){
		crc_status = CRC_PASSED;
	}
	else{
		crc_status = CRC_FAILED;
	}
	return crc_status;
}

static void BL_Send_Ack (uint8_t message_len){
	uint8_t ack_value[2]={0};
	ack_value[0] = CBL_SEND_ACK;
	ack_value[1] = message_len;
	HAL_UART_Transmit (BL_RX_HOST_PACKET ,(uint8_t *)ack_value,2,HAL_MAX_DELAY );
}
static void BL_Send_Nack (){
	uint8_t ack_value = CBL_SEND_NACK;
	HAL_UART_Transmit (BL_RX_HOST_PACKET ,&ack_value,1,HAL_MAX_DELAY );
}
static uint8_t Host_Jump_Address_Verification (uint32_t jump_address){
	uint8_t address_verification = ADDRESS_IS_INVALID;
	if(jump_address >= SRAM1_BASE  && jump_address <= STM32F407_SRAM1_END ){
		address_verification = ADDRESS_IS_VALID;
	}
	else 	if(jump_address >= SRAM2_BASE  && jump_address <= STM32F407_SRAM2_END ){
		address_verification = ADDRESS_IS_VALID;
	}
	else 	if(jump_address >= CCMDATARAM_BASE  && jump_address <= STM32F407_SRAM3_END ){
		address_verification = ADDRESS_IS_VALID;
	}
	else 	if(jump_address >= FLASH_BASE  && jump_address <= STM32F407_FLASH_END ){
		address_verification = ADDRESS_IS_VALID;
	}	
	else{
		address_verification = ADDRESS_IS_INVALID;
	}
	
	return address_verification;
}


static uint8_t Perform_Flash_Erase (uint8_t sector_number , uint8_t number_of_sectors){
  uint8_t sector_validity_status = INVALID_SECTOR_NUMBER;
	FLASH_EraseInitTypeDef pflashErase;
	uint8_t RemainigSectors=0;
	HAL_StatusTypeDef HAL_Status = HAL_ERROR;
	uint32_t sector_error = 0;
	
	if (number_of_sectors > FLASH_SECTOR_MAX_SIZE ){
	 sector_validity_status = INVALID_SECTOR_NUMBER;
}
 else {
  if((sector_number <= FLASH_SECTOR_MAX_SIZE - 1) || (sector_number == CBL_FLASH_MASS_ERASE)){
		if((sector_number == CBL_FLASH_MASS_ERASE)){
			/*flash mass erase*/
			pflashErase.TypeErase = FLASH_TYPEERASE_MASSERASE;
		}
		else {
			/*force to erace sectors within range*/
			RemainigSectors = FLASH_SECTOR_MAX_SIZE - sector_number;
			if(number_of_sectors > RemainigSectors){
			number_of_sectors = RemainigSectors ;
			} 
			else {/*Nothing*/}
		/*flash sector erase*/
			pflashErase.TypeErase = FLASH_TYPEERASE_SECTORS;
			pflashErase.Sector = sector_number;
			pflashErase.NbSectors = number_of_sectors;
		}
		
		pflashErase.Banks =FLASH_BANK_1 ; /*!< Bank 1   */
		pflashErase.VoltageRange = FLASH_VOLTAGE_RANGE_3; /*!< Device operating range: 2.7V to 3.6V*/
		
		/*unlock the flash control register */
		HAL_Status =HAL_FLASH_Unlock();
		/*perform mass or sector erase */
		HAL_Status = HAL_FLASHEx_Erase(&pflashErase ,&sector_error );
		if(HAL_SUCCESSFULLY_ERASED == sector_error){
			sector_validity_status = ERASE_SUCCESSFUL;
		}
		else {
			sector_validity_status =ERASE_UNSUCCESSFUL;
		}
		/*lock the flash control register */
		HAL_Status =HAL_FLASH_Lock();
	}
	else {
		sector_validity_status =ERASE_UNSUCCESSFUL;
	}
 }
 return sector_validity_status;
}
static uint8_t flash_memory_write_cmd(uint8_t *host_payload ,uint32_t address,uint16_t payload_len){
	
  HAL_StatusTypeDef HAL_Status =HAL_ERROR;
	uint16_t payload_counter =0;
	uint8_t flash_payload_write_status = FLASH_PAYLOAD_WRITE_FAILED;
	
	/*unlock the flash control register access*/
	HAL_Status = HAL_FLASH_Unlock();
		if(HAL_Status != HAL_OK){
		flash_payload_write_status = FLASH_PAYLOAD_WRITE_FAILED;
		}else{
			/*flash memory write byte_by_byte*/
			for( payload_counter =0;payload_counter <payload_len;payload_counter++){
				HAL_Status= HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE ,address + payload_counter ,host_payload[payload_counter]);
				if(HAL_Status != HAL_OK){
				flash_payload_write_status = FLASH_PAYLOAD_WRITE_FAILED;	
				break;
				}else{
				flash_payload_write_status = FLASH_PAYLOAD_WRITE_SUCCESS;
		}
	}
		}
		if((FLASH_PAYLOAD_WRITE_SUCCESS == flash_payload_write_status) && ( HAL_OK == HAL_Status)){
				/*lock the flash control register access*/
	      HAL_Status = HAL_FLASH_Lock();
			if(HAL_Status != HAL_OK){
				flash_payload_write_status = FLASH_PAYLOAD_WRITE_FAILED;
			}
			else {
				flash_payload_write_status = FLASH_PAYLOAD_WRITE_SUCCESS;
			}
				
		} else{flash_payload_write_status = FLASH_PAYLOAD_WRITE_FAILED;}

	return flash_payload_write_status;
}

static uint8_t CBL_stm32f407_Read_protection_level(){
	
	FLASH_OBProgramInitTypeDef  flash_OBProgram ;
  HAL_FLASHEx_OBGetConfig(&flash_OBProgram);
	
	return  (uint8_t)(flash_OBProgram.RDPLevel) ;
}

static uint8_t Change_ROP_Level (uint32_t ROP_Level){
	
	HAL_StatusTypeDef HAL_Status = HAL_ERROR;
	FLASH_OBProgramInitTypeDef OBProgram;
	uint8_t ROP_Level_Status =ROP_LEVEL_CHANGE_INVALID ;
	
	/*unlock the option flash control register access*/
	HAL_Status = HAL_FLASH_OB_Unlock();
	if(HAL_Status != HAL_OK){
		ROP_Level_Status =ROP_LEVEL_CHANGE_INVALID ;
	}else{
	/*program option bytes*/
		OBProgram.OptionType = OPTIONBYTE_RDP ;
		OBProgram.Banks = FLASH_BANK_1;
		OBProgram.RDPLevel = ROP_Level;
	  HAL_Status = HAL_FLASHEx_OBProgram(&OBProgram);
		if(HAL_Status != HAL_OK){
		  ROP_Level_Status =ROP_LEVEL_CHANGE_INVALID ;
	}else{
	/*Launch option bytes loading*/
	  HAL_Status = HAL_FLASH_OB_Launch();
		if(HAL_Status != HAL_OK){
		  ROP_Level_Status =ROP_LEVEL_CHANGE_INVALID ;
	}else{
	/*lock the option flash control register access*/
	  HAL_Status =HAL_FLASH_OB_Lock();
		if(HAL_Status != HAL_OK){
		  ROP_Level_Status =ROP_LEVEL_CHANGE_INVALID ;
	}else{
			ROP_Level_Status =ROP_LEVEL_CHANGE_VALID ;
	}
	}
	}
	}
	return ROP_Level_Status;
}

static void BootLoader_Get_Version (uint8_t *HostBuffer)
{
	uint8_t BL_Version[4]={CBL_VENDOR_ID,CBL_SW_MAJOR_VERSION,CBL_SW_MINOR_VERSION,CBL_SW_PATCH_VERSION};
	uint32_t BL_package_len = 0;
	uint32_t BL_CRC32 = 0;
	
	/*Extract the package length and CRC32 sent by host */
	BL_package_len = HostBuffer[0] + 1;
	BL_CRC32 = *((uint32_t *)((HostBuffer +BL_package_len )-4));
	
	/*CRC verification */
	if(CRC_PASSED == BL_CRC_Verify(HostBuffer,BL_package_len - 4,BL_CRC32)){
		BL_print_message("CRC verification passed \r\n");
		BL_Send_Ack(4);
		HAL_UART_Transmit (BL_RX_HOST_PACKET ,(uint8_t *)BL_Version,4,HAL_MAX_DELAY );
	}
	else{
		BL_print_message("CRC verification failed \r\n");
		BL_Send_Nack();
	}
}
static void BootLoader_Get_Help (uint8_t *HostBuffer)
{
	uint32_t BL_package_len = 0;
	uint32_t BL_CRC32 = 0;
	BL_print_message("Read the commands supported by the bootloader \r\n");
	
	/*Extract the package length and CRC32 sent by host */
	BL_package_len = HostBuffer[0] + 1;
	BL_CRC32 = *((uint32_t *)((HostBuffer +BL_package_len )-4));
	
	/*CRC verification */
	if(CRC_PASSED == BL_CRC_Verify(HostBuffer,BL_package_len - 4,BL_CRC32)){
		BL_print_message("CRC verification passed \r\n");
		BL_Send_Ack(12);
		HAL_UART_Transmit (BL_RX_HOST_PACKET ,BL_Supported_CMDS,12,HAL_MAX_DELAY );
	}
	else{
		BL_print_message("CRC verification failed \r\n");
		BL_Send_Nack();
	}

}
static void BootLoader_Get_chip_identification_number (uint8_t *HostBuffer)
{
	uint32_t BL_package_len = 0;
	uint32_t BL_CRC32 = 0;
	uint16_t MCU_ID = 0;
	
	BL_print_message("Read the MCU chip identification number \r\n");
	/*Extract the package length and CRC32 sent by host */
	BL_package_len = HostBuffer[0] + 1;
	BL_CRC32 = *((uint32_t *)((HostBuffer +BL_package_len )-4));
	/*CRC verification */
	if(CRC_PASSED == BL_CRC_Verify(HostBuffer,BL_package_len - 4,BL_CRC32)){
		BL_print_message("CRC verification passed \r\n");
		/*Get the MCU chip identification number */
		MCU_ID = (uint16_t)((DBGMCU->IDCODE) & 0x00000FFF);
		/*Report the MCU chip identification number */
		BL_Send_Ack(2);
		HAL_UART_Transmit (BL_RX_HOST_PACKET ,(uint8_t *)&MCU_ID,2,HAL_MAX_DELAY );
	}
	else{
		BL_print_message("CRC verification failed \r\n");
		BL_Send_Nack();
	}
}	
static void BootLoader_Read_Protection_Level (uint8_t *HostBuffer)
{
	uint32_t BL_package_len = 0;
	uint32_t BL_CRC32 = 0;
	uint8_t RDP_level = 0;
	
	BL_print_message(" bootloader read the flash protection level \r\n");
	/*Extract the package length and CRC32 sent by host */
	BL_package_len = HostBuffer[0] + 1;
	BL_CRC32 = *((uint32_t *)((HostBuffer +BL_package_len )-4));
	
	/*CRC verification */
	if(CRC_PASSED == BL_CRC_Verify(HostBuffer,BL_package_len - 4,BL_CRC32)){
		BL_print_message("CRC verification passed \r\n");
	/*Read the flash Protection Level */
		 RDP_level = CBL_stm32f407_Read_protection_level();
		/* Report the flash Read Protection Level */	
		BL_Send_Ack(1);
		HAL_UART_Transmit (BL_RX_HOST_PACKET ,(uint8_t *)&RDP_level,1,HAL_MAX_DELAY );	
	}
	else{
		BL_print_message("CRC verification failed \r\n");
		BL_Send_Nack();
	}
}	
static void BootLoader_Jump_To_Address (uint8_t *HostBuffer)
{
	uint32_t BL_package_len = 0;
	uint32_t BL_CRC32 = 0;
	uint32_t Host_Jump_Address = 0;
	uint8_t address_verification = ADDRESS_IS_INVALID;
	
	BL_print_message("jump bootloader to a specific address \r\n");
	/*Extract the package length and CRC32 sent by host */
	BL_package_len = HostBuffer[0] + 1;
	BL_CRC32 = *((uint32_t *)((HostBuffer +BL_package_len )-4));
	/*CRC verification */
	if(CRC_PASSED == BL_CRC_Verify(HostBuffer,BL_package_len - 4,BL_CRC32)){
		BL_print_message("CRC verification passed \r\n");
		
	/*	Extract the Address From The Host Packet*/
		Host_Jump_Address = *((uint32_t *)&HostBuffer[2]);
		
	/*verify the extracted address from host within range or not */
		address_verification = Host_Jump_Address_Verification(Host_Jump_Address);
	if(ADDRESS_IS_VALID == address_verification){
		BL_print_message("address verification succeeded \r\n");
		BL_Send_Ack(1);
		HAL_UART_Transmit (BL_RX_HOST_PACKET ,(uint8_t *)&address_verification,1,HAL_MAX_DELAY );	
		
	/*prepare the address for jump*/
		jump_ptr jump_address = (jump_ptr)(Host_Jump_Address + 1); // +1 cause of set t_bit
		jump_address();
	}
	else{
		BL_print_message("address verification failed \r\n");
		HAL_UART_Transmit (BL_RX_HOST_PACKET ,(uint8_t *)&address_verification,1,HAL_MAX_DELAY );
	}
}
	else{
		BL_print_message("CRC verification failed \r\n");
		BL_Send_Nack();
	}
}	
static void BootLoader_Erase_Flash (uint8_t *HostBuffer)
{
	uint32_t BL_package_len = 0;
	uint32_t BL_CRC32 = 0;
	uint8_t Erase_Status = ERASE_UNSUCCESSFUL;
	
	BL_print_message(" bootloader erase the flash sectors \r\n");
	/*Extract the package length and CRC32 sent by host */
	BL_package_len = HostBuffer[0] + 1;
	BL_CRC32 = *((uint32_t *)((HostBuffer +BL_package_len )-4));
	/*CRC verification */
	if(CRC_PASSED == BL_CRC_Verify(HostBuffer,BL_package_len - 4,BL_CRC32)){
		BL_print_message("CRC verification passed \r\n");
	/*	Extract the sectors From The Host Packet*/

	/*verify the extracted sectors from host within range or not */
	Erase_Status = Perform_Flash_Erase(HostBuffer[2] , HostBuffer[3]);
	if(ERASE_SUCCESSFUL == Erase_Status){
		BL_print_message("Erase  succeeded \r\n");
		BL_Send_Ack(1);
		HAL_UART_Transmit (BL_RX_HOST_PACKET ,(uint8_t *)&Erase_Status,1,HAL_MAX_DELAY );	
	}
	else{
		BL_print_message("Erase  failed \r\n");
		HAL_UART_Transmit (BL_RX_HOST_PACKET ,(uint8_t *)&Erase_Status,1,HAL_MAX_DELAY );
	}
}
	else{
		BL_print_message("CRC verification failed \r\n");
		BL_Send_Nack();
	}
}	
static void BootLoader_Memory_Write (uint8_t *HostBuffer)
{
	uint32_t BL_package_len = 0;
	uint32_t BL_CRC32 = 0;
	uint32_t host_address = 0;
	uint8_t payload_len = 0;
	uint8_t address_verification = INVALID_ADDRESS;
	uint8_t flash_payload_write_status = FLASH_PAYLOAD_WRITE_FAILED;
	
	BL_print_message(" bootloader write into the flash sectors \r\n");
	/*Extract the package length and CRC32 sent by host */
	BL_package_len = HostBuffer[0] + 1;
	BL_CRC32 = *((uint32_t *)((HostBuffer +BL_package_len )-4));
	
	/*CRC verification */
	if(CRC_PASSED == BL_CRC_Verify(HostBuffer,BL_package_len - 4,BL_CRC32)){
		BL_print_message("CRC verification passed \r\n");
	/*Extract the base address From The Host Packet*/
	host_address = *((uint32_t *)&HostBuffer[2]);
	/*Extract the payload From The Host Packet*/
	payload_len = HostBuffer[6];
		
  /*verify the extracted address*/
		address_verification = Host_Jump_Address_Verification(host_address);
	if(VALID_ADDRESS == address_verification){
		BL_print_message("address is valid  \r\n");
		
		/*write the host payload*/
		flash_payload_write_status = flash_memory_write_cmd(&HostBuffer[7],host_address,payload_len);
		if(flash_payload_write_status == FLASH_PAYLOAD_WRITE_SUCCESS){
		/*Report the host */	
		BL_Send_Ack(1);
		HAL_UART_Transmit (BL_RX_HOST_PACKET ,(uint8_t *)&flash_payload_write_status,1,HAL_MAX_DELAY );
		}else{
		HAL_UART_Transmit (BL_RX_HOST_PACKET ,(uint8_t *)&flash_payload_write_status,1,HAL_MAX_DELAY );
		}
	}
	else{
		BL_print_message("address is invalid  \r\n");
		HAL_UART_Transmit (BL_RX_HOST_PACKET ,(uint8_t *)&address_verification,1,HAL_MAX_DELAY );
	}
}
	else{
		BL_print_message("CRC verification failed \r\n");
		BL_Send_Nack();
	}
}	
static void BootLoader_Enable_RW_Protection (uint8_t *HostBuffer)
{
	
}	
static void BootLoader_Memory_Read (uint8_t *HostBuffer)
{
	
}	
static void BootLoader_Get_Sector_Protection_Status (uint8_t *HostBuffer)
{
	
}	
static void BootLoader_Read_OTP (uint8_t *HostBuffer)
{
	
}	
static void BootLoader_Disable_RW_Protection (uint8_t *HostBuffer)
{
	
}	
static void BootLoader_Change_Read_Protection_Level (uint8_t *HostBuffer)
{
	uint32_t BL_package_len = 0;
	uint32_t BL_CRC32 = 0;
	uint8_t status = 0;
	uint8_t ROP_LEVEL = 0;
	BL_print_message(" bootloader program the flash protection level \r\n");
	/*Extract the package length and CRC32 sent by host */
	BL_package_len = HostBuffer[0] + 1;
	BL_CRC32 = *((uint32_t *)((HostBuffer +BL_package_len )-4));
	
	/*CRC verification */
	if(CRC_PASSED == BL_CRC_Verify(HostBuffer,BL_package_len - 4,BL_CRC32)){
		BL_print_message("CRC verification passed \r\n");
	/*program the flash Protection Level */
		ROP_LEVEL = HostBuffer[2];
		if(ROP_LEVEL == OB_RDP_LEVEL_2){
			status = ROP_LEVEL_CHANGE_INVALID;
		}
		else{
		 status = Change_ROP_Level(HostBuffer[2]);
		/* Report the flash Read Protection Level */	
		BL_Send_Ack(1);
		HAL_UART_Transmit (BL_RX_HOST_PACKET ,(uint8_t *)&status,1,HAL_MAX_DELAY );
		}
		 	
	}
	else{
		BL_print_message("CRC verification failed \r\n");
		BL_Send_Nack();
	}
}
	