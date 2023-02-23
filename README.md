# M253BSP_IAP_XMODEM_APROM
 M253BSP_IAP_XMODEM_APROM


update @ 2023/02/21

1. Add SW CRC32  , check define : ENABLE_SW_CRC32

2. check define : USE_SRAM_TABLE , USE_FLASH_TABLE , to use CRC32 table put in SRAM or FLASH

3. Below is flash allocation about ISP code , application code  (NOTE : M253 have no data flash)

![image](https://github.com/released/M253BSP_IAP_XMODEM_APROM/blob/main/flash_allocation.jpg)	

4. Scenario notice:

	- Boot loader project : ISP_UART 
	
		- PUT ISP code in APROM , address 0x00			
	
		- when power on , will check power on source (ex : power on reset , nReset , from application code)
	
		- use CRC to calculate Application code checksum (from START : 0x4800 , END ADDRESS : 0x20000-4 : 0x1FFFC )
		
		- load Application code checksum , from specific address (at 0x1FE00 last 4 bytes : 0x1FFFC)
		
		- if two checksum result are different , will stuck in Boot loader , and wait for teraterm Xmodem transfer data
		
![image](https://github.com/released/M253BSP_IAP_XMODEM_APROM/blob/main/LDROM_checksum_err.jpg)		
		
		- if boot from application code 

![image](https://github.com/released/M253BSP_IAP_XMODEM_APROM/blob/main/LDROM_boot_from_application_code.jpg)	

		- under teraterm Xmodem , when under transfer data bytes 
		
![image](https://github.com/released/M253BSP_IAP_XMODEM_APROM/blob/main/LDROM_xmodem_under_transfer.jpg)			

		- under teraterm Xmodem , when upgrade finish 
		
![image](https://github.com/released/M253BSP_IAP_XMODEM_APROM/blob/main/LDROM_xmodem_tranfer_finish.jpg)

		- if reset from application code , will entry timeout counting , jump to application code if not receive teraterm Xmodem transfer data		
	
	- Application code project : AP
	
		- use SRecord , to calculate application code checksum , add binary to hex , by SRecord tool
		
		- SRecord file : srec_cat.exe 

		- under generateChecksum.bat will execute generateChecksum.cmd , generateCRCbinary.cmd , generateCRChex.cmd
	
			- generateChecksum.cmd : calculate checksum by load the original binary file , and display on KEIL project
		
			- generateCRCbinary.cmd : calculate checksum by load the original binary file , and fill 0xFF , range up to 0x1B7FC
		
			- generateCRChex.cmd : conver binary file into hex file
		
		- check sum calculate will start from 0x4800 to 0x20000-4 : 0x1FFFC , and store in 0x1FFFC , the last 4 bytes 
		
		- at KEIL output file , file name is APROM_application , under \obj folder , 
	
			which mapping to generateChecksum.cmd , generateCRCbinary.cmd , generateCRChex.cmd
	
			modify the file name in KEIL project , also need to modify the file name in these 3 generate***.cmd

![image](https://github.com/released/M253BSP_IAP_XMODEM_APROM/blob/main/APROM_KEIL_output_file.jpg)

![image](https://github.com/released/M253BSP_IAP_XMODEM_APROM/blob/main/APROM_SRecord_cmd_file.jpg)
		
		- after project compile finish , binary size will be 110K (total application code size : 0x1B800)
		
		- under terminal , use keyboard , '1' , will write specific value in SRAM address , and return to boot loader
		
		- use teraterm Xmodem transfer data , to programming Application code project binary (110K) , when under Boot loader flow

		- with teraterm Xmodem to select file , 

![image](https://github.com/released/M253BSP_IAP_XMODEM_APROM/blob/main/teraterm_select_file_by_xmodem.jpg)

		- with teraterm Xmodem , under file transfer

![image](https://github.com/released/M253BSP_IAP_XMODEM_APROM/blob/main/teraterm_transfer_under_xmodem.jpg)

		- if plan to use KEIL to programming flash with CRC , refer to below setting
		
![image](https://github.com/released/M253BSP_IAP_XMODEM_APROM/blob/main/program_by_KEIL.jpg)	
		
	
5. Flash allocation

	- LDROM_Bootloader.bin : 0x0000 ~ 0x4800
	
	- Application code START address : 0x4800
	
	- Data flash : M253 have no data flash
	
	- Chcecksum storage : 0x1FFFC

6. Function assignment

	- debug port : UART4 (PA2 , PA3) , in Boot loader an Application code project
	
	- ISP UART port : UART1 (PB2 , PB3) , in Boot loader project
	
	- enable CRC , Timer1 module
	
7. Need to use ICP tool , to programm boot loader project file (ISP_UART.bin) @ 0x00 in APROM

below is boot loader project , Config setting 

![image](https://github.com/released/M253BSP_IAP_XMODEM_APROM/blob/main/LDROM_ICP_config.jpg)

below is boot loader project , ICP programming setting 

- ISP_UART.bin : 0x0000

![image](https://github.com/released/M253BSP_IAP_XMODEM_APROM/blob/main/LDROM_ICP_update.jpg)

8. under Application code KEIL project setting 

![image](https://github.com/released/M253BSP_IAP_XMODEM_APROM/blob/main/APROM_KEIL_checksum_calculate.jpg)

in Application project , press '1' will reset to Boot loader 

![image](https://github.com/released/M253BSP_IAP_XMODEM_APROM/blob/main/APROM_press_Z.jpg)

9. under GCC project , replace different loader file as below (ISP code , app code)

ISP code
![image](https://github.com/released/M253BSP_IAP_XMODEM_APROM/blob/main/GCC_ld_isp.jpg)

APP code
![image](https://github.com/released/M253BSP_IAP_XMODEM_APROM/blob/main/GCC_ld_application.jpg)

