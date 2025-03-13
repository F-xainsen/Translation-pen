/****************************************************************************
**
** Copyright (C) 2011 Beken Corporation, All rights reserved.
**
** Contact: Beken Corporation (www.beken.com)
**
** Author:  river
**
** History: 2012/03/07 
**
** Version: 1.0
**
****************************************************************************/
/*! \file driver_rf.c
    \brief The driver interface of rf module.
*/

#include "headfile\includes.h"

/*! \fn UINT8 driver_rf_spi_rw(UINT8 value) 
    \brief Write \value on spi mosi, and read status on spi miso.

    \param value - written value on mosi
    \return the status of miso
*/
UINT8 driver_rf_spi_rw(UINT8 value)                                    
{                                                           
    UINT8 bit_ctr;

    for(bit_ctr=0; bit_ctr<8; bit_ctr++)
    {
        if(value & 0x80)
            RF_SPI_MOSI=1;
        else
            RF_SPI_MOSI=0;		

    	value = (value << 1);
    	RF_SPI_CLK = 1;           
    	value |= RF_SPI_MISO;
    	RF_SPI_CLK = 0;          
    }
    
    return(value);   
}    

/*! \fn void driver_rf_spi_write_register(UINT8 reg,  UINT8 value)
    \brief Write the value of register address \reg.

    \param reg - register address
    \param value - data
    \return void
*/
void driver_rf_spi_write_register(UINT8 reg,  UINT8 value)
{
    RF_SPI_CS = 0;    
    driver_rf_spi_rw(reg);  
    driver_rf_spi_rw(value); 
    RF_SPI_CS = 1;  
}       

/*! \fn UINT8 driver_rf_spi_read_register(UINT8 reg)
    \brief Read the register value of address \reg.

    \param reg - register address
    \return the value of register
*/
UINT8 driver_rf_spi_read_register(UINT8 reg)
{                                                           
    UINT8 value;
    
    RF_SPI_CS = 0;
    driver_rf_spi_rw(reg); 
    value = driver_rf_spi_rw(0);
    RF_SPI_CS = 1;          

    return(value); 
}   

/*! \fn void driver_rf_spi_read_buffer(UINT8 reg,  UINT8 *pBuf,  UINT8 length) 
    \brief Write a \length data to \pBuf from the start register \reg.

    \param reg - start register address
    \param pBuf - the pointer address
    \param length - data length
    \return void
*/
void driver_rf_spi_read_buffer(UINT8 reg,  UINT8 *pBuf,  UINT8 length)     
{                                                           
    UINT8 byte_ctr;                              
                                                        
    RF_SPI_CS = 0;                    	
    driver_rf_spi_rw(reg);  
    for(byte_ctr = 0; byte_ctr < length; byte_ctr++)           
        pBuf[byte_ctr] = driver_rf_spi_rw(0);
    RF_SPI_CS = 1;  
}                                                           

/*! \fn void driver_rf_spi_write_buffer(UINT8 reg,  UINT8 *pBuf,  UINT8 length) 
    \brief Write a \length data stored in \pBuf to the start register \reg.

    \param reg - start register address
    \param pBuf - the pointer address
    \param length - data length
    \return void
*/
void driver_rf_spi_write_buffer(UINT8 reg,  UINT8 *pBuf,  UINT8 length)    
{                                                           
    UINT8 byte_ctr;                              
                                                        
    RF_SPI_CS = 0;                   
    driver_rf_spi_rw(reg);  
    for(byte_ctr=0; byte_ctr<length; byte_ctr++) 
        driver_rf_spi_rw(*pBuf++);                                    
    RF_SPI_CS = 1;             
}                                  

/*! \fn void driver_rf_spi_flush_rx(void)
    \brief Flush rf module rx fifo of pipe 0.

    \param void
    \return void
*/
void driver_rf_spi_flush_rx(void)
{
    //UINT8 value;

    driver_rf_spi_write_register(FLUSH_RX, 0);                          //clear rx fifo
    //value = driver_rf_spi_read_register(STATUS);                      //clear all status, rx/tx/max
    driver_rf_spi_write_register(WRITE_REG | STATUS, 0x4E);
}

/*! \fn void driver_rf_spi_set_mode_tx(void)
    \brief Set rf module to tx mode

    \param void
    \return void
*/
void driver_rf_spi_set_mode_tx(void)
{
    UINT8 value;

    driver_rf_spi_flush_rx();  //add
	driver_rf_spi_flush_tx();

    RF_CHIP_DISABLE;
    value = driver_rf_spi_read_register(CONFIG);
    value = value & 0xfe;
    driver_rf_spi_write_register(WRITE_REG | CONFIG,  value); 
    RF_CHIP_ENABLE;
}

/*! \fn void driver_rf_spi_set_mode_rx(void)
    \brief Set rf module to rx mode

    \param void
    \return void
*/
void driver_rf_spi_set_mode_rx(void)
{
    UINT8 value;
	driver_rf_spi_flush_tx();//add

    driver_rf_spi_flush_rx();

    RF_CHIP_DISABLE;
    value = driver_rf_spi_read_register(CONFIG);
    value = value | 0x01;
    driver_rf_spi_write_register(WRITE_REG | CONFIG, value); 
    RF_CHIP_ENABLE;
}

/*! \fn void driver_rf_spi_flush_tx(void)
    \brief Flush rf module tx fifo.

    \param void
    \return void
*/
void driver_rf_spi_flush_tx(void)
{
    //UINT8 value;

    driver_rf_spi_write_register(FLUSH_TX, 0);
    //value = driver_rf_spi_read_register(STATUS);
    driver_rf_spi_write_register(WRITE_REG | STATUS, 0x30);
}

/*! \fn void driver_rf_spi_switch_bank(char bank)
    \brief Switch rf register bank to 1 or 0

    \param bank -  register bank, 1 or 0
    \return void
*/
void driver_rf_spi_switch_bank(char bank)   
{
    UINT8 read_bank;

    read_bank = driver_rf_spi_read_register(7);
    read_bank &= 0x80;
    if((read_bank && !bank) || ( !read_bank && bank))
    	driver_rf_spi_write_register(ACTIVATE_CMD, 0x53);
}

/*! \fn void driver_rf_spi_power_mode(UINT8 mode) 
    \brief Set rf module to power up or down mode

    \param mode -  RF_POWER_MODE_UP/RF_POWER_MODE_DOWN
    \return void
*/
void driver_rf_spi_power_mode(UINT8 mode)   
{
    UINT8 value;
    
    value = driver_rf_spi_read_register(CONFIG);
    if(mode)
        value |= 0x02;
    else
        value &= 0xfd;
    driver_rf_spi_write_register(WRITE_REG|CONFIG, value);
}

/*! \fn void void driver_rf_ouput_power_value_set(UINT8 power)
    \brief Set rf out power through writing \a power*2+1 into BK2401_SETUP.

    \param power - 0-3
    \return void
*/
void driver_rf_ouput_power_value_set(UINT8 power)
{
    /*
        bank1.Reg4	"Reg4<29:27>txctrl<2:0>"	"Reg4<20>palow"	pactrl<1:0>	TX Power(dBm)		
        F996821B	7	1	3	6.3		14米
        F996821B	7	1	2	4.8		
        F996821B	7	1	1	2.9		
        F996821B	7	1	0	1.3		
        						
        F986821B	7	0	3	-10		
        F986821B	7	0	2	-18		
        F986821B	7	0	1	-30		
        F986821B	7	0	0	-39		
        						
        C196821B	0	1	3	0	    4米/5.5米
        C196821B	0	1	2	-5	    1.5米	
        C196821B	0	1	1	-14		
        C196821B	0	1	0	-21	    20厘米	
        						    
        C996821B	1	1	3	0.5		
        C996821B	1	1	2	-4.5		
        C996821B	1	1	1	-12.5		
        C996821B	1	1	0	-19.5		
        						
        D196821B	2	1	3	1.7		
        D196821B	2	1	2	-3		
        D196821B	2	1	1	-10	    1米	
        D196821B	2	1	0	-16		
        						
        D996821B	3	1	3	2		
        D996821B	3	1	2	-2.3		
        D996821B	3	1	1	-9.5		
        D996821B	3	1	0	-15	    40厘米	
        						
        E196821B	3	1	3	-0.4		
        E196821B	3	1	2	1.6		
        E196821B	3	1	1	4.2		
        E196821B	3	1	0	5.8		
    ->
                bank1.reg4  rf_setup
        5dbm    F996821B    3
        0dbm    C196821B    3
        -5dbm   C196821B    2
        -10dbm  D196821B    1
        -15dbm  C196821B    1
        -20dbm  C196821B    0
    */
    if(power > POWER_VALUE_5DBM)
        power = POWER_VALUE_5DBM;
    switch(power)
    {
        case POWER_VALUE_5DBM:
            driver_rf_spi_write_bank1_reg((WRITE_REG | 4), 0xF996821B);
    #if defined(SetAirDataRate_250Kbps)   
            driver_rf_spi_write_register(WRITE_REG | RF_SETUP, (3 << 1) | 0x21); // 0=-10dbm,1=-5dbm,2=0dbm,3=5dbm,250k
    #else    
            driver_rf_spi_write_register(WRITE_REG | RF_SETUP, (3 << 1) | 0x01); // 0=-10dbm,1=-5dbm,2=0dbm,3=5dbm
    #endif
            break;
        case POWER_VALUE_0DBM:
            driver_rf_spi_write_bank1_reg((WRITE_REG | 4), 0xC196821B);
    #if defined(SetAirDataRate_250Kbps)   
            driver_rf_spi_write_register(WRITE_REG | RF_SETUP, (3 << 1) | 0x21); // 0=-10dbm,1=-5dbm,2=0dbm,3=5dbm,250k
    #else    
            driver_rf_spi_write_register(WRITE_REG | RF_SETUP, (3 << 1) | 0x01); // 0=-10dbm,1=-5dbm,2=0dbm,3=5dbm
    #endif
            break;
        case POWER_VALUE_MINUS_5DBM:
            driver_rf_spi_write_bank1_reg((WRITE_REG | 4), 0xC196821B); 
    #if defined(SetAirDataRate_250Kbps)   
            driver_rf_spi_write_register(WRITE_REG | RF_SETUP, (2 << 1) | 0x21); // 0=-10dbm,1=-5dbm,2=0dbm,3=5dbm,250k
    #else    
            driver_rf_spi_write_register(WRITE_REG | RF_SETUP, (2 << 1) | 0x01); // 0=-10dbm,1=-5dbm,2=0dbm,3=5dbm
    #endif
            break;
        case POWER_VALUE_MINUS_10DBM:
            driver_rf_spi_write_bank1_reg((WRITE_REG | 4), 0xD196821B); 
    #if defined(SetAirDataRate_250Kbps)   
            driver_rf_spi_write_register(WRITE_REG | RF_SETUP, (1 << 1) | 0x21); // 0=-10dbm,1=-5dbm,2=0dbm,3=5dbm,250k
    #else    
            driver_rf_spi_write_register(WRITE_REG | RF_SETUP, (1 << 1) | 0x01); // 0=-10dbm,1=-5dbm,2=0dbm,3=5dbm
    #endif
            break;
        case POWER_VALUE_MINUS_15DBM:
            driver_rf_spi_write_bank1_reg((WRITE_REG | 4), 0xC196821B); 
    #if defined(SetAirDataRate_250Kbps)   
            driver_rf_spi_write_register(WRITE_REG | RF_SETUP, (1 << 1) | 0x21); // 0=-10dbm,1=-5dbm,2=0dbm,3=5dbm,250k
    #else    
            driver_rf_spi_write_register(WRITE_REG | RF_SETUP, (1 << 1) | 0x01); // 0=-10dbm,1=-5dbm,2=0dbm,3=5dbm
    #endif
            break;
        case POWER_VALUE_MINUS_20DBM:
            driver_rf_spi_write_bank1_reg((WRITE_REG | 4), 0xC196821B); 
    #if defined(SetAirDataRate_250Kbps)   
            driver_rf_spi_write_register(WRITE_REG | RF_SETUP, (0 << 1) | 0x21); // 0=-10dbm,1=-5dbm,2=0dbm,3=5dbm,250k
    #else    
            driver_rf_spi_write_register(WRITE_REG | RF_SETUP, (0 << 1) | 0x01); // 0=-10dbm,1=-5dbm,2=0dbm,3=5dbm
    #endif
            break;
    }
}

/*! \fn void driver_rf_spi_write_bank1_reg(UINT8 reg, UINT32 value)
    \brief Write rf ban1 register \reg to \value.

    \param reg - register address
    \param value - 4 bytes value need be reversed
    \return void
*/
void driver_rf_spi_write_bank1_reg(UINT8 reg, UINT32 value)
{
    UINT8 i, temp[4];
    
    for(i=0; i<4; i++)
    	temp[i] = (value>>(8*(3-i))) & 0xff;
    
    driver_rf_spi_switch_bank(1);
    driver_rf_spi_write_buffer(reg, temp, 4);
    driver_rf_spi_switch_bank(0);
}

/*! \fn void driver_rf_initial(void)
    \brief Initialize rf module before first rf sending.

    \param void
    \return void
*/
void driver_rf_initial(void)
{
    INT8 i;
    RF_POWER_UP;
    driver_delay_us(50*300);                                               //delay more than 50ms.
    
    driver_rf_spi_switch_bank(0);
//bank0
    for(i = 20; i >= 0; i--)
        driver_rf_spi_write_register((WRITE_REG | RF_BANK0_REG[i][0]), RF_BANK0_REG[i][1]);
    
    //set communication address
    rf_address_rx0[0] = ID0;
    rf_address_rx0[1] = ID1;
    rf_address_rx0[2] = ID2;
    rf_address_rx0[3] = ID3;
    rf_address_rx0[4] = ID4;
    driver_rf_spi_write_buffer((WRITE_REG|RX_ADDR_P0), rf_address_rx0, RF_ADDRESS_LEN);
	
    driver_rf_spi_write_buffer((WRITE_REG|RX_ADDR_P1), rf_address_rx0, RF_ADDRESS_LEN);	
	
    driver_rf_spi_write_buffer((WRITE_REG|TX_ADDR), rf_address_rx0, RF_ADDRESS_LEN);
//    driver_rf_spi_write_buffer((WRITE_REG|RX_ADDR_P1), rf_address_rx0, RF_ADDRESS_LEN);




    if(driver_rf_spi_read_register(RF_FEATURE)==0)                                                              // i!=0 showed that chip has been actived.so do not active again.
        driver_rf_spi_write_register(ACTIVATE_CMD, 0x73);                   // Active
							    
    for(i = 22; i >= 21; i--)
        driver_rf_spi_write_register((WRITE_REG|RF_BANK0_REG[i][0]), RF_BANK0_REG[i][1]);

//bank1
	driver_rf_spi_switch_bank(1);

	driver_rf_spi_write_bank1_reg((WRITE_REG | 4), RF_BANK1_REG_0_13[0]);
	driver_rf_spi_write_bank1_reg((WRITE_REG | 5), RF_BANK1_REG_0_13[1]);
	driver_rf_spi_write_bank1_reg((WRITE_REG | 12), RF_BANK1_REG_0_13[2]);
	driver_rf_spi_write_bank1_reg((WRITE_REG | 13), RF_BANK1_REG_0_13[3]);

    //toggle
    driver_rf_spi_write_bank1_reg((WRITE_REG | 4), 0xff96821b);//(RF_BANK1_REG_0_13[4]|(0x06<<24)));
	driver_delay_us(300);
	driver_rf_spi_write_bank1_reg((WRITE_REG | 4), RF_BANK1_REG_0_13[0]);

    
    driver_rf_spi_write_buffer((WRITE_REG|14), &(RF_BANK1_REG14[0]), 11);
    //driver_rf_spi_read_buffer(8, rf_fifo_data, 4);
    //if(rf_fifo_data[0] == 0x63)                                             //0x00000063(BK2423)
        flag_rf_bk2423 = 1;
    driver_rf_spi_switch_bank(0);

    //0xF9 96 82 1B ,31-28 27-24, 1111 1111 -> 1111 1001
    //driver_rf_spi_write_bank1_reg((WRITE_REG | 4), RF_BANK1_REG_4_2401[0]); //initial to 11, so just clear to 00

//    driver_rf_spi_set_mode_rx();                                            //RX mode for page or dongle
//    driver_delay_us(300);                                                   //delay more than 1ms.
//    driver_rf_spi_write_register(FLUSH_RX, 0);
//	driver_rf_spi_flush_rx();

//	 driver_rf_spi_set_mode_tx();
//    driver_rf_spi_set_mode_rx();   	
}

/*! \fn BOOL driver_rf_data_send(UINT8 len)
    \brief Send \a len bytes rf data to the other part, and the data is stored in rf_fifo_data array.

    Step1: send data through writing rf_fifo_data to BK2401_FIFO register.\n
    Step2: wait for the sending state through checking the BK2401_STATUS register.
    \param len - the length of data
    \return TRUE - send successfully, FALSE - otherwise
*/
#if 0
BOOL driver_rf_data_send(UINT8 len)
{
    UINT8 status;
    BOOL result = FALSE;
	
    RF_CMD_FLUSH_TX;
	RF_CMD_FLUSH_RX;
	RF_CMD_CLEAR_STATUS_ALL;
    driver_rf_spi_write_buffer(0xB0,  rf_fifo_data,  len);
    do
    {
        status = driver_rf_spi_read_register(STATUS);
    }while((status & STATUS_RX_TX_MAX) == 0x00);

    if(status & STATUS_TX_DS)     
    {
        result = TRUE;
        //RF_CMD_FLUSH_TX;
        RF_CMD_CLEAR_STATUS_TX;
    }
    else if(status & STATUS_MAX_RT)    
    {
		//test_pin7 = ~test_pin7;
		RF_CMD_FLUSH_TX;
        RF_CMD_FLUSH_RX;
        RF_CMD_CLEAR_STATUS_ALL;
		
    }

    return result;
}
#endif

//=============================================================
BOOL driver_rf_data_send(UINT8 len)
{
    UINT8 status;
	BOOL result = FALSE;

	//driver_rf_spi_write_register(WRITE_REG|STATUS, 0x70);
	//RF_CMD_CLEAR_STATUS_ALL;
	driver_rf_spi_write_register(FLUSH_RX, 0);	   //原来是rx
	RF_CMD_FLUSH_TX;
 
    driver_rf_spi_write_buffer(WR_TX_PLOAD, rf_fifo_data, len);
	//driver_delay_us(2);
   // RF_CHIP_ENABLE;
    do
    {
        status = driver_rf_spi_read_register(STATUS);
		#ifdef WATCHDOG_ENABLE
        WDCTL = 0xa5;       //enable wdt and clear it
        #endif
        
    }while((status & 0x30) == 0x00);

    if(status & STATUS_TX_DS)     
    {
        result = TRUE;
        RF_CMD_CLEAR_STATUS_TX;
    }

//rf_send_exit:
    if(status & STATUS_MAX_RT)    
    {
        RF_CMD_FLUSH_TX;
        RF_CMD_FLUSH_RX;
        RF_CMD_CLEAR_STATUS_ALL;
    }

    return result;
}


/*! \fn void driver_rf_receive_packet(void)
    \brief Receive the packet to rf_fifo_data, whose length is rf_data_len_received. If received a valid flag, the flag flag_rf_driver_received is true.

    \param void
    \return void
*/
void driver_rf_receive_packet(void)	 //2MS调用一次
{
    UINT8 status;
    UINT8 fifo_status;

    status = driver_rf_spi_read_register(STATUS);    	     // read register STATUS's value 读RF状态
    // DEG(("000=%bx\n",status));												
    if(status & STATUS_RX_DR)			                     // if receive data ready (RX_DR) interrupt
    {
		//P1 ^=  1;
		//test_pin0 = ~test_pin0;
		driver_rf_spi_write_register(WRITE_REG|STATUS, status);  
        do
        {
            system_data.rf_received_len = driver_rf_spi_read_register(R_RX_PL_WID_CMD);
            if((system_data.rf_received_len <= MAX_PACKET_LEN)&&(system_data.rf_received_len>0))
            {
                //driver_buffer_set(rf_fifo_data, 0, 32);
                //P1 ^=  1;
                driver_rf_spi_read_buffer(RD_RX_PLOAD, rf_fifo_data, system_data.rf_received_len);// read receive payload from RX_FIFO buffer
                flag_rf_driver_received = 1;
				flag_rf_link_ok = 1;
				flag_rf_link_status = 1;		//代表已连接上
                system_data.rf_pipe_num = (status >> 1) & 0x07;
            }
            else
            {
                driver_rf_spi_write_register(FLUSH_RX, 0);                     // flush Rx
                //driver_rf_spi_write_register(FLUSH_TX, 0);                     // flush Tx
            }

            fifo_status = driver_rf_spi_read_register(FIFO_STATUS);	           // read register FIFO_STATUS's value
//            DEG(("000=%bx,%bx\n",system_data.rf_received_len,fifo_status));
		}while((fifo_status & FIFO_STATUS_RX_EMPTY) == 0);                     // while not empty
		 //driver_rf_spi_write_register(WRITE_REG|STATUS, status);  

	}
    
                 // clear RX_DR or TX_DS or MAX_RT interrupt flag
}

/*! \fn void driver_rf_channel_switch(void)
    \brief Switch rf channel to the next value in RF_CHANNEL_TABLE array.

    \param void
    \return void
*/
void driver_rf_channel_switch(void)
{
    system_data.rf_channel++;
    system_data.rf_channel &= 0x0F; //<=15
    driver_rf_spi_write_register(WRITE_REG|RF_CH, RF_CHANNEL_TABLE[system_data.rf_channel]);
}


#if 1 //def MODE_TEST
/*! \fn void driver_rf_mode_set_single_wave(UINT8 mode)
    \brief Set rf module to single wave mode.

    \param mode - 0/single wave, 1/normal mode
    \return void
*/
void driver_rf_mode_set_single_wave(UINT8 mode)
{
    if(mode)
        driver_rf_spi_write_bank1_reg(WRITE_REG|4, 0xD996821B);
    else
        driver_rf_spi_write_bank1_reg(WRITE_REG|4, 0xD9968221);
}

#endif

/***********************************************************
						end file
***********************************************************/

