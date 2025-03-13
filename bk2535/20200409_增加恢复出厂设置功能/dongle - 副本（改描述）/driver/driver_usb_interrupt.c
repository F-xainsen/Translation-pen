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
/*! \file driver_usb_interrupt.c
    \brief The driver interface of usb interrupt module.
*/

#include "headfile\includes.h"

/*! \fn void driver_usb_isr_data_control(void)
    \brief Process the control data from usb endpoint0.

    \param void
    \return void
*/
void driver_usb_isr_data_control(void)
{
    UINT8 report_cnt;

#ifdef BK5100_FEATURE
    if((EP_STATUS_OUT & 0x01)||(EP_STATUS_IN & 0x01))                                                                                          // 8 byte setup data received
    {     
        EP_STATUS_OUT = 0x01;                                                                                 // write 1 to clear
        EP_STATUS_IN = 0x01;
#else
    if(EP_STATUS & 0x01)                                                                                          // 8 byte setup data received
    {     
        EP_STATUS = 0x01;                                                                                         // write 1 to clear
#endif
        usb_endpoint_state[0] = EP_IDLE;                                                                // force usb_endpoint_state[0] to EP_IDLE
        system_xdata.usbp0_data.wait_out_report = 0;
        
        if(CNT0 == 8)                                                                                                 // Make sure that EP 0 has an Out Packet of size 8 byte
        {                                
            if((EP0_ADDR_DEF == 0x21) && (EP0_ADDR_DEF_1 == 0x09))
            {
                USBINT0 = 0xc0;
                driver_usb_set_report();
                system_xdata.usbp0_data.report_id = EP0_ADDR_DEF_2;
                system_xdata.usbp0_data.report_type = EP0_ADDR_DEF_3;

                driver_usb_set_current_endpoint(EP_0);
            }
            else
            {
                driver_usb_fifo_read(EP_0, 8, (UINT8 *)&setup);                                   // Get setup Packet from Fifo, it is currently Big-Endian
                setup.wValue.i = setup.wValue.c[MSB] +((int)setup.wValue.c[LSB] << 8); // Compiler Specific - these next three statements swap the bytes of the setup packet words to Big Endian so they can be compared to other 16-bit values elsewhere properly
                setup.wIndex.i = setup.wIndex.c[MSB] +((int)setup.wIndex.c[LSB] << 8);
                setup.wLength.i = setup.wLength.c[MSB] +((int)setup.wLength.c[LSB] << 8);

                if((setup.bmRequestType & 0x7f) == DSC_HID)                                       // HID class - specific requests
                {
                    switch(setup.bRequest) {
                        case GET_REPORT:
                            driver_usb_get_report();
                            break;
                        case SET_REPORT:
                            driver_usb_set_report();          
                            break;
                        case GET_IDLE:
                            driver_usb_get_idle();
                            break;
                        case SET_IDLE:
                            driver_usb_set_idle();
                            break;
                        case GET_PROTOCOL:
                            driver_usb_get_protocol();
                            break;
                        case SET_PROTOCOL:
                            driver_usb_set_protocol();
                            break;
                        default:
                            driver_usb_force_stall();      // Send stall to host if invalid request
                            break;                  
                    }
                } 
                else
                {
                    switch(setup.bRequest)
                    {                             
                        case GET_STATUS:
                           driver_usb_get_status();
                           break;
                        case CLEAR_FEATURE:
                           driver_usb_clear_feature();
                           break;
                        case SET_FEATURE:
                           driver_usb_set_feature();
                           break;
                        case SET_ADDRESS:
                           driver_usb_set_address();
                           break;
                        case GET_DESCRIPTOR:
                           driver_usb_get_descriptor();
                           break;
                        case GET_CONFIGURATION:
                           driver_usb_get_configuration();
                           break;
                        case SET_CONFIGURATION:
                           driver_usb_set_configuration();
                           break;
                        case GET_INTERFACE:
                           driver_usb_get_interface();
                           break;
                        case SET_INTERFACE:
                           driver_usb_set_interface();
                           break;
                        default:
                           driver_usb_force_stall();                 // Send stall to host if invalid request
                           break;
                    }               
                }//(setup.bmRequestType & ~ 0x80) == DSC_HID
            }//(EP0_ADDR_DEF==0x21 && EP0_ADDR_DEF_1 == 0x09 )
        }      //(CNT0 == 8)
        else 
        {
            driver_usb_force_stall();                                     // if rec setup data is not 8 byte , protacal stall
        }
   }   
   else if(usb_endpoint_state[0] == EP_SET_ADDRESS)     // Handle Status Phase of Set Address
   {
        FUNCT_ADR = setup.wValue.c[LSB];                            // set usb Address SFR
        usb_endpoint_state[0] = EP_IDLE;
   }
   else if(usb_endpoint_state[0] == EP_WAIT_STATUS)
   {
        usb_endpoint_state[0] = EP_IDLE;
        flag_usb_send_wait_state = 0;
   }   
   else if(usb_endpoint_state[0] == EP_RX)                       // See whether endpoint0 should transmit
   {  
   		

        if(system_data.usb_data_size >= EP0_PACKET_SIZE)  // read bytes from FIFO
        {
            driver_usb_fifo_read(EP_0, EP0_PACKET_SIZE, (UINT8 *)system_data.usb_data_pointer);
            system_data.usb_data_pointer  += EP0_PACKET_SIZE;
            system_data.usb_data_size -= EP0_PACKET_SIZE; 
            system_data.usb_data_send_count += EP0_PACKET_SIZE;
			
        }
        else
        {
            if(system_xdata.usbp0_data.wait_out_report && (system_xdata.usbp0_data.report_type == PC_SET_REPORT_2) && (system_xdata.usbp0_data.report_id == PC_SET_REPORT_1))
            {
                system_xdata.usbp0_data.wait_out_report = 0;
                system_data.usb_data_pointer = &EP0_ADDR_DEF;
                if(PC_SET_REPORT_1 == system_data.usb_data_pointer[0])
                {
                    system_data.usb_data_pointer++;
                    system_data.usb_data_size--;
                }
                for(report_cnt = 0; report_cnt < system_data.usb_data_size; report_cnt++)
                    system_xdata.usbp0_data.set_report_data[report_cnt] = system_data.usb_data_pointer[report_cnt];
                flag_usb_endpoint0_accept = 1;  	  //用做收到USB数据的标志位，拉高就可以读取数据
                flag_usb_report_state = 1;
				
            }
            else
            {
                driver_usb_fifo_read(EP_0, system_data.usb_data_size, (UINT8 *) system_data.usb_data_pointer);
                if(system_xdata.usbp0_data.wait_out_report)   // KEYBOARD_OUT_REPORT
                {
					if(system_xdata.usbp0_data.out_report[0]==0x55)
                    {
						test_pin0 ^= 1;
						test_pin0 ^= 1;
						test_pin0 ^= 1;
						test_pin0 ^= 1;
					}
					else				
                     system_data.numlock_state = system_xdata.usbp0_data.out_report[0];

//                    system_data.numlock_state = system_xdata.usbp0_data.out_report[0] & 0x01;
                    system_xdata.usbp0_data.out_report_index = system_data.usb_data_size;
                    system_xdata.usbp0_data.wait_out_report = 0;
                    flag_usb_report_state = 1;
                }
				
            }
            driver_usb_set_wait_status();                                 
        }
 
        if((system_data.usb_data_send_count == setup.wLength.i) && !flag_usb_report_state)
            driver_usb_set_wait_status();
      
        flag_usb_report_state = 0;
   }   

   if(usb_endpoint_state[0] == EP_TX)                                        // See if endpoint should transmit
   {     
        if((system_data.usb_data_size == 0))// ||(system_data.usb_data_send_count == setup.wLength.i))   // when all data has been sent over
        {
            driver_usb_set_wait_status();
        }
        else
        {   
            CFG_EP0_1 = CONST_CFG_EP0_1;
            if(system_data.usb_data_size >= EP0_PACKET_SIZE)        // Break Data into multiple packets if larger than Max Packet
            {
                driver_usb_fifo_write(EP_0, EP0_PACKET_SIZE, (UINT8*)system_data.usb_data_pointer);
                system_data.usb_data_pointer  += EP0_PACKET_SIZE;
                system_data.usb_data_size -= EP0_PACKET_SIZE;   
                system_data.usb_data_send_count += EP0_PACKET_SIZE;
            }
            else                                                                                  // If data is less than Max Packet size or zero
            {
                driver_usb_fifo_write(EP_0, system_data.usb_data_size, (UINT8*)system_data.usb_data_pointer);
                system_data.usb_data_send_count += system_data.usb_data_size;// Increment data sent counter
                system_data.usb_data_size = 0;
                USBINT0 = 0xc0;   
                EP_RDY = 0x01;

            #ifdef BK5100_FEATURE
                while(((USBINT0 & 0x4C) == 0) && ((EP_STATUS_OUT & 0x01) == 0));   //wait usb_reset, suspend or sending end interrupt  happend, wait setup come
            #else
                while(((USBINT0 & 0x4C) == 0) && ((EP_STATUS & 0x01) == 0));   //wait usb_reset, suspend or sending end interrupt  happend, wait setup come
            #endif
                if(USBINT0 & 0x40)                                                        //sending end 
                {  
                    CFG_EP0_1 &= ~0x80;
                    EP_RDY = 0x01;
                    USBINT0 = 0xc0;
                    usb_endpoint_state[0] = EP_WAIT_STATUS;
                }
                flag_usb_send_wait_state = 1; 
                system_xdata.usbp0_data.wait_out_report = 0;
            } 
        }   
    }

    if(!system_xdata.usbp0_data.wait_out_report && !flag_usb_send_wait_state)
    {  
        USBINT0 = 0xc0;
        //driver_usb_set_current_endpoint(EP_0);   // set ready to receive or send next packet
        EP_RDY = 0x01;
    }
}

/*! \fn void driver_usb_isr_data_receive(void)
    \brief Process the interrupt data from usb endpoint1-7.

    \param void
    \return void
*/
void driver_usb_isr_data_receive(void)
{ 
#ifdef BK5100_FEATURE
    if(EP_STATUS_OUT & 0x80) // ep7
        EP_STATUS_OUT = 0x80;
    
    if(EP_STATUS_OUT & 0x40) // ep6
        EP_STATUS_OUT = 0x40;
    
    if(EP_STATUS_OUT & 0x20) // ep5
        EP_STATUS_OUT = 0x20;
    
    if(EP_STATUS_OUT & 0x10) // ep4  
        EP_STATUS_OUT = 0x10; 
    
    if(EP_STATUS_OUT & 0x08) // ep3  
        EP_STATUS_OUT = 0x08;    
    
    if(EP_STATUS_OUT & 0x04) // ep2  
        EP_STATUS_OUT = 0x04; 

    if(EP_STATUS_OUT & 0x02) // ep1
        EP_STATUS_OUT = 0x02;
#else
    if(EP_STATUS & 0x80) // ep7
        EP_STATUS = 0x80;
    
    if(EP_STATUS & 0x40) // ep6
        EP_STATUS = 0x40;
    
    if(EP_STATUS & 0x20) // ep5
        EP_STATUS = 0x20;
    
    if(EP_STATUS & 0x10) // ep4  
        EP_STATUS = 0x10; 
    
    if(EP_STATUS & 0x08) // ep3  
        EP_STATUS = 0x08;    
    
    if(EP_STATUS & 0x04) // ep2  
        EP_STATUS = 0x04; 

    if(EP_STATUS & 0x02) // ep1
        EP_STATUS = 0x02;
#endif    
    USBINT0 = 0x20;
}

/*! \fn void driver_usb_isr_data_send(void)
    \brief Send the input data through usb endpoint1-7 to PC.

    \param void
    \return void
*/
void driver_usb_isr_data_send(void)
{
#ifdef BK5100_FEATURE
    if(EP_STATUS_IN & 0x80) // ep7
        EP_STATUS_IN = 0x80;
    
    if(EP_STATUS_IN & 0x40) // ep6
        EP_STATUS_IN = 0x40;
    
    if(EP_STATUS_IN & 0x20) // ep5
        EP_STATUS_IN = 0x20;
    
    if(EP_STATUS_IN & 0x10) // ep4  
        EP_STATUS_IN = 0x10;

    if(EP_STATUS_IN & 0x08) // ep3
    {
    	EP_STATUS_IN = 0x08; //media
    	flag_usb_endpoint3_ready_uploading = 1;
    }

    if(EP_STATUS_IN & 0x04) // ep2
    { 
    	EP_STATUS_IN = 0x04;
    	flag_usb_endpoint2_ready_uploading = 1;
    }

    if(EP_STATUS_IN & 0x02) // ep1  
    {
    	EP_STATUS_IN = 0x02;
    	flag_usb_endpoint1_ready_uploading = 1;
    }    
#else
    if(EP_STATUS & 0x80) // ep7
        EP_STATUS = 0x80;
    
    if(EP_STATUS & 0x40) // ep6
        EP_STATUS = 0x40;
    
    if(EP_STATUS & 0x20) // ep5
        EP_STATUS = 0x20;
    
    if(EP_STATUS & 0x10) // ep4  
        EP_STATUS = 0x10;

    if(EP_STATUS & 0x08) // ep3
    {
    	EP_STATUS = 0x08; //media
    	flag_usb_endpoint3_ready_uploading = 1;
    }

    if(EP_STATUS & 0x04) // ep2
    { 
    	EP_STATUS = 0x04;
    	flag_usb_endpoint2_ready_uploading = 1;
    }

    if(EP_STATUS & 0x02) // ep1  
    {
    	EP_STATUS = 0x02;
    	flag_usb_endpoint1_ready_uploading = 1;
    }    
#endif
     USBINT0 = 0x10;
}

/*! \fn void driver_usb_isr_reset(void)
    \brief Reset usb after receiving the command from usb interrupt.

    \param void
    \return void
*/
void driver_usb_isr_reset(void)
{
    flag_usb_get_protocol = 0;
    flag_usb_idle = 0;
    flag_usb_get_hid_report = 0;
    flag_usb_suspend = 0;
    //flag_usb_state_normal = 0;

    PCON |= 0x40;
    USB_PWR_CN &= ~ 0x02;   // clr sus_mode  bit
    USB_PWR_CN |= 0x04;
	
    while(FUNCT_ADR != 0)
        FUNCT_ADR =  0;
    system_data.usb_state = DEV_DEFAULT;
    usb_endpoint_state[0] = EP_IDLE;
    usb_endpoint_state[1] = EP_STALL;
    usb_endpoint_state[2] = EP_STALL;
    usb_endpoint_state[3] = EP_STALL;
    flag_usb_send_wait_state = 0;
    system_xdata.usbp0_data.wait_out_report = 0;
    USBINT0 = 0xc8;
}

/*! \fn void driver_usb_isr_suspend(void)
    \brief Suspend usb after receiving the command from usb interrupt.

    \param void
    \return void
*/
void driver_usb_isr_suspend(void)
{  
    USB_PWR_CN |= 0x02;  //close usb clk

    USBINT0 = 0x04;   
    EXIF &= ~0x04;            // clr mcu flag  

    //PCON &= ~0x40;       //close clk 48m
    flag_usb_suspend = 1;
}

/*! \fn void driver_usb_isr_resume(void)
    \brief Resume usb after receiving the command from usb interrupt.

    \param void
    \return void
*/
void driver_usb_isr_resume(void)
{
    PCON |= 0x40;
    USB_PWR_CN &= ~0x02;   // clr sus_mode  bit
    USBINT0 = 0x02;
    flag_usb_suspend = 0;
}

/*! \fn void driver_usb_isr_sof(void)
    \brief Process sof event of usb.

    \param void
    \return void
*/
void driver_usb_isr_sof(void)
{
    USBINT0 = 0x01;
}

/*! \fn void driver_usb_isr_unsupported_token(void)
    \brief Process unsupported token event of usb.

    \param void
    \return void
*/
void driver_usb_isr_unsupported_token(void)
{
    USBINT1 = 0x80;
}

/*! \fn void driver_usb_isr_crc_error(void)
    \brief Process crc error event of usb.

    \param void
    \return void
*/
void driver_usb_isr_crc_error(void)
{
    USBINT1 = 0x40;
}

/*! \fn void driver_usb_isr_overtime(void)
    \brief Process overtime error event of usb.

    \param void
    \return void
*/
void driver_usb_isr_overtime(void)
{
    USBINT1 = 0x20;
}

/*! \fn void driver_usb_isr_data_pid_error(void)
    \brief Process data pid error event of usb.

    \param void
    \return void
*/
void driver_usb_isr_data_pid_error(void)
{
    USBINT1 = 0x10;
}

/*! \fn void driver_usb_force_stall(void)
    \brief Put the endpoint in stall state.

    \param void
    \return void
*/
void driver_usb_force_stall(void)
{
     usb_endpoint_state[0] = EP_STALL; 
     EP_HALT |= 0x01;
}

/*! \fn void driver_usb_fifo_read(UINT8 index, UINT8 len, UINT8 *p)
    \brief Read \a len bytes from the endpoint \a index to the buffer defined by pointer.

    \param index - endpoint number, 0-4
    \prarm len - the length of data
    \param pointer - buffer pointer storing the data
    \return void
*/
void driver_usb_fifo_read(UINT8 index, UINT8 len, UINT8 *pointer)
{
#if 1
    UINT8 volatile XDATA * fifo_p;
#else
    XDATA UINT8 volatile * fifo_p;
#endif
    UINT8 i;

    switch(index)
    {
        case 0 :  
            fifo_p = &EP0_ADDR_DEF;
            break;
        case 1 :  
            fifo_p = &EP1_ADDR_DEF;
            break;
        case 2 :  
            fifo_p = &EP2_ADDR_DEF;
            break;
        case 3 :  
            fifo_p = &EP3_ADDR_DEF;
            break;
    }

    for(i=0; i<len; i++)
        pointer[i] = fifo_p[i];
}


/*! \fn void driver_usb_fifo_write(UINT8 index, UINT8 len, UINT8 *pointer)
    \brief Write \a len bytes from the buffer defined by pointer to the endpoint \a index.

    \param index - endpoint number, 0-4
    \prarm len - the length of data
    \param pointer - buffer pointer storing the data
    \return void
*/
void driver_usb_fifo_write(UINT8 index, UINT8 len, UINT8 *pointer)
{
#if 1
    UINT8 volatile XDATA * fifo_p;
    UINT8 volatile XDATA * cnt_p ;
#else
    XDATA UINT8 volatile * fifo_p;
    XDATA UINT8 volatile * cnt_p ;
#endif
    UINT8 i;
    
    switch(index)
    {
        case 0: 
            fifo_p = &EP0_ADDR_DEF; 
            cnt_p = &CNT0;
            break;
        case 1 : 
            fifo_p = &EP1_ADDR_DEF;
            cnt_p = &CNT1; 
            break;
        case 2 : 
            fifo_p = &EP2_ADDR_DEF; 
            cnt_p = &CNT2; 
            break;
        case 3 : 
            fifo_p = &EP3_ADDR_DEF; 
            cnt_p = &CNT3; 
            break;
    }

    for(i=0; i<len; i++)             
        *(fifo_p++) = *(pointer++);
    
    *cnt_p = len;   
}



/*! \fn void driver_usb_set_wait_status(void)
    \brief Set usb to wait state.

    \param void
    \return void
*/
void driver_usb_set_wait_status(void)
{
    //driver_usb_set_endpoint0_directory( !(CFG_EP0_1 & 0x80) );  // EP0_DIR = ~EP0_DIR;
    if(CFG_EP0_1 & 0x80)
        CFG_EP0_1 &= ~0x80;
    else
        CFG_EP0_1 |= 0x80;
    usb_endpoint_state[0] = EP_WAIT_STATUS;
    CNT0 = 0;                              // status package has zero bytes to send or receive
}

/*! \fn void driver_usb_set_wait_address(void)
    \brief Set the address of wait state for usb.

    \param void
    \return void
*/
void driver_usb_set_wait_address(void)
{
    usb_endpoint_state[0] = EP_SET_ADDRESS;
    driver_usb_set_endpoint0_directory( !(CFG_EP0_1 & 0x80) );   // EP0_DIR = ~EP0_DIR;
    CNT0 = 0;                                               // status package has zero bytes to send or receive
}

/*! \fn void driver_usb_set_current_endpoint(UINT8 ep_index)
    \brief Set the endpiont \a index to the current activate one.

    \param void
    \return void
*/
void driver_usb_set_current_endpoint(UINT8 ep_index)      
{
    EP_RDY = (0x01 << ep_index);   
}

/*! \fn void driver_usb_set_endpoint0_directory(UINT8 flag)
    \brief Set the directory of endpiont 0.

    \param void
    \return void
*/
void driver_usb_set_endpoint0_directory(UINT8 flag)
{
    if(flag)
        CFG_EP0_1 |= 0x80;
    else
        CFG_EP0_1 &= ~0x80;
}

/***********************************************************
						end file
***********************************************************/
