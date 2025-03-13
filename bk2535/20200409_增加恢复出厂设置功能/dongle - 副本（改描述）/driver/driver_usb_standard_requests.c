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
/*! \file driver_usb_standard_requests.c
    \brief The driver interface of usb standard requests module.
*/

#include "headfile\includes.h"

/*! \fn void driver_usb_get_status_out_endpoint(UINT8 index)
    \brief If endpoint is halted, return 0x01 0x00, otherwise return 0x00,0x00 to indicate endpoint active.

    \param index - usb endpoint state
    \return void
*/
void driver_usb_get_status_out_endpoint(UINT8 index)
{
    if((index <= 3) && (usb_endpoint_state[index] != EP_STALL))
    {           
        system_data.usb_data_pointer = (UINT8 * )&VALUE_ZERO_PACKET; //return 0x00,0x00 to indicate endpoint active
        system_data.usb_data_size = 2;
    }
    else
    {
        system_data.usb_data_pointer = (UINT8 * ) & VALUE_ONE_PACKET; //If endpoint is halted, return 0x01,0x00
        system_data.usb_data_size = 2;
    }
}

/*! \fn void driver_usb_get_status(void)
    \brief  This routine returns a two byte status packet to the host.

    \param void
    \return void
*/
void driver_usb_get_status(void)                 
{                                       
    //If non-zero return length or data length not Equal to 2, then send a stall indicating invalid request       
    if(setup.wValue.c[MSB] || setup.wValue.c[LSB] || setup.wLength.c[MSB] || (setup.wLength.c[LSB] != 2))
         driver_usb_force_stall();

     switch(setup.bmRequestType)                    // check whether recipient was device,interface, or EP
     {                                    
        case OUT_DEVICE:                                 
           if(setup.wIndex.c[MSB] || setup.wIndex.c[LSB])
           {
                driver_usb_force_stall();             
           }
           else
           {
              if(flag_usb_remote_wakeup)
                  system_data.usb_data_pointer = (UINT8*) & VALUE_TWO_PACKET;
              else
                  system_data.usb_data_pointer = (UINT8*) & VALUE_ZERO_PACKET;  // send 0x00, indicating bus power and no remote wake-up supported
              system_data.usb_data_size = 2;
           }
           break;

        case OUT_INTERFACE:              
           if((system_data.usb_state != DEV_CONFIGURED))
           {
               driver_usb_force_stall();            
           }
           else
           {
              system_data.usb_data_pointer = (UINT8 * ) & VALUE_ZERO_PACKET;
              system_data.usb_data_size = 2;
           }
         break;

         case OUT_ENDPOINT:             
            if((system_data.usb_state != DEV_CONFIGURED) || setup.wIndex.c[MSB]) //Make sure device is configured and index msb = 0x00
            {                             
                driver_usb_force_stall();
            }
            else
            {
                if(setup.wIndex.c[LSB] == IN_EP1)              
                    driver_usb_get_status_out_endpoint(1);
                else if(setup.wIndex.c[LSB] == IN_EP2)
                    driver_usb_get_status_out_endpoint(2);
                else if(setup.wIndex.c[LSB] == IN_EP3)
                    driver_usb_get_status_out_endpoint(3);
                else
                    driver_usb_force_stall();
            }
         break;

      default:
         driver_usb_force_stall();
         break;
   }
     
   if(usb_endpoint_state[0] != EP_STALL)
   {
        usb_endpoint_state[0] = EP_TX;          // Set serviced setup Packet, Endpoint 0 in transmit mode, and reset system_data.usb_data_send_count counter
        system_data.usb_data_send_count = 0;
   }
}

/*! \fn void driver_usb_clear_feature(void)
    \brief  This routine can clear halt endpoint features on endpoint 1.

    \param void
    \return void
*/
void driver_usb_clear_feature(void)                  
{                                       
   // Send procedural stall if device isn't configured request is made to host(remote wakeup not supported) Or data length set to non-zero Or request is made to interface
   if((system_data.usb_state != DEV_CONFIGURED) || (setup.bmRequestType == IN_INTERFACE) || setup.wLength.c[MSB] || setup.wLength.c[LSB])
         //||(setup.bmRequestType == IN_DEVICE)
   {
      driver_usb_force_stall();
   }
   else
   {
        // Verify that packet was directed at an endpoint // flag_usb_remote_wakeup feature is selected
        if((setup.bmRequestType == IN_DEVICE) && (setup.wValue.c[LSB] == DEVICE_REMOTE_WAKEUP))
        {
            flag_usb_remote_wakeup = 0;
        }
        else if((setup.bmRequestType == IN_ENDPOINT) && (setup.wValue.c[LSB] == ENDPOINT_HALT))     // The feature selected was HALT_ENDPOINT 
        {
            if(setup.wIndex.c[LSB] == IN_EP1)
            {
                EP_HALT &= ~0x02;
                usb_endpoint_state[1] = EP_IDLE;    
            }
            else if(setup.wIndex.c[LSB] == IN_EP2)
            {
                EP_HALT &= ~0x04;
                usb_endpoint_state[2] = EP_IDLE;  
            }
            else if(setup.wIndex.c[LSB] == IN_EP3)
            {
                EP_HALT &= ~0x08;
                usb_endpoint_state[3] = EP_IDLE;  
            }
        }
        else
        {
            driver_usb_force_stall();               // Send procedural stall
        }
   }
   
   if(usb_endpoint_state[0] != EP_STALL)
   {
      driver_usb_set_wait_status();         // Set Serviced Out packet ready and data end to indicate transaction is over
   }
}

/*! \fn void driver_usb_set_feature(void)
    \brief  This routine can set halt endpoint features on endpoint 1.

    \param void
    \return void
*/
void driver_usb_set_feature(void)                
{                                      
   // Make sure device is configured, setup data is all valid and that request is directed at an endpoint
   if((system_data.usb_state != DEV_CONFIGURED) || (setup.bmRequestType == IN_INTERFACE) || setup.wLength.c[MSB] || setup.wLength.c[LSB])
   {
      driver_usb_force_stall();                  
   }
   else
   {
        if((setup.bmRequestType == IN_DEVICE) && (setup.wValue.c[LSB] == DEVICE_REMOTE_WAKEUP))//flag_usb_remote_wakeup feature is selected
        {
            flag_usb_remote_wakeup = 1;
        }
        else if((setup.bmRequestType == IN_ENDPOINT) && (setup.wValue.c[LSB] == ENDPOINT_HALT) )  //endpoint feature is selected
        {
            if(setup.wIndex.c[LSB] == IN_EP1)
            {
                EP_HALT |= 0x02;
                usb_endpoint_state[1] = EP_STALL;
            }
            else if(setup.wIndex.c[LSB] == IN_EP2)
            {
                EP_HALT |= 0x04;
                usb_endpoint_state[2] = EP_STALL;
            }
            else if(setup.wIndex.c[LSB] == IN_EP3)
            {
                EP_HALT |= 0x08;
                usb_endpoint_state[3] = EP_STALL;
            }
         }
         else
         {
             driver_usb_force_stall();
         }
   }
   
   if(usb_endpoint_state[0] != EP_STALL)
      driver_usb_set_wait_status();
}

/*! \fn void driver_usb_set_address(void)
    \brief  This routine can set new function address. Request must be directed to device

    \param void
    \return void
*/
void driver_usb_set_address(void)               
{
    if((setup.bmRequestType != IN_DEVICE) || setup.wIndex.c[MSB]  || setup.wIndex.c[LSB]|| setup.wLength.c[MSB] || setup.wLength.c[LSB]|| setup.wValue.c[MSB]  || (setup.wValue.c[LSB] & 0x80))
        driver_usb_force_stall();                                   // Send stall if setup data invalid

    if(setup.wValue.c[LSB] != 0)
        system_data.usb_state = DEV_ADDRESS;            // Indicate that device state is now address
    else
        system_data.usb_state = DEV_DEFAULT;             // If new address was 0x00, return device to default state
    if(usb_endpoint_state[0] != EP_STALL)
        driver_usb_set_wait_address();

    //system_data.usb_data_size = 0;
    //usb_endpoint_state[0] = EP_TX ;
}

/*! \fn void driver_usb_get_descriptor(void)
    \brief  This routine sets the data pointer and size to correct descriptor, and sets the endpoint state to transmit.

    \param void
    \return void
*/
void driver_usb_get_descriptor(void)             
{                                       
    switch(setup.wValue.c[MSB])         
    {                                    
        case DSC_DEVICE:                  
            system_data.usb_data_pointer = (UINT8 *) &hid_device_descriptor;
            system_data.usb_data_size = 18;
            break;

        case DSC_CONFIG:
            system_data.usb_data_pointer = (UINT8 *) &hid_configuration_descriptor.configuration_descriptor;
            system_data.usb_data_size = LEN_USB_CONFIGURATION_DESCRIPTOR;
            break;

        case DSC_INTERFACE:
            if(setup.wValue.c[LSB] == 0)
            { 
                system_data.usb_data_pointer = (UINT8*) & hid_configuration_descriptor.interface_descriptor_keyboard;
                system_data.usb_data_size = hid_configuration_descriptor.interface_descriptor_keyboard.bLength;
            }
            else if(setup.wValue.c[LSB] == 1)
            { 
                system_data.usb_data_pointer = (UINT8*) & hid_configuration_descriptor.interface_descriptor_mouse;
                system_data.usb_data_size = hid_configuration_descriptor.interface_descriptor_mouse.bLength;
            }
            else if(setup.wValue.c[LSB] == 2)
            { 
                system_data.usb_data_pointer = (UINT8*) & hid_configuration_descriptor.interface_descriptor_mce;
                system_data.usb_data_size = hid_configuration_descriptor.interface_descriptor_mce.bLength;
            }
            break;

        case DSC_HID:                            // HID Specific (HID class descriptor)
            if(setup.wIndex.i == 0x00)
            {
                system_data.usb_data_pointer = (UINT8*)&hid_configuration_descriptor.class_descriptor_keyboard;
                system_data.usb_data_size = hid_configuration_descriptor.class_descriptor_keyboard.bLength;
            }
            else if(setup.wIndex.i == 0x01)
            {
                system_data.usb_data_pointer = (UINT8*)&hid_configuration_descriptor.class_descriptor_mouse;
                system_data.usb_data_size = hid_configuration_descriptor.class_descriptor_mouse.bLength;		
            }
            else if(setup.wIndex.i == 0x02)
            {
                system_data.usb_data_pointer = (UINT8*)&hid_configuration_descriptor.class_descriptor_mce;
                system_data.usb_data_size = hid_configuration_descriptor.class_descriptor_mce.bLength;		
            }
            break;

        case DSC_HID_REPORT:                // HID Specific (HID report descriptor)
            flag_usb_get_hid_report = 1;
            flag_usb_get_protocol = 0;
            if(setup.wIndex.i == 0x00)
            {
                system_data.usb_data_pointer = (UINT8*)&hid_keyboard_report_descriptor;
                if(setup.wLength.i > HID_KEYBOARD_REPORT_DESCRIPTOR_SIZE)
                    setup.wLength.i = HID_KEYBOARD_REPORT_DESCRIPTOR_SIZE;
                system_data.usb_data_size =  setup.wLength.i;
            }
            else if(setup.wIndex.i == 0x01)
            {
                system_data.usb_data_pointer = (UINT8*)&hid_mouse_report_descriptor;
                if(setup.wLength.i > HID_MOUSE_REPORT_DESCRIPTOR_SIZE)
                    setup.wLength.i = HID_MOUSE_REPORT_DESCRIPTOR_SIZE;
                system_data.usb_data_size = setup.wLength.i;
            }
            else if(setup.wIndex.i == 0x02)
            {
                system_data.usb_data_pointer = (UINT8*)&hid_media_report_descriptor;
                if(setup.wLength.i > HID_MEDIA_REPORT_DESCRIPTOR_SIZE)
                    setup.wLength.i = HID_MEDIA_REPORT_DESCRIPTOR_SIZE;
                system_data.usb_data_size = setup.wLength.i;
            }
            break;

        case DSC_STRING:
            // NOTE: if strings are added to this project, the hard-coded value of 2 will need to be increased
            if(setup.wValue.c[LSB] > 2)        // If asking for string that's N/A
            {
                driver_usb_force_stall();
            }
            else
            {
                system_data.usb_data_pointer = usb_string_table[setup.wValue.c[LSB]];
                system_data.usb_data_size = *system_data.usb_data_pointer;      // Can have a maximum of 255 strings
            }
            break;

        default:
            //system_data.usb_protocol_status = 0;
            driver_usb_force_stall();               // Send Stall if unsupported request
            break;
    }

   // Verify that the requested descriptor is valid
   if((setup.wValue.c[MSB] == DSC_DEVICE )|| (setup.wValue.c[MSB] == DSC_CONFIG) ||(setup.wValue.c[MSB] == DSC_STRING) || (setup.wValue.c[MSB] == DSC_INTERFACE) || (setup.wValue.c[MSB] == DSC_ENDPOINT))
   {
        if(setup.wLength.i< system_data.usb_data_size)
        {
             system_data.usb_data_size = setup.wLength.i;  // Send only requested amount of data
        }
   }
   if(usb_endpoint_state[0] != EP_STALL)            // Make sure endpoint not in stall mode
   {
       usb_endpoint_state[0] = EP_TX;                 // Put endpoint in transmit mode
       system_data.usb_data_send_count = 0;         // Reset Data Sent counter
   }
}

/*! \fn void driver_usb_get_configuration(void)
    \brief  This routine returns current configuration value.

    \param void
    \return void
*/
void driver_usb_get_configuration (void)          
{                                       
   // This request must be directed to the device // With value word set to zero // And index set to zero // And setup length set to one
   if((setup.bmRequestType != OUT_DEVICE) ||setup.wValue.c[MSB] ||setup.wValue.c[LSB] ||setup.wIndex.c[MSB]  || setup.wIndex.c[LSB]||setup.wLength.c[MSB] || (setup.wLength.c[LSB] != 1))
   {
      driver_usb_force_stall();                        // send a stall to host
   }
   else
   {
      if(system_data.usb_state == DEV_CONFIGURED)
      { 
         system_data.usb_data_pointer = (UINT8*) & VALUE_ONE_PACKET; // If the device is configured, then return value 0x01 since this software only supports one configuration
         system_data.usb_data_size = 1;
      }
      if(system_data.usb_state == DEV_ADDRESS)   
      {                                
         system_data.usb_data_pointer = (UINT8*) & VALUE_ZERO_PACKET;// If the device is in address state, it is not configured, so return 0x00
         system_data.usb_data_size = 1;
      }
   }
   
   if(usb_endpoint_state[0] != EP_STALL)
   {
      usb_endpoint_state[0] = EP_TX;             // Put endpoint into transmit mode
      system_data.usb_data_send_count = 0;    // Reset Data Sent counter to zero
   }
}

/*! \fn void driver_usb_set_configuration(void)
    \brief  This routine allows host to change current device configuration value.

    \param void
    \return void
*/
void driver_usb_set_configuration (void)          
{                                      
   // Device must be addressed before configured //and request recipient must be the device // the index and length words must be zero// This software only supports config = 0,1
   if((system_data.usb_state == DEV_DEFAULT) || (setup.bmRequestType != IN_DEVICE) || setup.wIndex.c[MSB]  || setup.wIndex.c[LSB]||setup.wLength.c[MSB] || setup.wLength.c[LSB] || setup.wValue.c[MSB]  || (setup.wValue.c[LSB] > 1))
   {
      driver_usb_force_stall();                        // Send stall if setup data is invalid
   }
   else
   {
        if(setup.wValue.c[LSB] > 0)                       // Any positive configuration request results in configuration being set to 1
        {                                 
            system_data.usb_state = DEV_CONFIGURED;
            usb_endpoint_state[1] = EP_IDLE;       // Set endpoint status to idle (enabled)
            usb_endpoint_state[2] = EP_IDLE;       // Set endpoint status to idle (enabled)         
            usb_endpoint_state[3] = EP_IDLE;       // Set endpoint status to idle (enabled) 
        }
        else
        {
            system_data.usb_state = DEV_ADDRESS;// Unconfigures device by setting state to address, and changing endpoint
            usb_endpoint_state[1] = EP_STALL;      
            usb_endpoint_state[2] = EP_STALL;       // 1 and 2
            usb_endpoint_state[3] = EP_STALL;       // 1 and 2
        }
   }

   if(usb_endpoint_state[0] != EP_STALL)
       driver_usb_set_wait_status();
   flag_usb_config_set = 1;
}

/*! \fn void driver_usb_get_interface(void)
    \brief  This routine returns 0x00, since no alternate interface is supported by this firmware.

    \param void
    \return void
*/
void driver_usb_get_interface (void)              
{                                                                             
   // If device is not configured// or recipient is not an interface// or non-zero value or index fields // or data length not equal to one
   if((system_data.usb_state != DEV_CONFIGURED) || (setup.bmRequestType != OUT_INTERFACE) || setup.wValue.c[MSB]  ||setup.wValue.c[LSB] || setup.wIndex.c[MSB]  ||setup.wIndex.c[LSB] || setup.wLength.c[MSB] ||(setup.wLength.c[LSB] != 1))
   {
      driver_usb_force_stall();                    // Then return stall due to invalid request
   }
   else
   {
      system_data.usb_data_pointer = (UINT8*) & VALUE_ZERO_PACKET;         // return 0x00 to host
      system_data.usb_data_size = 1;
   }
   
   if(usb_endpoint_state[0] != EP_STALL)
   {
     // Set Serviced setup packet, put endpoint in transmit mode and reset data sent counter
      usb_endpoint_state[0] = EP_TX;
      system_data.usb_data_send_count = 0;
   }
}

/*! \fn void driver_usb_set_interface(void)
    \brief  This routine sets the interface. Make sure request is directed at interface and all other packet values are set to zero.

    \param void
    \return void
*/
void driver_usb_set_interface (void)
{
   if((setup.bmRequestType != IN_INTERFACE)  || setup.wLength.c[MSB] ||setup.wLength.c[LSB]||setup.wValue.c[MSB]  ||setup.wValue.c[LSB] ||setup.wIndex.c[MSB]  ||setup.wIndex.c[LSB])
      driver_usb_force_stall();                     // send a stall to host
   
   if(usb_endpoint_state[0] != EP_STALL)
      driver_usb_set_wait_status();  	     // Indicate setup packet has been serviced
}

/*! \fn void driver_usb_get_idle(void)
    \brief  This routine checks whether usb is in idle state.

    \param void
    \return void
*/
void driver_usb_get_idle(void) 
{
    flag_usb_idle = 1;
    system_data.usb_data_pointer = (UINT8*) & system_data.usb_idle_rate;
    system_data.usb_data_size = 1;
    if(usb_endpoint_state[0] != EP_STALL)
    {
        //Set Serviced setup packet, put endpoint in transmit mode and reset Data sent counter
        usb_endpoint_state[0] = EP_TX;
        system_data.usb_data_send_count = 0;
    }
}

/*! \fn void driver_usb_set_idle(void)
    \brief  This routine sets the idle feature by interrupt in endpoint.

    \param void
    \return void
*/
void driver_usb_set_idle (void)
{ 
    flag_usb_idle = 1;
    system_data.usb_idle_rate = setup.wValue.c[MSB];
    driver_usb_set_wait_status();
}

/*! \fn void driver_usb_get_protocol(void)
    \brief  This routine returns the usb protocol.

    \param void
    \return void
*/
void driver_usb_get_protocol(void) 
{ 
    flag_usb_get_protocol = 1;
    system_data.usb_data_pointer = (UINT8*) &system_data.usb_protocol_status;
    system_data.usb_data_size = 1;

    usb_endpoint_state[0] = EP_TX;        // Endpoint 0 in transmit mode
    system_data.usb_data_send_count = 0;
}

/*! \fn void driver_usb_set_protocol(void)
    \brief  This routine sets the usb protocol.

    \param void
    \return void
*/
void driver_usb_set_protocol (void) 
{ 
    system_data.usb_protocol_status = setup.wValue.c[LSB];
    driver_usb_set_wait_status();
}

/*! \fn void driver_usb_get_report(void)
    \brief  This routine sends a given report type to the host.

    \param void
    \return void
*/
void driver_usb_get_report (void)
{
    system_data.usb_data_pointer = (UINT8*) & VALUE_ZERO_PACKET;            //set system_data.usb_data_pointer to buffer used inside Control Endpoint
    system_data.usb_data_size = EP0_ADDR_DEF_6;

    if((setup.wValue.c[MSB] == PC_GET_REPORT_2) && (setup.wValue.c[LSB] == PC_GET_REPORT_1))
    {
        if(flag_usb_endpoint0_ready_report)      // data has been filled before, send start from here
        {
            flag_usb_endpoint0_ready_report = 0;
            system_xdata.usbp0_data.get_report_data[0] = PC_GET_DATA;	        //data head, 01 means ok
        }
        else					                          // no data
        {
            system_xdata.usbp0_data.get_report_data[0] = PC_GET_NONDATA;	//data head, 00 means fail
        }
        system_data.usb_data_pointer = (UINT8*) & system_xdata.usbp0_data.get_report_data;
		
    }

    if(usb_endpoint_state[0] != EP_STALL)
    {
        //Set serviced setup Packet
        usb_endpoint_state[0] = EP_TX ;              // Endpoint 0 in transmit mode
        system_data.usb_data_send_count = 0;       // Reset system_data.usb_data_send_count counter
    }
}

/*! \fn void driver_usb_set_report(void)
    \brief  This routine receives a report sent from the host.

    \param void
    \return void
*/
void driver_usb_set_report (void)
{
   system_data.usb_data_pointer  = system_xdata.usbp0_data.out_report;      // prepare buffer for OUT packet, set system_data.usb_data_pointer to buffer
   system_data.usb_data_size = EP0_ADDR_DEF_6;
   
   system_xdata.usbp0_data.wait_out_report = 1;
   if(usb_endpoint_state[0] != EP_STALL)
   {
      // Set serviced setup Packet
      usb_endpoint_state[0] = EP_RX ;             // Endpoint 0 in transmit mode
      system_data.usb_data_send_count = 0;     // Reset system_data.usb_data_send_count counter
   }
}

/***********************************************************
						end file
***********************************************************/
