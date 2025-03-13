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

#include "headfile\includes.h"

CODE struct_usb_device_descriptor hid_device_descriptor =
{
    18,                                     // bLength
    0x01,                                 // bDescriptorType
    0x1001,                             // bcdUSB
    0x00,                                 // bDeviceClass
    0x00,                                 // bDeviceSubClass
    0x00,                                 // bDeviceProtocol
    EP0_PACKET_SIZE,             // bMaxPacketSize0
    0xA845,                             // idVendor
    0x0107,                             // idProduct
    0x0110,                             // bcdDevice
    0x01,                                // iManufacturer
    0x02,                                // iProduct
    0x00,                                // iSerialNumber
    0x01                                 // bNumConfigurations
}; // end of hid_device_descriptor

/* From "USB Device Class Definition for Human Interface Devices (HID)" Section 7.1:
    "When a Get_Descriptor(Configuration) request is issued, it returns the Configuration descriptor, all Interface descriptors,
    all Endpoint descriptors, and the HID descriptor for each interface."*/
CODE const struct_hid_configuration_descriptor hid_configuration_descriptor =
{
    { //struct_hid_configuration_descriptor
    0x09,                               // Length
    0x02,                               // Type
    LEN_USB_CONFIGURATION_DESCRIPTOR_LE, // Totallength = 9+(9+9+7)*3
    0x03,                               // NumInterfaces
    0x01,                               // bConfigurationValue
    0x00,                               // iConfiguration
    0xa0,                               // bmAttributes
    0x32                                // MaxPower (in 2mA units)
    },

    //  keyboard
    { // struct_usb_interface_descriptor hid_interface_descriptor
    0x09,                               // bLength
    0x04,                               // bDescriptorType
    0x00,                               // bInterfaceNumber
    0x00,                               // bAlternateSetting
    0x01,                               // bNumEndpoints
    0x03,                               // bInterfaceClass (3 = HID)
    0x01,                               // bInterfaceSubClass
    0x01,                               // bInterfaceProcotol
    0x00                                // iInterface
    },

    { // class_descriptor hid_descriptor
    0x09,                               // bLength
    0x21,                               // bDescriptorType
    0x0101,                            // bcdHID
    0x00,                               // bCountryCode
    0x01,                               // bNumDescriptors
    0x22,                               // bDescriptorType
    HID_KEYBOARD_REPORT_DESCRIPTOR_SIZE_LE// wItemLength (tot. len. of report descriptor)
    },

    // IN endpoint (mandatory for HID)
    { // struct_usb_endpoint_descriptor hid_endpoint_in_descriptor
    0x07,                               // bLength
    0x05,                               // bDescriptorType
    0x81,                               // bEndpointAddress
    0x03,                               // bmAttributes
    EP1_PACKET_SIZE_LE,     // MaxPacketSize (LITTLE ENDIAN)
    10                                  // bInterval
    },

    //mouse
    { // struct_usb_interface_descriptor hid_interface_descriptor
    0x09,                               // bLength
    0x04,                               // bDescriptorType
    0x01,                               // bInterfaceNumber
    0x00,                               // bAlternateSetting
    0x01,                               // bNumEndpoints
    0x03,                               // bInterfaceClass (3 = HID)
    0x01,                               // bInterfaceSubClass
    0x02,                               // bInterfaceProcotol
    0x00                                // iInterface
    },

    { // class_descriptor hid_descriptor
    0x09,                               // bLength
    0x21,                               // bDescriptorType
    0x0101,                            // bcdHID
    0x00,                               // bCountryCode
    0x01,                               // bNumDescriptors
    0x22,                               // bDescriptorType
    HID_MOUSE_REPORT_DESCRIPTOR_SIZE_LE  // wItemLength (tot. len. of report descriptor)
    },

    // IN endpoint (mandatory for HID)
    { // struct_usb_endpoint_descriptor hid_endpoint_in_descriptor
    0x07,                               // bLength
    0x05,                               // bDescriptorType
    0x82,                               // bEndpointAddress
    0x03,                               // bmAttributes
    EP2_PACKET_SIZE_LE,     // MaxPacketSize (LITTLE ENDIAN)
    4                                  // bInterval
    },
    
    // media +power key
    { // struct_usb_interface_descriptor hid_interface_descriptor
    0x09,                               // bLength
    0x04,                               // bDescriptorType
    0x02,                               // bInterfaceNumber
    0x00,                               // bAlternateSetting
    0x01,                               // bNumEndpoints
    0x03,                               // bInterfaceClass (3 = HID)
    0x01,                               // bInterfaceSubClass
    0x01,                               // bInterfaceProcotol // keyboard:mouse = 1:2 
    0x00                                // iInterface
    },

    { // class_descriptor hid_descriptor
    0x09,                               // bLength
    0x21,                               // bDescriptorType
    0x0101,                            // bcdHID
    0x00,                               // bCountryCode
    0x01,                               // bNumDescriptors
    0x22,                               // bDescriptorType
    HID_MEDIA_REPORT_DESCRIPTOR_SIZE_LE  // wItemLength (tot. len. of report descriptor)
    },

    // IN endpoint (mandatory for HID)
    { // struct_usb_endpoint_descriptor hid_endpoint_in_descriptor
    0x07,                               // bLength
    0x05,                               // bDescriptorType
    0x83,                               // bEndpointAddress
    0x03,                               // bmAttributes
    EP3_PACKET_SIZE_LE,     // MaxPacketSize (LITTLE ENDIAN)
    10                                  // bInterval
    }
};

CODE const struct_hid_keyboard_report_descriptor hid_keyboard_report_descriptor =
{
    0x05, 0x01,        // Usage Page (Generic Desktop),                    
    0x09, 0x06,        // Usage (Keyboard),                               
    0xA1, 0x01,        // Collection (Application),                       
    0x05, 0x07,        // Usage Page (Key Codes);                         
    0x19, 0xE0,        // Usage Minimum (224),                            
    0x29, 0xE7,        // Usage Maximum (231),                            
    0x15, 0x00,        // Logical Minimum (0),                            
    0x25, 0x01,        // Logical Maximum (1),                            
    0x75, 0x01,        // Report Size (1),                                
    0x95, 0x08,        // Report Count (8),                               
    0x81, 0x02,        // Input (Data, Variable, Absolute), ;Modifier byte
    0x95, 0x01,        // Report Count (1),                               
    0x75, 0x08,        // Report Size (8),                                
    0x81, 0x03,        // Input (Constant), ;Reserved byte                
    0x95, 0x05,        // Report Count (5),                               
    0x75, 0x01,        // Report Size (1),                                
    0x05, 0x08,        // Usage Page (Page# for LEDs),                    
    0x19, 0x01,        // Usage Minimum (1),                              
    0x29, 0x05,        // Usage Maximum (5),                              
    0x91, 0x02,        // Output (Data, Variable, Absolute), ;Led report  
    0x95, 0x01,        // Report Count (1),   01                            
    0x75, 0x03,        // Report Size (3),                                
    0x91, 0x03,        // Output (Constant), ;Led report padding          
    0x95, 0x06,        // Report Count (6),                               
    0x75, 0x08,        // Report Size (8),                                
    0x15, 0x00,        // Logical Minimum (0),                            
    0x26, 0xff, 0x00,  //0x25, 0xff,        // Logical Maximum(101),                           
    0x05, 0x07,        // Usage Page (Key Codes),                         
    0x19, 0x00,        // Usage Minimum (0),                              
    0x29, 0x65,        // Usage Maximum (101),                            
    0x81, 0x00,        // Input (Data, Array), ;Key arrays (6 bytes)      
    0xC0                  // End Collection
};

CODE const struct_hid_mouse_report_descriptor hid_mouse_report_descriptor =
{
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x02,                    // USAGE (Mouse)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x09, 0x01,                    //   USAGE (Pointer)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x85, 0x01,
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)      5bit+3bit
    0x29, 0x05,                    //     USAGE_MAXIMUM (Button 5)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x95, 0x05,                    //     REPORT_COUNT (5)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs), bit0-3: left/middle/right
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x75, 0x03,                    //     REPORT_SIZE (3)
    0x81, 0x03,                    //     INPUT (Cnst,Var,Abs)
    
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x15, 0x81, 	      //     Logical Minimum (-32767)
    0x25, 0x7F,	      //     Logical Maximum (32767) 
    //0x15, 0x00,        // Logical Minimum (0),                            
   // 0x26, 0xff,0x00,   
    0x75, 0x08,                    //     REPORT_SIZE (16)
    0x95, 0x02,                    //     REPORT_COUNT (2)    
    0x81, 0x06,                    //     INPUT (Data,Var,Rel)
    
    0x09, 0x38,                    //     USAGE (Wheel)
    0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
    0x25, 0x7f,                    //     LOGICAL_MAXIMUM (127)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x81, 0x06,                    //     INPUT (Data,Var,Rel)  

#ifdef MOUSE_SWING
    0x05, 0x0c,		      //     USAGE_PAGE(Consumer Devices)
    0x0a, 0x38, 0x02,           //     SWING
    0x95, 0x01,		      //     REPORT_COUNT (1)	
    0x81, 0x06,                    //     INPUT (Data,Var,Rel)
#endif

    0xc0,                             //    END_COLLECTION
    0xc0,                              //    END_COLLECTION

	0x06, 0x00,0xff, 
    0x09, 0x60, 
    0xa1, 0x01, 
    0x85, 0x33, 
    0x19, 0x00, 
    0x2a, 0xff,0x00,  
    0x15, 0x00,
    0x26, 0xff,0x00, 
    0x75, 0x08, 
    0x95, 0x07,
    0xb1, 0x00,
    0xc0
};

CODE const struct_hid_media_report_descriptor hid_media_report_descriptor =
{
#if 0
    //====================电源控制======31==============================
    0x05, 0x01, 	// Usage Page (Generic Desktop),
    0x09, 0x80,	// USAGE (3-D Digitizer)
    0xa1, 0x01,	// Collection (Application),
    0x85, 0x02, 	// Report ID (2)
    0x05, 0x01, 	// Usage Page (Generic Desktop),
    0x19, 0x81, 	// Usage Minimum (), 
    0x29, 0x83, 	// Usage Maximum (),
    0x15, 0x00, 	// Logical Minimum (0),
    0x25, 0x01, 	// Logical Maximum (1),
    0x95, 0x03, 	// Report Count (3),
    0x75, 0x01, 	// Report Size (1),
    0x81, 0x06, 	// Input (Data,Var,Rel)
    0x95, 0x01, 	// Report Count (1),  
    0x75, 0x05, 	// Report Size (5),	
    0x81, 0x01,	// Input (Const,Ary,Abs)
    0xc0,			// END_COLLECTION
 #endif   

    //====================多媒体按键控制=====25========================
    0x05, 0x0c,	// USAGE_PAGE (Consumer Devices)
    0x09, 0x01,	// USAGE (Consumer Control)
    0xa1, 0x01,  	// COLLECTION (Application)
    0x85, 0x03, 	// Report ID (3)
    0x15, 0x00,	// LOGICAL_MINIMUM (0)
    0x26, 0x80, 0x03, // Logical Maximum (1),
    0x19, 0x00,       // Usage Minimum (), 
    0x2a,  0x80, 0x03,  // Usage Maximum (),
    0x75, 0x10, 	 // REPORT_SIZE (0x10), 16bit
    0x95, 0x01, 	 // REPORT_COUNT (1)
    0x81, 0x00, 	 // Input (Data,Ary,Abs)
    0xc0, 			 // END_COLLECTION
    
#ifdef KEYBOARD_MCE
    //====================MCE键盘控制=======26===========================
    0x06, 0xbc, 0xff, // USAGE_PAGE (Vendor Defined Page *)
    0x09, 0x88,   	  // USAGE(?)
    0xa1, 0x01, 	  // Collection (Application),
    0x85, 0x04, 	  // Report ID (4)
    0x19, 0x00, 	  // USAGE_MINIMUM (undefined)
    0x2a, 0xff, 0x00, 
    0x15, 0x00, 	  // Logical Minimum (0),
    0x26, 0xff, 0x00, 
    0x75, 0x08,  	  // REPORT_SIZE (8), 8bit
    0x95, 0x01,  	  // REPORT_COUNT (1)
    0x81, 0x00,  	  // Input (Data,Ary,Abs)
    0xc0,  		  // END_COLLECTION
#endif

#if 0
    //====================beken============38======================
    0x06, 0x00, 0xff,//   USAGE_PAGE (Vendor Defined Page 1)
    0x09, 0x0e,         //   USAGE (Vendor Usage 1)
    0xa1, 0x01,         //   COLLECTION (Application)
    0x85, 0xBA,         //   Report ID
    0x95, 0x1f,         //   REPORT_COUNT()
    0x75, 0x08,         //   REPORT_SIZE (8)
    0x26, 0xff, 0x00,//   LOGICAL_MAXIMUM (255)
    0x15, 0x00,         //   LOGICAL_MINIMUM (0)
    0x09, 0x01,         //   USAGE (Vendor Usage 1)
    0x91, 0x02,         //   OUTPUT (Data,Var,Abs)

    0x85, 0xBA,         //   Report ID
    0x95, 0x1f,         //   REPORT_COUNT ( )
    0x75, 0x08,         //   REPORT_SIZE (8)
    0x26, 0xff, 0x00,//   LOGICAL_MAXIMUM (255)
    0x15, 0x00,         //   LOGICAL_MINIMUM (0)
    0x09, 0x01,         //   USAGE (Vendor Usage 1)
    0x81, 0x02,         //   INPUT (Data,Var,Abs)
    0xC0                   //   end Application Collection};
#endif    
};


CODE const UINT8 usb_string_descriptor0[LEN_STRING0] =
{
   LEN_STRING0, 0x03, 0x09, 0x04
}; 

CODE const UINT8 usb_string_descriptor1[LEN_STRING1] =
{
   LEN_STRING1, 0x03, 
   'S', 0,
   'm', 0,
   'a', 0,
   'r', 0,
   't', 0
}; 

CODE const UINT8 usb_string_descriptor2[LEN_STRING2] =
{
   LEN_STRING2, 0x03,
   'S', 0,
   'm', 0,
   'a', 0,
   'r', 0,
   't', 0,
   ' ', 0,
   'W', 0,
   'i', 0,
   'r', 0,
   'e', 0,
   'l', 0,
   'e', 0,
   's', 0,
   's', 0,
   ' ', 0,
   'D', 0,
   'e', 0,
   'v', 0,
   'i', 0,
   'c', 0,
   'e', 0
};

CODE UINT8 * usb_string_table[LEN_USB_STRING] =
{
// 3B
   usb_string_descriptor0,
   usb_string_descriptor1,
   usb_string_descriptor2
};

IDATA setup_buffer setup; 
/***********************************************************
						end file
***********************************************************/
