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
/*! \file driver_usb_descriptor.h
    \brief The driver headfile of usb descriptor module.
*/

#ifndef _USB_DESCRIPTOR_H_
#define _USB_DESCRIPTOR_H_

#define LEN_USB_STRING   3
#define LEN_STRING0         4
#define LEN_STRING1         12 //sizeof("Beken") * 2
#define LEN_STRING2         44//sizeof("Beken Wireless Device") * 2

#define LEN_USB_CONFIGURATION_DESCRIPTOR_LE 0x5400
#define LEN_USB_CONFIGURATION_DESCRIPTOR 0x0054

#define HID_KEYBOARD_REPORT_DESCRIPTOR_SIZE 0x0040
#define HID_KEYBOARD_REPORT_DESCRIPTOR_SIZE_LE 0x4000

#ifdef MOUSE_SWING
#define HID_MOUSE_REPORT_DESCRIPTOR_SIZE 0x0049
#define HID_MOUSE_REPORT_DESCRIPTOR_SIZE_LE 0x4900
#else
#define HID_MOUSE_REPORT_DESCRIPTOR_SIZE 0x005A
#define HID_MOUSE_REPORT_DESCRIPTOR_SIZE_LE 0x5A00
#endif

#ifdef KEYBOARD_MCE
#define HID_MEDIA_REPORT_DESCRIPTOR_SIZE 0x0078
#define HID_MEDIA_REPORT_DESCRIPTOR_SIZE_LE 0x7800
#else
#define HID_MEDIA_REPORT_DESCRIPTOR_SIZE 0x0019//0x005E
#define HID_MEDIA_REPORT_DESCRIPTOR_SIZE_LE 0X1900//0x5E00
#endif

typedef  struct
{
//19B
   UINT8 bLength;              // Size of this Descriptor in Bytes
   UINT8 bDescriptorType;      // Descriptor Type (=1)
   WORD bcdUSB;                        // USB Spec Release Number in BCD
   UINT8 bDeviceClass;         // Device Class Code
   UINT8 bDeviceSubClass;      // Device Subclass Code
   UINT8 bDeviceProtocol;      // Device Protocol Code
   UINT8 bMaxPacketSize0;      // Maximum Packet Size for EP0
   WORD idVendor;                      // Vendor ID
   WORD idProduct;                     // Product ID
   WORD bcdDevice;                     // Device Release Number in BCD
   UINT8 iManufacturer;        // Index of String Desc for Manufacturer
   UINT8 iProduct;             // Index of String Desc for Product
   UINT8 iSerialNumber;        // Index of String Desc for SerNo
   UINT8 bNumConfigurations;   // Number of possible Configurations
} struct_usb_device_descriptor;                   // End of Device Descriptor Type

//--------------------------------------------------
// Standard Configuration Descriptor Type Definition
//--------------------------------------------------
typedef  struct
{
   UINT8 bLength;              // Size of this Descriptor in Bytes
   UINT8 bDescriptorType;      // Descriptor Type (=2)
   WORD wTotalLength;                  // Total Length of Data for this Conf
   UINT8 bNumInterfaces;       // No of Interfaces supported by this Conf
   UINT8 bConfigurationValue;  // Designator Value for this Configuration
   UINT8 iConfiguration;       // Index of String Desc for this Conf
   UINT8 bmAttributes;         // Configuration Characteristics (see below)
   UINT8 bMaxPower;            // Max. Power Consumption in this Conf (*2mA)
} struct_usb_configuration_descriptor;            // End of Configuration Descriptor Type

//----------------------------------------------
// Standard Interface Descriptor Type Definition
//----------------------------------------------
typedef  struct
{
   UINT8 bLength;              // Size of this Descriptor in Bytes
   UINT8 bDescriptorType;      // Descriptor Type (=4)
   UINT8 bInterfaceNumber;     // Number of *this* Interface (0..)
   UINT8 bAlternateSetting;    // Alternative for this Interface (if any)
   UINT8 bNumEndpoints;        // No of EPs used by this IF (excl. EP0)
   UINT8 bInterfaceClass;      // Interface Class Code
   UINT8 bInterfaceSubClass;   // Interface Subclass Code
   UINT8 bInterfaceProtocol;   // Interface Protocol Code
   UINT8 iInterface;           // Index of String Desc for this Interface
} struct_usb_interface_descriptor;                // End of Interface Descriptor Type

//------------------------------------------
// Standard Class Descriptor Type Definition
//------------------------------------------
typedef struct
{
   UINT8 bLength;               // Size of this Descriptor in Bytes (=9)
   UINT8 bDescriptorType;       // Descriptor Type (HID=0x21)
   WORD bcdHID;                         // HID Class Specification
                                        // release number (=1.01)
   UINT8 bCountryCode;          // Localized country code
   UINT8 bNumDescriptors;       // Number of class descriptors to follow
   UINT8 bReportDescriptorType; // Report descriptor type (HID=0x22)
   unsigned int wItemLength;            // Total length of report descriptor table
} struct_usb_class_descriptor;                     // End of Class Descriptor Type

//---------------------------------------------
// Standard Endpoint Descriptor Type Definition
//---------------------------------------------
typedef  struct
{
   UINT8 bLength;              // Size of this Descriptor in Bytes
   UINT8 bDescriptorType;      // Descriptor Type (=5)
   UINT8 bEndpointAddress;     // Endpoint Address (Number + Direction)
   UINT8 bmAttributes;         // Endpoint Attributes (Transfer Type)
   WORD wMaxPacketSize;                // Max. Endpoint Packet Size
   UINT8 bInterval;            // Polling Interval (Interrupt) ms
} struct_usb_endpoint_descriptor;                 // End of Endpoint Descriptor Type

//---------------------------------------------
// HID Configuration Descriptor Type Definition
//---------------------------------------------
typedef struct 
{
    struct_usb_configuration_descriptor 	configuration_descriptor;

    struct_usb_interface_descriptor 		interface_descriptor_keyboard;
    struct_usb_class_descriptor 			class_descriptor_keyboard;
    struct_usb_endpoint_descriptor 		endpoint_descriptor_keyboard;

    struct_usb_interface_descriptor 		interface_descriptor_mouse;
    struct_usb_class_descriptor 			class_descriptor_mouse;
    struct_usb_endpoint_descriptor 		endpoint_descriptor_mouse;

    struct_usb_interface_descriptor 		interface_descriptor_mce;
    struct_usb_class_descriptor 			class_descriptor_mce;
    struct_usb_endpoint_descriptor 		endpoint_descriptor_mce;
}struct_hid_configuration_descriptor;

typedef UINT8 struct_hid_keyboard_report_descriptor[HID_KEYBOARD_REPORT_DESCRIPTOR_SIZE];
typedef UINT8 struct_hid_mouse_report_descriptor[HID_MOUSE_REPORT_DESCRIPTOR_SIZE];
typedef UINT8 struct_hid_media_report_descriptor[HID_MEDIA_REPORT_DESCRIPTOR_SIZE];

//-----------------------------
// SETUP Packet Type Definition
//-----------------------------
typedef struct
{
//8B
   UINT8 bmRequestType;                     // Request recipient, type, and dir.
   UINT8 bRequest;                                // Specific standard request number
   WORD wValue;                                   // varies according to request
   WORD wIndex;                                  // varies according to request
   WORD wLength;                                // Number of bytes to transfer
} setup_buffer;                                 // End of SETUP Packet Type

#endif

/***********************************************************
						end file
***********************************************************/
