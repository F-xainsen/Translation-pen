/*************************************************************
 * @file        usb_desc.c
 * @brief        USB descriptors ,discribed USB spec1.1 chapter 9
 * @author        Jiang Kaigan
 * @version        V1.0
 * @date        2020-12-03
 * @par
 * @attention
 *
 * @history        2020-12-03 jkg    create this file
 */

/*file type declaration section*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*internal symbols import section*/
#include "usbclass.h"
#include "usb.h"
#include "usbdef.h"
#include "driver_usb.h"

#pragma pack(1)
typedef struct {
    int ind;
    CBufferBaseDesc desc;
} _t_index_descriptor;
#pragma pack()


// ==================================================================================
// DELI USB HID Mouse Report Descriptor
#define STANDARD_KEY_REPORT_ID			3
#define CUSTOM_KEY_REPORT_ID			1

const UI8 tHidEP1ReportDesc[]={
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x02,        // Usage (Mouse)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x01,        //   Report ID (1)
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x05, 0x09,        //     Usage Page (Button)
    0x19, 0x01,        //     Usage Minimum (0x01)
    0x29, 0x05,        //     Usage Maximum (0x05)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x95, 0x05,        //     Report Count (5)
    0x75, 0x01,        //     Report Size (1)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x95, 0x01,        //     Report Count (1)
    0x75, 0x03,        //     Report Size (3)
    0x81, 0x01,        //     Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x16, 0x00, 0x00,  //     Logical Minimum (0)
    0x26, 0xFF, 0x7F,  //     Logical Maximum (32767)
    0x75, 0x10,        //     Report Size (16)
    0x95, 0x02,        //     Report Count (2)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              //   End Collection
    0xC0,              // End Collection
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x02,        // Usage (Mouse)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x02,        //   Report ID (2)
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x05, 0x09,        //     Usage Page (Button)
    0x19, 0x01,        //     Usage Minimum (0x01)
    0x29, 0x05,        //     Usage Maximum (0x05)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x95, 0x05,        //     Report Count (5)
    0x75, 0x01,        //     Report Size (1)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x95, 0x01,        //     Report Count (1)
    0x75, 0x03,        //     Report Size (3)
    0x81, 0x01,        //     Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x09, 0x38,        //     Usage (Wheel)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x03,        //     Report Count (3)
    0x81, 0x06,        //     Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              //   End Collection
    0xC0,              // End Collection

    
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x06,        // Usage (Keyboard)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x03,        //   Report ID (3)
    0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
    0x19, 0xE0,        //   Usage Minimum (0xE0)
    0x29, 0xE7,        //   Usage Maximum (0xE7)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x08,        //   Report Count (8)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x08,        //   Report Size (8)
    0x81, 0x01,        //   Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x95, 0x05,        //   Report Count (5)
    0x75, 0x01,        //   Report Size (1)
    0x05, 0x08,        //   Usage Page (LEDs)
    0x19, 0x01,        //   Usage Minimum (Num Lock)
    0x29, 0x05,        //   Usage Maximum (Kana)
    0x91, 0x02,        //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x03,        //   Report Size (3)
    0x91, 0x01,        //   Output (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x95, 0x06,        //   Report Count (6)
    0x75, 0x08,        //   Report Size (8)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0xFF,        //   Logical Maximum (255)
    0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
    0x19, 0x00,        //   Usage Minimum (0x00)
    0x29, 0xFF,        //   Usage Maximum (0xFF)
    0x81, 0x00,        //   Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              // End Collection
};

const UI8 tHidEP2ReportDesc[]={
    0x06, 0x00, 0xFF,  // Usage Page (Vendor Defined 0xFF00)
    0x09, 0x01,        // Usage (0x01)
    0xA1, 0x01,        // Collection (Application)
    0x85, CUSTOM_KEY_REPORT_ID,        //   Report ID (1)
    0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
    0x09, 0x46,        //   Usage (Vno)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x95, 0x09,        //   Report Count (9)
    0x75, 0x08,        //   Report Size (8)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x06, 0x00, 0xFF,  //   Usage Page (Vendor Defined 0xFF00)
    0x09, 0x01,        //   Usage (0x01)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x95, 0x09,        //   Report Count (9)
    0x75, 0x08,        //   Report Size (8)
    0x91, 0x02,        //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x05, 0x0C,        //   Usage Page (Consumer)
    0x09, 0x00,        //   Usage (Unassigned)
    0x15, 0x80,        //   Logical Minimum (-128)
    0x25, 0x7F,        //   Logical Maximum (127)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x08,        //   Report Count (8)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0xC0,              // End Collection

};
    
const UI8 tHidEP3ReportDesc[]={
    0x05, 0x0C,        // Usage Page (Consumer)
    0x09, 0x01,        // Usage (Consumer Control)
    0xA1, 0x01,        // Collection (Application)
    0x19, 0x00,        //   Usage Minimum (0x00)
    0x2A, 0x9C, 0x02,  //   Usage Maximum (0x29C)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0x9C, 0x02,  //   Logical Maximum (668)
    0x75, 0x10,        //   Report Size (16)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x00,        //   Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              // End Collection
};

// ==================================================================================

const UI16 tUsbLanguageID[]={
    0x04+(USB_DESCTYPE_STRING<<8),
    _LANGUAGE_ID_
};

const UI16 tUsbProductStr[]={
    (((12+1)*2))+(USB_DESCTYPE_STRING<<8),'M','A','C',':','4','b','d','3','8','c','9','1',
};

const UI16 tUsbVersionStr[]={
    (((12+1)*2))+(USB_DESCTYPE_STRING<<8),'V','e','r',':','2','0','2','4','1','2','2','1',
};

const UI16 tUsbVendorStr[]={
    (((4+1)*2))+(USB_DESCTYPE_STRING<<8),'D','E','L','I'
};

const CUsbDevDesc tUsbDeviceDescriptor={
    sizeof(CUsbDevDesc),    // bLength: 18
    USB_DESCTYPE_DEVICE,
    USB_VERSION,            // bcdUSB
    USB_DEVICE_CLASS(0),
    USB_DEVICE_SUBCLASS(0),
    USB_DEVICE_PROTOCOL(0),
    USB_MAX_PACKET_LEN(64),
    _VID_,                      //idVendor: Unknown (0x300e)
    _PID_,                      //idProduct: Unknown (0x4062)
    0x0101,
    _Vendor_String_ID_,         //iManufacturer: 1
    _Product_String_ID_,        //iProduct: 2
    0x03,                                       //usb.iSerialNumber
    USB_DEVICE_CONFIG_NUM(USB_MAX_CONFIG_NUM)   //usb.bNumConfigurations
};

#if 1
/*
*MIC+SPK+Mouse+HID vendor IO
*/
#if(USB_MIC_ONLY)
#define SZ_Audio_ACI(nIntfAs)                9/*Interface AC*/+\
    8+((nIntfAs))+/*Header*/+\
    12    /*IT(1)*/+\
    9    /*OT(2)*/+\
    0    /*FU(1)*/+\
    0    /*FU(2)*/+\
    0
#else
#define SZ_Audio_ACI(nIntfAs)                9/*Interface AC*/+\
    8+((nIntfAs))+/*Header*/+\
    12    /*IT(1)*/+\
    12    /*IT(2)*/+\
    9    /*OT(1)*/+\
    9    /*OT(2)*/+\
    0    /*FU(1)*/+\
    0    /*FU(2)*/+\
    0
#endif
#define SZ_Audio_ASI(nSR)                9+/*Interface AS Bw0*/+\
    9    /*Interface AS Normal*/+\
    7    /*AS General*/+\
    8+((nSR)*3)    /*AS FormatI*/+\
    9    /*Endpoint for Audio class*/+\
    7    /*Endpoint AS General*/+\
    0

#define SZ_HID_Intf(nEndp)                9+/*HID Interface descriptor*/+\
    9    /*HID class descriptor*/+\
    7*(nEndp)    /*Endpoint descriptor*/+\
    0
#if(USB_MIC_ONLY)
#define SZ_CONFIG(nMicSR,nSpkSR,nIntfAs)        9/*config*/+\
    SZ_Audio_ACI(nIntfAs)+\
    SZ_Audio_ASI(nMicSR)/*MIC descs size*/+\
    SZ_HID_Intf(2)/*HID Mouse descs size*/+\
    SZ_HID_Intf(2)/*HID DBG descs size*/+\
    SZ_HID_Intf(1)/*HID Mkey descs size*/+\
    0
#else
#define SZ_CONFIG(nMicSR,nSpkSR,nIntfAs)        9/*config*/+\
    SZ_Audio_ACI(nIntfAs)+\
    SZ_Audio_ASI(nMicSR)/*MIC descs size*/+\
    SZ_Audio_ASI(nSpkSR)/*SPK descs size*/+\
    SZ_HID_Intf(2)/*HID Mouse descs size*/+\
    SZ_HID_Intf(2)/*HID DBG descs size*/+\
    SZ_HID_Intf(1)/*HID Mkey descs size*/+\
    0
#endif
const UI8 tUsbConfig[]={
    // CONFIGURATION DESCRIPTOR
    0x09,       // bLength: 9
    0x02,       // bDescriptorType: 0x02 (CONFIGURATION)
    0x62,0x00,  // wTotalLength: 98
    0x03,       // bNumInterfaces: 3
    0x01,       // bConfigurationValue: 1
    0x00,       // iConfiguration: 0
    0x80,       // Configuration bmAttributes: 0x80  NOT SELF-POWERED  NO REMOTE-WAKEUP
    0x32,       // bMaxPower: 50  (100mA)

    // INTERFACE DESCRIPTOR (0.0): class HID
    0x09,       // bLength: 9
    0x04,       // bDescriptorType: 0x04 (INTERFACE)
    0x00,       // bInterfaceNumber: 0
    0x00,       // bAlternateSetting: 0
    0x02,       // bNumEndpoints: 2
    0x03,       // bInterfaceClass: HID (0x03)
    0x00,       // bInterfaceSubClass: No Subclass (0x00)
    0x00,       // bInterfaceProtocol: 0x00
    0x00,       // iInterface: 0

    // HID DESCRIPTOR
    0x09,       // bLength: 9
    0x21,       // bDescriptorType: 0x21 (HID)
    0x00,0x01,  // bcdHID: 0x0100
    0x00,       // bCountryCode: Not Supported (0x00)
    0x01,       // bNumDescriptors: 1
    0x22,       // bDescriptorType: HID Report (0x22)
    0xad,0x00,  // wDescriptorLength: 173

    // INPUT ENDPOINT DESCRIPTOR
    0x07,       // bLength: 7
    0x05,       // bDescriptorType: 0x05 (ENDPOINT)
    0x81,       // bEndpointAddress: 0x81  IN  Endpoint:1
    0x03,       // bmAttributes: 0x03
    0x10,0x00,  // wMaxPacketSize: 16
    0x01,       // bInterval: 1

    // OUTPUT ENDPOINT DESCRIPTOR
    0x07,       // bLength: 7
    0x05,       // bDescriptorType: 0x05 (ENDPOINT)
    0x02,       // bEndpointAddress: 0x02  OUT  Endpoint:2
    0x03,       // bmAttributes: 0x03
    0x08,0x00,  // wMaxPacketSize: 8
    0x01,       // bInterval: 1

    // INTERFACE DESCRIPTOR (1.0): class HID
    0x09,       // bLength: 9
    0x04,       // bDescriptorType: 0x04 (INTERFACE)
    0x01,       // bInterfaceNumber: 1
    0x00,       // bAlternateSetting: 0
    0x02,       // bNumEndpoints: 2
    0x03,       // bInterfaceClass: HID (0x03)
    0x00,       // bInterfaceSubClass: No Subclass (0x00)
    0x00,       // bInterfaceProtocol: 0x00
    0x00,       // iInterface: 0

    // HID DESCRIPTOR
    0x09,       // bLength: 9
    0x21,       // bDescriptorType: 0x21 (HID)
    0x00,0x01,  // bcdHID: 0x0100
    0x00,       // bCountryCode: Not Supported (0x00)
    0x01,       // bNumDescriptors: 1
    0x22,       // bDescriptorType: HID Report (0x22)
    0x37,0x00,  // wDescriptorLength: 55

    // ENDPOINT DESCRIPTOR
    0x07,       // bLength: 7
    0x05,       // bDescriptorType: 0x05 (ENDPOINT)
    0x84,       // bEndpointAddress: 0x84  IN  Endpoint:4
    0x03,       // bmAttributes: 0x03
    0x40,0x00,  // wMaxPacketSize: 64
    0x01,       // bInterval: 1

    // ENDPOINT DESCRIPTOR
    0x07,       // bLength: 7
    0x05,       // bDescriptorType: 0x05 (ENDPOINT)
    0x03,       // bEndpointAddress: 0x03  OUT  Endpoint:3
    0x03,       // bmAttributes: 0x03
    0x40,0x00,  // wMaxPacketSize: 64
    0x01,       // bInterval: 1

    // INTERFACE DESCRIPTOR (2.0): class HID
    0x09,   // bLength: 9
    0x04,   // bDescriptorType: 0x04 (INTERFACE)
    0x02,   // bInterfaceNumber: 2
    0x00,   // bAlternateSetting: 0
    0x01,   // bNumEndpoints: 1
    0x03,   // bInterfaceClass: HID (0x03)
    0x00,   // bInterfaceSubClass: No Subclass (0x00)
    0x00,   // bInterfaceProtocol: 0x00
    0x00,   // iInterface: 0

    // HID DESCRIPTOR
    0x09,       // bLength: 9
    0x21,       // bDescriptorType: 0x21 (HID)
    0x00,0x01,  // bcdHID: 0x0100
    0x00,       // bCountryCode: Not Supported (0x00)
    0x01,       // bNumDescriptors: 1
    0x22,       // bDescriptorType: HID Report (0x22)
    0x17,0x00,  // wDescriptorLength: 23

    // INPUT ENDPOINT DESCRIPTOR
    0x07,       // bLength: 7
    0x05,       // bDescriptorType: 0x05 (ENDPOINT)
    0x85,       // bEndpointAddress: 0x85  IN  Endpoint:5
    0x03,       // bmAttributes: 0x03
    0x10,0x00,  // wMaxPacketSize: 16
    0x01,       // bInterval: 1
};
#endif

const CBufferBaseDesc _c_device_desc={(void*)&tUsbDeviceDescriptor,sizeof(tUsbDeviceDescriptor)};
const CBufferBaseDesc _c_config_desc={(void*)tUsbConfig,sizeof(tUsbConfig)};
const _t_index_descriptor tblUsbHidRptDescs[]={
    {0,{(void*)tHidEP1ReportDesc,sizeof(tHidEP1ReportDesc)}},
    {1,{(void*)tHidEP2ReportDesc,sizeof(tHidEP2ReportDesc)}},
    {2,{(void*)tHidEP3ReportDesc,sizeof(tHidEP3ReportDesc)}},
};
const _t_index_descriptor tblUsbStringDescs[]={
    {0,{(void*)tUsbLanguageID,sizeof(tUsbLanguageID)}},
    {_Vendor_String_ID_,{(void*)tUsbVendorStr,sizeof(tUsbVendorStr)}},
    {_Version_String_ID_,{(void*)tUsbVersionStr,sizeof(tUsbVersionStr)}},
    {_Product_String_ID_,{(void*)tUsbProductStr,sizeof(tUsbProductStr)}},
};

void*USBDesc_GetDeviceDesc(void){
    return (void*)&_c_device_desc;
}

void*USBDesc_GetConfigDesc(void){
    return (void*)&_c_config_desc;
}
void*UsbDesc_Find(int idx,void*tb,int cnt){
    _t_index_descriptor*ptbl=(_t_index_descriptor*)tb;
    int i;
    for(i=0;i<cnt;i++){
        if(ptbl[i].ind==idx){
            return (void*)&ptbl[i].desc;
        }
    }
    return(NULL);
}

void*USBDesc_GetHidRptDesc(int idx){
    return UsbDesc_Find(idx,(void*)tblUsbHidRptDescs,GET_ELEMENT_TBL(tblUsbHidRptDescs));
}

void*USBDesc_GetStringDesc(int idx){
    return UsbDesc_Find(idx,(void*)tblUsbStringDescs,GET_ELEMENT_TBL(tblUsbStringDescs));
}

