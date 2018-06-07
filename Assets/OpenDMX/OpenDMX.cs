using System;
using System.Runtime.InteropServices;
using System.IO;
using System.Threading;
using UnityEngine;

namespace OpenDMX
{

    public class DMX

    {
  #if UNITY_STANDALONE_WIN || UNITY_EDITOR_WIN
        const UInt32 FT_LIST_NUMBER_ONLY = 0x80000000;
        const UInt32  FT_LIST_BY_INDEX = 0x40000000;
        const UInt32  FT_LIST_ALL = 0x20000000;

        const UInt32 FT_OPEN_BY_SERIAL_NUMBER =  0x00000001;


        public const byte BITS_8 = 8;
        public const byte STOP_BITS_2 = 2;
        public const byte PARITY_NONE = 0;
        public const UInt16 FLOW_NONE = 0;
        public const byte PURGE_RX = 1;
        public const byte PURGE_TX = 2;

        [DllImport("FTD2XX.dll")]
        public static extern FT_STATUS FT_ListDevices(ref UInt32 devices, ref UInt64 id, UInt32 flags );

        [DllImport("FTD2XX.dll")]
        public static extern FT_STATUS FT_ListDevices(ref UInt32 devices, ref char[] msg, UInt32 flags );

        [DllImport("FTD2XX.dll")]
        public static extern FT_STATUS FT_CreateDeviceInfoList(ref UInt32 devices);

        [DllImport("FTD2XX.dll")]
        public static extern FT_STATUS FT_Open(UInt32 uiPort, ref uint ftHandle);
        [DllImport("FTD2XX.dll")]
        public static extern FT_STATUS FT_Close(uint ftHandle);
        [DllImport("FTD2XX.dll")]
        public static extern FT_STATUS FT_Read(uint ftHandle, IntPtr lpBuffer, UInt32 dwBytesToRead, ref UInt32 lpdwBytesReturned);
        [DllImport("FTD2XX.dll")]
        public static extern FT_STATUS FT_Write(uint ftHandle, IntPtr lpBuffer, UInt32 dwBytesToRead, ref UInt32 lpdwBytesWritten);
        [DllImport("FTD2XX.dll")]
        public static extern FT_STATUS FT_SetDataCharacteristics(uint ftHandle, byte uWordLength, byte uStopBits, byte uParity);
        [DllImport("FTD2XX.dll")]
        public static extern FT_STATUS FT_SetFlowControl(uint ftHandle, char usFlowControl, byte uXon, byte uXoff);
        [DllImport("FTD2XX.dll")]
        public static extern FT_STATUS FT_GetModemStatus(uint ftHandle, ref UInt32 lpdwModemStatus);
        [DllImport("FTD2XX.dll")]
        public static extern FT_STATUS FT_Purge(uint ftHandle, UInt32 dwMask);
        [DllImport("FTD2XX.dll")]
        public static extern FT_STATUS FT_ClrRts(uint ftHandle);
        [DllImport("FTD2XX.dll")]
        public static extern FT_STATUS FT_SetBreakOn(uint ftHandle);
        [DllImport("FTD2XX.dll")]
        public static extern FT_STATUS FT_SetBreakOff(uint ftHandle);
        [DllImport("FTD2XX.dll")]
        public static extern FT_STATUS FT_GetStatus(uint ftHandle, ref UInt32 lpdwAmountInRxQueue, ref UInt32 lpdwAmountInTxQueue, ref UInt32 lpdwEventStatus);
        [DllImport("FTD2XX.dll")]
        public static extern FT_STATUS FT_ResetDevice(uint ftHandle);
        [DllImport("FTD2XX.dll")]
        public static extern FT_STATUS FT_SetDivisor(uint ftHandle, char usDivisor);

        [DllImport("FTD2XX.dll")]
        public static extern FT_STATUS FT_SetBaudRate(uint ftHandle, UInt32 speed);
 

        private volatile static Thread thread;
        private volatile static bool valuesChanged = false;
        private volatile static byte[] buffer = new byte[64];
        private volatile static bool done = false;
        //private static uint handle;
        private volatile static int bytesWritten = 0;
        private static FT_STATUS status;

        
        public static void start()
        {
			
            //handle = 0;
            //status = FT_Open(0, ref handle)
            if( thread == null || !thread.IsAlive ){
                uint devices = 0;
                FT_CreateDeviceInfoList(ref devices);
                
                Debug.LogFormat("Number of Devices {0}", devices);
                if( devices == 0 )
                    return;
                
                //uint id = 0;
                //char[] buf = new char[50];
                //FT_ListDevices(ref devices, ref id, FT_LIST_NUMBER_ONLY);                
                //var status = FT_ListDevices(ref id, ref buf, FT_LIST_BY_INDEX|FT_OPEN_BY_SERIAL_NUMBER);
                //Debug.LogFormat("Device {1} {0}", new string(buf),status);

                Debug.Log("Starting DMX comms thread");
                done = false;
                valuesChanged = false;
                thread = new Thread(new ThreadStart(writeData));            
                thread.Start();
            }
            else {
                done = false;
            }
			
        }

        public static void stop()
        {            
            done = true;
            
        }

        public static void setDmxValue(int channel, byte value)
        {
            if( buffer[channel] != value ){
                buffer[channel] = value;
                valuesChanged = true;
            }
        }

        public static void setRange(int start, int end, byte value)
        {
            for(int i=start;i<=end;i++)
            {
                if( buffer[i] != value ){
                    buffer[i] = value;
                    valuesChanged |= true;
                }                
            }
        }

 
        public static FT_STATUS GetStatus(ref uint handle){
            UInt32 rx = 0;
            UInt32 tx = 0;
            UInt32 eventStatus = 0;
            return FT_GetStatus( handle, ref rx, ref tx, ref eventStatus );
        }

        public static int GetTxBuffer(ref uint handle){
            UInt32 rx = 0;
            UInt32 tx = 0;
            UInt32 eventStatus = 0;
            FT_GetStatus( handle, ref rx, ref tx, ref eventStatus );
            return (int)tx;
        }

        private static void writeData()
        {
            uint handle = 0;
            while (!done || valuesChanged)
            {
                if( !valuesChanged ) {
                    continue;
                }
                
                status = GetStatus(ref handle);
                if( status == FT_STATUS.FT_INVALID_HANDLE || status == FT_STATUS.FT_DEVICE_NOT_FOUND || status == FT_STATUS.FT_DEVICE_NOT_OPENED ){
                    try {
                        handle = 0;
                        status = FT_Open(0, ref handle);
                        // This is really important, there has to be a delay before sending to the DMX controller
                        Thread.Sleep(5);
                        
                        initOpenDMX(handle);                            
                        FT_SetBreakOn(handle);
                        FT_SetBreakOff(handle);
                    }
                    catch(Exception e){
                        FT_Close(handle);
                        Debug.LogError(e.ToString());
                        thread = null;
                        return;
                    }
                }
                
                bytesWritten = write(handle, buffer, buffer.Length);
                //Debug.LogFormat("Bytes written {0} tx={1} status={2}",bytesWritten, GetTxBuffer(ref handle), status);
                valuesChanged = false;
                FT_Close(handle);
                
            }
            
            thread = null;
            Debug.Log("Stopping DMX comms thread");
        }

        private static int write(uint handle, byte[] data, int length)
        {
            IntPtr ptr = Marshal.AllocHGlobal((int)length);
            Marshal.Copy(data, 0, ptr, (int)length);
            uint bytesWritten = 0;
            status = FT_Write(handle, ptr, (uint)length, ref bytesWritten);
            return (int)bytesWritten;
        }

        private static void initOpenDMX(uint handle)
        {
            status = FT_ResetDevice(handle);
            //status = FT_SetDivisor(handle, (char)12);  // legacy method
            status = FT_SetBaudRate(handle, 250000);
            status = FT_SetDataCharacteristics(handle, BITS_8, STOP_BITS_2, PARITY_NONE);
            status = FT_SetFlowControl(handle, (char)FLOW_NONE, 0, 0);
            status = FT_ClrRts(handle);
            status = FT_Purge(handle, PURGE_TX);
            status = FT_Purge(handle, PURGE_RX);
        }
#else
        public static void start(){}
        public static void stop(){}
        public static void setDmxValue(int channel, byte value){}
        public static void setRange(int start, int end, byte value){}
#endif
    }

    /// <summary>
    /// Enumaration containing the varios return status for the DLL functions.
    /// </summary>
    public enum FT_STATUS
    {
        FT_OK = 0,
        FT_INVALID_HANDLE,
        FT_DEVICE_NOT_FOUND,
        FT_DEVICE_NOT_OPENED,
        FT_IO_ERROR,
        FT_INSUFFICIENT_RESOURCES,
        FT_INVALID_PARAMETER,
        FT_INVALID_BAUD_RATE,
        FT_DEVICE_NOT_OPENED_FOR_ERASE,
        FT_DEVICE_NOT_OPENED_FOR_WRITE,
        FT_FAILED_TO_WRITE_DEVICE,
        FT_EEPROM_READ_FAILED,
        FT_EEPROM_WRITE_FAILED,
        FT_EEPROM_ERASE_FAILED,
        FT_EEPROM_NOT_PRESENT,
        FT_EEPROM_NOT_PROGRAMMED,
        FT_INVALID_ARGS,
        FT_OTHER_ERROR
    };



}