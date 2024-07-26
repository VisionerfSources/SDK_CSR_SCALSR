// See https://aka.ms/new-console-template for more information
using System.Net;
using System.Net.NetworkInformation;
using System.Runtime.InteropServices;


[DllImport("C:\\VNDepotExtern\\SDK_CSR_SCALSR\\lib\\Win\\Win64\\VnCirrus3DLib.dll", SetLastError = true, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
static extern int VN_CirrusCom_Init();

[DllImport("C:\\VNDepotExtern\\SDK_CSR_SCALSR\\lib\\Win\\Win64\\VnCirrus3DLib.dll", SetLastError = true, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
static extern int VN_ExecuteCommandSTS([MarshalAs(UnmanagedType.LPStr)] string pIPAddress,
                                       ref int pStatus);

[DllImport("C:\\VNDepotExtern\\SDK_CSR_SCALSR\\lib\\Win\\Win64\\VnCirrus3DLib.dll", SetLastError = true, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
static extern int VN_Cirrus3DHandler_Initialize([MarshalAs(UnmanagedType.LPStr)] string pIPAddress, out Cirrus3DHandler pCirrus3DHandler);


int status;
string str="192.168.5.222";

Console.WriteLine("Test VN_CirrusCom_Init function");
int err= VN_CirrusCom_Init();
Console.WriteLine(err);

err = VN_ExecuteCommandSTS(str, ref status);
Console.WriteLine(err);


err = VN_Cirrus3DHandler_Initialize(str,
                                    out Cirrus3DHandler toto);
Console.WriteLine(err);




public struct Cirrus3DHandler
{
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 256)]
    public char[] IPAddress; // [Read only] 
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 256)]
    public char[] SerialNumber; // [Read only] 
    public uint ProtocolVersion { get; set; } // [Read only] 
    public uint HeaderVersion { get; set; } // [Read only] 
    public uint DeviceVersion { get; set; } // [Read only] 
}








