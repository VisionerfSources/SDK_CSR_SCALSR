// See https://aka.ms/new-console-template for more information
using System.Diagnostics;
using System.Net;
using System.Net.NetworkInformation;
using System.Runtime.InteropServices;
using System.Text;

const string libPath = "C:\\VNDepotExtern\\SDK_CSR_SCALSR\\lib\\Win\\Win64\\dll\\VNCirrus3DLib.dll";

[DllImport(libPath, SetLastError = true, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
static extern int VN_CirrusCom_Init();

[DllImport(libPath, SetLastError = true, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
static extern int VN_CirrusCom_End();

[DllImport(libPath, SetLastError = true, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
static extern int VN_ExecuteCommandSTS([MarshalAs(UnmanagedType.LPStr)] string pIPAddress,
                                       ref int pStatus);

[DllImport(libPath, SetLastError = true, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
static extern int VN_Cirrus3DHandler_Initialize([MarshalAs(UnmanagedType.LPStr)] string pIPAddress, out Cirrus3DHandler pCirrus3DHandler);

[DllImport(libPath, SetLastError = true, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
static extern int VN_ExecuteCommandRECI([MarshalAs(UnmanagedType.LPStr)] string pIPAddress, int ConfigId);

[DllImport(libPath, SetLastError = true, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
static extern int VN_ExecuteCommandINFO([MarshalAs(UnmanagedType.LPStr)] string pIPAddress, int InfoType, [MarshalAs(UnmanagedType.LPStr)] StringBuilder pResult, int resultSize);

[DllImport(libPath, SetLastError = true, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
static extern int VN_ExecuteSCAN_XYZ(
    [MarshalAs(UnmanagedType.LPStr)] string IPAddress,
    int MaxCloudSize,
    [In, Out, MarshalAs(UnmanagedType.LPArray, SizeParamIndex = 1)] byte[] pCloud_XYZ,
    ref int pCloudSize
);

[DllImport(libPath, SetLastError = true, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
static extern int VN_SaveXYZCloudAsTxtFile(
    [In, MarshalAs(UnmanagedType.LPArray, SizeParamIndex = 1)] float[] pCloud_XYZ,
    int cloudSize,
    [MarshalAs(UnmanagedType.LPStr)] string pFileName
);

[DllImport(libPath, SetLastError = true, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
static extern int VN_SaveXYZCloudAsBinFile(
    [In, MarshalAs(UnmanagedType.LPArray, SizeParamIndex = 1)] float[] pCloud_XYZ,
    int cloudSize,
    [MarshalAs(UnmanagedType.LPStr)] string pFileName
);

[DllImport(libPath, SetLastError = true, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
static extern int VN_ExecuteSCAN_CameraCOP_MiddleCam2_XYZI(
    [MarshalAs(UnmanagedType.LPStr)] string IPAddress,
    int MaxCloudSize,
    [In, Out, MarshalAs(UnmanagedType.LPArray, SizeParamIndex = 1)] VN_Point_XYZI8[] pCloud_XYZ,
    ref int pMatrixColumns,
    ref int pMatrixRows
);

[DllImport(libPath, SetLastError = true, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
static extern int VN_SaveXYZI8CloudAsLumBmp(
    [In, Out, MarshalAs(UnmanagedType.LPArray, SizeParamIndex = 1)] VN_Point_XYZI8[] pCloud_XYZ,
    int pMatrixColumns,
    int pMatrixRows,
    [MarshalAs(UnmanagedType.LPStr)] string pFileName
);

[DllImport(libPath, SetLastError = true, CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
static extern int VN_SaveXYZI8CloudAsPcdFile(
    [In, Out, MarshalAs(UnmanagedType.LPArray, SizeParamIndex = 1)] VN_Point_XYZI8[] pCloud_XYZ,
    int pMatrixColumns,
    int pMatrixRows,
    [MarshalAs(UnmanagedType.LPStr)] string pFileName
);

int status;
int err;
string CirrusIP = "192.168.1.201";
int Config = 1000;
StringBuilder CirrusInfo = new StringBuilder(64);

Console.WriteLine("Test VN_CirrusCom_Init function");
err= VN_CirrusCom_Init();
Console.WriteLine(err);

err = VN_ExecuteCommandSTS(CirrusIP, ref status);
Console.WriteLine(err);

err = VN_ExecuteCommandRECI(CirrusIP, Config);
Console.WriteLine(err);

err = VN_ExecuteCommandINFO(CirrusIP, 1, CirrusInfo, CirrusInfo.Capacity);
Console.WriteLine(err);


const int MaxCloudSize = 6000000;  
byte[] CloudData = new byte[MaxCloudSize * 3 * sizeof(float)];  
int ActualCloudSize = 0;

err = VN_ExecuteSCAN_XYZ(CirrusIP, MaxCloudSize, CloudData, ref ActualCloudSize);
if (err == 0)
{
    float[] CloudPoints = new float[ActualCloudSize * 3 * sizeof(float)];
    Buffer.BlockCopy(CloudData, 0, CloudPoints, 0, ActualCloudSize * 3 * sizeof(float));

    err = VN_SaveXYZCloudAsTxtFile(CloudPoints, ActualCloudSize, "./PointCloud_1.txt");
    Console.WriteLine(err);

    err = VN_SaveXYZCloudAsBinFile(CloudPoints, ActualCloudSize, "./PointCloud_1.bin");
    Console.WriteLine(err);

}
else
{
    Console.WriteLine("Error occurred with code: " + err);
}

int ActualMatrixColumns = 0;
int ActualMatrixRows = 0;
VN_Point_XYZI8[] pCloudXYZI8= new VN_Point_XYZI8[MaxCloudSize];

err = VN_ExecuteSCAN_CameraCOP_MiddleCam2_XYZI(CirrusIP, MaxCloudSize, pCloudXYZI8, ref ActualMatrixColumns, ref ActualMatrixRows);
if (err == 0)
{
    err = VN_SaveXYZI8CloudAsLumBmp(pCloudXYZI8, ActualMatrixRows, ActualMatrixColumns, "./PointCloud_2.bmp");
    Console.WriteLine(err);

    err = VN_SaveXYZI8CloudAsPcdFile(pCloudXYZI8, ActualMatrixRows, ActualMatrixColumns, "./PointCloud_2.bin");
    Console.WriteLine(err);

}
else
{
    Console.WriteLine("Error occurred with code: " + err);
}



err = VN_Cirrus3DHandler_Initialize(CirrusIP,
                                    out Cirrus3DHandler handler);
Console.WriteLine(err);



err = VN_CirrusCom_End();
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

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct VN_Point_XYZI8
{
    public float x;
    public float y;
    public float z;
    public uint i;
}




