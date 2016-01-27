// KinectRemoteDataSender.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "FrameDataStruct.h"
#include <atomic>
#include <ws2def.h>
#include <mutex>
#include <cmath>

#pragma comment(lib,"ws2_32.lib")

#define BUFFER_LENGHT 512
#define MAX_PACKET_SIZE 54272

void KinectFrameReader(std::atomic<bool> & isRunning);
void UdpDataSender(std::atomic<bool> & isRunning);

template<class I> inline void SafeRelease(I *& interfaceToRelease);

inline bool Succeeded(long value);
inline bool Failed(long value);

inline void LogSucceeded(char * message);
inline void LogError(char * message);

std::mutex logMutex;
KinectFramesData * kinectFramesDataRef = nullptr;
bool getDepthData = false;
bool getColorData = false;

int _tmain(int argc, _TCHAR* argv[])
{
    auto key = 'y';

    kinectFramesDataRef = new KinectFramesData();

    std::atomic<bool> kinectFrameReaderThreadRunFlag = true;
    std::thread kinectFrameReaderThread;
    
    kinectFrameReaderThread = std::thread(KinectFrameReader, std::ref(kinectFrameReaderThreadRunFlag));
    
    std::atomic<bool> udpDataSenderThreadRunFlag = true;
    std::thread udpDataSenderThread;

    udpDataSenderThread = std::thread(UdpDataSender, std::ref(udpDataSenderThreadRunFlag));;
    
    while (key != 'n')
    {
        if (_kbhit())
        {
            key = _getch();
            // TODO add remake ip and port
        }
    }

    kinectFrameReaderThreadRunFlag = false;
    udpDataSenderThreadRunFlag = false;
    
    // Release add data
    kinectFrameReaderThread.join();
    udpDataSenderThread.join();

    return 0;
}

void ProcessFrame(ICoordinateMapper * coordinateMapper)
{
    if (coordinateMapper)
    {
        auto result = coordinateMapper->MapDepthFrameToColorSpace(KinectFramesData::DEPTH_FRAME_SIZE, kinectFramesDataRef->depthBuffer,
                                                                  KinectFramesData::DEPTH_FRAME_SIZE, kinectFramesDataRef->depthMappedToColorPoints);

        if (Succeeded(result))
        {
            for (auto depthIndex = 0; depthIndex < KinectFramesData::DEPTH_FRAME_SIZE; ++depthIndex)
            {
                auto & point = kinectFramesDataRef->depthMappedToColorPoints[depthIndex];
                int colorX = floor(point.X + 0.5f);
                int colorY = floor(point.Y + 0.5f);

                auto & depth = kinectFramesDataRef->depthBuffer[depthIndex];

                if ((colorX >= 0) && (colorX < KinectFramesData::COLOR_FRAME_WIDTH) && 
                    (colorY >= 0) && (colorY < KinectFramesData::DEPTH_FRAME_HEIGHT))
                {
                    auto colorImageIndex = ((KinectFramesData::COLOR_FRAME_WIDTH * colorY) + colorX);
                    auto depthPixel = depthIndex * KinectFramesData::BYTES_PER_PIXEL;

                    kinectFramesDataRef->rgbMapDepthBuffer[depthIndex]     = kinectFramesDataRef->colorBuffer[colorImageIndex].rgbRed;
                    kinectFramesDataRef->rgbMapDepthBuffer[depthIndex + 1] = kinectFramesDataRef->colorBuffer[colorImageIndex].rgbGreen;
                    kinectFramesDataRef->rgbMapDepthBuffer[depthIndex + 2] = kinectFramesDataRef->colorBuffer[colorImageIndex].rgbBlue;
                    kinectFramesDataRef->rgbMapDepthBuffer[depthIndex + 3] = kinectFramesDataRef->bodyIndexBuffer ?
                                                                             kinectFramesDataRef->bodyIndexBuffer[depthIndex] :
                                                                             0;
                }
            }

            getColorData = true;
        }
    }
}

void KinectFrameUpdate(IMultiSourceFrameReader * multiSourceFrameReader,
                       ICoordinateMapper * coordinateMapper,
                       std::atomic<bool> & isRunning)
{
    auto * cameraSpacePoints = new CameraSpacePoint[KinectFramesData::COLOR_FRAME_SIZE];
    while (isRunning)
    {
        if (multiSourceFrameReader == nullptr)
        {
            std::cout << "ERROR: KinectFrameUpdate DepthFrameReader is null";
            return;
        }

        IMultiSourceFrame * multiSourceFrame = nullptr;
        IDepthFrame * depthFrame = nullptr;
        IColorFrame * colorFrame = nullptr;
        IBodyIndexFrame * bodyIndexFrame = nullptr;

        // Multi source frame reader acquire latest frame
        auto result = multiSourceFrameReader->AcquireLatestFrame(&multiSourceFrame);
        if (Succeeded(result))
        {
            //LogSucceeded("Multi source frame reader acquire latest frame");

            // Get depth frame reference
            IDepthFrameReference * depthFrameReference = nullptr;
            result = multiSourceFrame->get_DepthFrameReference(&depthFrameReference);

            if (Succeeded(result))
            {
                //LogSucceeded("Get depth frame reference");
                result = depthFrameReference->AcquireFrame(&depthFrame);
            }
            else
            {
                LogError("KinectFrameUpdate Can't get depth frame reference");
            }
            SafeRelease(depthFrameReference);

            if (Succeeded(result))
            {
                // Get color frame reference
                IColorFrameReference * colorFrameReference = nullptr;
                result = multiSourceFrame->get_ColorFrameReference(&colorFrameReference);

                if (Succeeded(result))
                {
                    //LogSucceeded("Get color frame reference");
                    result = colorFrameReference->AcquireFrame(&colorFrame);
                }
                else
                {
                    LogError("KinectFrameUpdate Can't get depth frame reference");
                }
                SafeRelease(colorFrameReference);
            }
            else
            {
                // LogError("KinectFrameUpdate Kinect error");
            }

            if (Succeeded(result))
            {
                // Get body index frame reference
                IBodyIndexFrameReference * bodyIndexFrameReference = nullptr;
                result = multiSourceFrame->get_BodyIndexFrameReference(&bodyIndexFrameReference);

                if (Succeeded(result))
                {
                    //LogSucceeded("Get body index frame reference");
                    result = bodyIndexFrameReference->AcquireFrame(&bodyIndexFrame);
                }
                else
                {
                    LogError("KinectFrameUpdate Can't body index frame reference");
                }
                SafeRelease(bodyIndexFrameReference);
            }
            else
            {
                // LogError("KinectFrameUpdate Kinect error");
            }

            // Process frames data
            if (Succeeded(result))
            {
                // Get depth size and buffer
                result = depthFrame->AccessUnderlyingBuffer(kinectFramesDataRef->GetPointerToDepthBufferSize(),
                                                            kinectFramesDataRef->GetPointerToDepthBuffer());
                getDepthData = kinectFramesDataRef->CopyDepthBuffer();
            }

            if (Succeeded(result))
            {
                if (kinectFramesDataRef->imageFormat == ColorImageFormat_Bgra)
                {
                    result = colorFrame->AccessRawUnderlyingBuffer(kinectFramesDataRef->GetPointerToColorBufferSize(),
                                                                   reinterpret_cast<BYTE**>(kinectFramesDataRef->GetPointerToColorBuffer()));
                }
                else if (kinectFramesDataRef->colorBufferRGBX)
                {
                    kinectFramesDataRef->colorBuffer = kinectFramesDataRef->colorBufferRGBX;
                    kinectFramesDataRef->colorBufferSize = KinectFramesData::COLOR_FRAME_SIZE * sizeof(RGBQUAD);
                    result = colorFrame->CopyConvertedFrameDataToArray(kinectFramesDataRef->colorBufferSize,
                                                                       reinterpret_cast<BYTE*> (kinectFramesDataRef->colorBuffer),
                                                                       ColorImageFormat_Bgra);
                }
                else
                {
                    result = E_FAIL;
                }
            }

            if (Succeeded(result))
            {
                result = bodyIndexFrame->AccessUnderlyingBuffer(&kinectFramesDataRef->bodyIndexBufferSize,
                                                                &kinectFramesDataRef->bodyIndexBuffer);
            }

            if (Succeeded(result))
            {
                ProcessFrame(coordinateMapper);
            }

            SafeRelease(depthFrame);
            SafeRelease(colorFrame);
            SafeRelease(bodyIndexFrame);
            SafeRelease(multiSourceFrame);
        }
    }
}

void KinectFrameReader(std::atomic<bool> & isRunning)
{
    logMutex.lock();
    IKinectSensor * kinectSensor = nullptr;
    auto depthCoordinate = new DepthSpacePoint[KinectFramesData::COLOR_FRAME_WIDTH * 
                                               KinectFramesData::COLOR_FRAME_HEIGHT];
    auto result = E_FAIL;

    std::cout << "\KinectFrameReader initialising kinect..." << std::endl;

    // Get Kinect Default Sensor
    result = GetDefaultKinectSensor(&kinectSensor);
    if (Failed(result))
    {
        LogError("KinectFrameReader can't get default kinect sensor.");
        logMutex.unlock();
        return;
    }
    LogSucceeded("KinectFrameReader get default kinect sensor.");
    
    // Get coordinate mapper
    ICoordinateMapper * coordinateMapper = nullptr;
    result = kinectSensor->get_CoordinateMapper(&coordinateMapper);
    if (Succeeded(result))
    {
        LogSucceeded("KinectFrameReader get coordinate mapper.");
    }
    else
    {
        LogError("KinectFrameReader can't get coordinate mapper");
    }

    // Open Sensor
    result = kinectSensor->Open();
    if (Succeeded(result))
    {
        LogSucceeded("KinectFrameReader open kinectSensor");

        // Open multi source frame reader
        IMultiSourceFrameReader * multiSourceFrameReader = nullptr;
        result = kinectSensor->OpenMultiSourceFrameReader(FrameSourceTypes_Depth |
                                                          FrameSourceTypes_Color |
                                                          FrameSourceTypes_BodyIndex,
                                                          &multiSourceFrameReader);
        if (Succeeded(result))
        {
            LogSucceeded("KinectFrameReader open multi source frame reader");
            logMutex.unlock();
            KinectFrameUpdate(multiSourceFrameReader, coordinateMapper, isRunning);
            
            SafeRelease(multiSourceFrameReader);
        }
        else
        {
            LogError("KinectFrameReader can't open multi source frame reader");
        }
    }
    else
    {
        LogError("KinectFrameReader can't open kinect sensor");
    }

    SafeRelease(coordinateMapper);

    if (kinectSensor)
    {
        kinectSensor->Close();
    }
    SafeRelease(kinectSensor);

    logMutex.unlock();
}

void UdpDataSender(std::atomic<bool> & isRunning)
{
    logMutex.lock();
    struct sockaddr_in socketAddressOther;
    int socketVal, socketLenght = sizeof(socketAddressOther);
    WSADATA wsData;
    
    // Initialise winsock
    std::cout << "\nUdpDataSender initialising winsock..." << std::endl;
    if (WSAStartup(MAKEWORD(2, 2), &wsData) != 0)
    {
        std::cout << "ERROR: UdpDataSender error Code: " << WSAGetLastError() << std::endl;
        logMutex.unlock();
        return;
    }

    std::cout << "SUCCESS: UdpDataSender initialised" << std::endl;

    // create socket
    if ((socketVal = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR)
    {
        std::cout << "ERROR: UdpDataSender socket() failed with error code : " << WSAGetLastError() << std::endl;
        logMutex.unlock();
        return;
    }

    //setup address structure
    memset(reinterpret_cast<char *>(&socketAddressOther), 0, sizeof(socketAddressOther));
    socketAddressOther.sin_family = AF_INET;
    socketAddressOther.sin_port   = htons(8888);
    socketAddressOther.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");

    auto depthBuffer       = new char[KinectFramesData::DEPTH_FRAME_SIZE];
    auto rgbMapDepthBuffer = new char[KinectFramesData::BYTES_PER_PIXEL *
                                      KinectFramesData::DEPTH_FRAME_SIZE];
    auto result = new char[MAX_PACKET_SIZE + 2];

    logMutex.unlock();

    void * handle;
    while (isRunning)
    {
        if (getDepthData && getColorData)
        {
            handle = GetStdHandle(STD_OUTPUT_HANDLE);
            SetConsoleTextAttribute(handle, FOREGROUND_BLUE);

            for (auto i = 0; i < 8; ++i)
            {
                result[0] = i;
                result[1] = i + 10;
                memcpy(result + 2, kinectFramesDataRef->depthBufferSavedData + MAX_PACKET_SIZE * i, MAX_PACKET_SIZE + 2);
                
                if (sendto(socketVal, result, MAX_PACKET_SIZE + 2, 0, reinterpret_cast<struct sockaddr *>(&socketAddressOther), socketLenght) == SOCKET_ERROR)
                {
                    std::cout << "ERROR: UdpDataSender sendto() failed with error code : " << WSAGetLastError() << std::endl;;
                }
            }
            for (auto i = 8; i < 24; ++i)
            {
                result[0] = i;
                result[1] = i + 10;
                memcpy(result + 2, rgbMapDepthBuffer + MAX_PACKET_SIZE * (i - 8), MAX_PACKET_SIZE + 2);

                if (sendto(socketVal, result, MAX_PACKET_SIZE + 2, 0, reinterpret_cast<struct sockaddr *>(&socketAddressOther), socketLenght) == SOCKET_ERROR)
                {
                    std::cout << "ERROR: UdpDataSender sendto() failed with error code : " << WSAGetLastError() << std::endl;;
                }
            }

            getDepthData = false;
            getColorData = false;
        }
    }

    delete[] result;
    result = nullptr;

    delete[] depthBuffer;
    depthBuffer = nullptr;

    delete[] rgbMapDepthBuffer;
    rgbMapDepthBuffer = nullptr;
}

template<class I> inline void SafeRelease(I *& interfaceToRelease)
{
  if (interfaceToRelease != nullptr)
  {
    interfaceToRelease->Release();
    interfaceToRelease = nullptr;
  }
}

inline bool Succeeded(long value)
{
    return value >= 0;
}

inline bool Failed(long value)
{
    return value < 0;
}

inline void LogSucceeded(char * message)
{
    auto handle = GetStdHandle(STD_OUTPUT_HANDLE);
    SetConsoleTextAttribute(handle, FOREGROUND_GREEN);
    std::cout << "SUCCESS: " << message << std::endl;
}

inline void LogError(char * message)
{
    auto handle = GetStdHandle(STD_OUTPUT_HANDLE);
    SetConsoleTextAttribute(handle, FOREGROUND_RED);
    std::cout << "ERROR: " << message << std::endl;
}