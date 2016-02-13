struct IFrameDescription;

using KinectFramesData = struct KinectFramesData
{
    // Depth frame data
    static const int DEPTH_FRAME_WIDTH  = 512;
    static const int DEPTH_FRAME_HEIGHT = 424;
    static const int DEPTH_FRAME_SIZE   = DEPTH_FRAME_WIDTH * DEPTH_FRAME_HEIGHT;

    unsigned int depthBufferSize = 0;
    unsigned short * depthBuffer = nullptr;
    unsigned short * depthBufferSavedData = nullptr;

    inline unsigned int* GetPointerToDepthBufferSize();
    inline unsigned short** GetPointerToDepthBuffer();
    inline bool CopyDepthBuffer() const;

    // Color frame data
    static const int COLOR_FRAME_WIDTH  = 1920;
    static const int COLOR_FRAME_HEIGHT = 1080;
    static const int COLOR_FRAME_SIZE   = COLOR_FRAME_WIDTH * COLOR_FRAME_HEIGHT;

    unsigned int colorBufferSize = 0;
    RGBQUAD * colorBuffer        = nullptr;
    ColorImageFormat imageFormat = ColorImageFormat_None;
    unsigned char * rgbMapDepthBuffer = nullptr;
  
    ColorSpacePoint * depthMappedToColorPoints = nullptr;

    inline unsigned int* GetPointerToColorBufferSize();
    inline RGBQUAD** GetPointerToColorBuffer();

    RGBQUAD * colorBufferRGBX = nullptr;;

    // Body index frame data
    unsigned int bodyIndexBufferSize = 0;
    unsigned char * bodyIndexBuffer  = nullptr;

    static const int BYTES_PER_PIXEL = 4;

    KinectFramesData();
    ~KinectFramesData();
};

inline unsigned* ::KinectFramesData::GetPointerToDepthBufferSize()
{
    return &depthBufferSize;
}

inline unsigned short** ::KinectFramesData::GetPointerToDepthBuffer()
{
    return &depthBuffer;
}

inline bool KinectFramesData::CopyDepthBuffer() const
{
    if (depthBuffer != nullptr)
    {
        memcpy(depthBufferSavedData, depthBuffer, DEPTH_FRAME_SIZE * sizeof (unsigned short));
        return true;
    }
    return false;
}

inline unsigned* ::KinectFramesData::GetPointerToColorBufferSize()
{
    return &colorBufferSize;
}

inline RGBQUAD** ::KinectFramesData::GetPointerToColorBuffer()
{
    return &colorBuffer;
}

inline KinectFramesData::KinectFramesData()
{
    colorBufferRGBX          = new RGBQUAD[COLOR_FRAME_SIZE];
    rgbMapDepthBuffer        = new unsigned char[BYTES_PER_PIXEL * DEPTH_FRAME_SIZE];
    memset(rgbMapDepthBuffer, 0, BYTES_PER_PIXEL * DEPTH_FRAME_SIZE);

    depthMappedToColorPoints = new ColorSpacePoint[DEPTH_FRAME_SIZE];
    depthBufferSavedData     = new unsigned short[DEPTH_FRAME_SIZE];
    memset(depthBufferSavedData, 0, DEPTH_FRAME_SIZE * 2);
}

inline KinectFramesData::~KinectFramesData()
{
    if (colorBufferRGBX)
    {
        delete[] colorBufferRGBX;
        colorBufferRGBX = nullptr;
    }

    if (rgbMapDepthBuffer)
    {
        delete[] rgbMapDepthBuffer;
        rgbMapDepthBuffer = nullptr;
    }

    if (depthMappedToColorPoints)
    {
        delete[] depthMappedToColorPoints;
        depthMappedToColorPoints = nullptr;
    }

    if (depthBufferSavedData)
    {
        delete[] depthBufferSavedData;
        depthBufferSavedData = nullptr;
    }
}

typedef struct PointCloudData
{

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    
    unsigned char r = 0;
    unsigned char g = 0;
    unsigned char b = 0;
    
    unsigned char bodyIndex = 0;

    PointCloudData();
    ~PointCloudData();

    void SetData(const float &x, const float &y, const float &z,
                 const float &r, const float &g, const float &b,
                 const float &bodyIndex);

} PointCloudData;

inline PointCloudData::PointCloudData()
{
}

inline PointCloudData::~PointCloudData()
{
}

inline void PointCloudData::SetData(const float& xValue, const float& yValue, const float& zValue,
                                    const float& rValue, const float& gValue, const float& bValue,
                                    const float& bodyIndexValue)
{
    x = xValue;
    y = yValue;
    z = zValue;

    r = rValue;
    g = gValue;
    b = bValue;

    bodyIndex = bodyIndexValue;
}