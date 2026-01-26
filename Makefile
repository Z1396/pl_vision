# 海康 MVS SDK 头文件路径
INCLUDE += -I/opt/MVS/include
INCLUDE += -I/opt/MVS/include/GenICam
INCLUDE += -I/opt/MVS/include/ThirdParty

# 海康 MVS SDK 库链接（核心：libMvCameraControl.so）
LIBS += -L/opt/MVS/lib/64 
LIBS += -lMvCameraControl -lMvUsb3vTL -lMVGigEVisionSDK -lusb-1.0
LIBS += -lpthread -lrt -ldl -lstdc++
