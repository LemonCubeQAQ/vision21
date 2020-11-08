//宏其命名像枚举命名一样全部大写, 使用下划线
#ifndef DEBUG_H_
#define DEBUG_H_

#define DEBUG_
#ifdef DEBUG_

    #define SHOW_DEBUG_MESSAGES_

    #define CAMERA_DEBUG_
    #ifdef CAMERA_DEBUG_

        #define DAHENG_CAMERA


    #else

        #denfine VEDIO

    #endif




#endif





#endif