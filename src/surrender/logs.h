#ifndef LOGS_H
#define LOGS_H

#include <iostream>

#define LOG_INFO()  std::clog << "[info] "
#define LOG_ERROR()  std::cerr << "[error] "
#define LOG_DEBUG()  std::clog << "[debug] "
#define LOG_WARNING()  std::cerr << "[warning] "

#undef DEBUG
#ifndef DEBUG
#define CHECK_GL()
#else

namespace
{
    //! \brief Takes an error code and returns a more explicit error message
    const char *getErrorString(int err)
    {
        switch(err)
        {
        case 0: return "GL_NO_ERROR";
        case 0x0500:    return "GL_INVALID_ENUM";
        case 0x0501:    return "GL_INVALID_VALUE";
        case 0x0502:    return "GL_INVALID_OPERATION";
        case 0x0503:    return "GL_STACK_OVERFLOW";
        case 0x0504:    return "GL_STACK_UNDERFLOW";
        case 0x0505:    return "GL_OUT_OF_MEMORY";
        default:
            return "unknown error";
        }
    }
}

#define CHECK_GL() \
    do\
    {\
        GLenum err = glGetError();\
        if (err != GL_NO_ERROR)\
            LOG_ERROR() << ' ' << __FILE__ << " l." << __LINE__ << " (" << err << ") " << getErrorString(err) << std::endl;\
    } while(false)
#endif

#endif // LOGS_H
