#ifdef WIN32
#include <GL/glew.h>
#endif
#define GL_GLEXT_PROTOTYPES
#include "fbo.h"

#include <GL/glu.h>
#include "logs.h"

namespace luxifer
{

    FBO::FBO(GLint w, GLint h, GLuint format)
        : w(w),
        h(h)
    {
        CHECK_GL();

        glGenFramebuffers(1, &hFBO);
        CHECK_GL();
        glGenTextures(1, &color_buffer);
        CHECK_GL();
        glGenTextures(1, &depth_buffer);
        CHECK_GL();

        glBindTexture(GL_TEXTURE_2D, color_buffer);
        CHECK_GL();
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        CHECK_GL();
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        CHECK_GL();
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
        CHECK_GL();
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        CHECK_GL();
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        CHECK_GL();
        glTexImage2D(GL_TEXTURE_2D, 0, format, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
        CHECK_GL();
        color_target = GL_TEXTURE_2D;
        bAutoDeleteColorBuffer = true;

        glBindTexture(GL_TEXTURE_2D, depth_buffer);
        CHECK_GL();
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        CHECK_GL();
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        CHECK_GL();
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
        CHECK_GL();
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        CHECK_GL();
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        CHECK_GL();
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, w, h, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
        CHECK_GL();
        bAutoDeleteDepthBuffer = true;
    }

    FBO::FBO(GLint target, GLint color_buffer, GLint nw, GLint nh)
        : w(0),
        h(0)
    {
        CHECK_GL();
        glGenFramebuffers(1, &hFBO);
        CHECK_GL();
        glGenTextures(1, &depth_buffer);
        CHECK_GL();
        this->color_buffer = 0;

        bAutoDeleteColorBuffer = false;
        bAutoDeleteDepthBuffer = true;
        setColorTarget(target, color_buffer, nw, nh);
    }

    FBO::~FBO()
    {
        CHECK_GL();
        glDeleteFramebuffers(1, &hFBO);
        CHECK_GL();
        if (bAutoDeleteColorBuffer)
        {
            glDeleteTextures(1, &color_buffer);
            CHECK_GL();
        }
        if (bAutoDeleteDepthBuffer)
        {
            glDeleteTextures(1, &depth_buffer);
            CHECK_GL();
        }
    }

    GLuint FBO::getColorTexture() const
    {
        return color_buffer;
    }

    GLuint FBO::getColorTarget() const
    {
        return color_target;
    }

    GLuint FBO::getDepthTexture() const
    {
        return depth_buffer;
    }

    void FBO::bind() const
    {
        CHECK_GL();
        glBindFramebuffer(GL_FRAMEBUFFER, hFBO);
        CHECK_GL();
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, color_target, color_buffer, 0);
        CHECK_GL();
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth_buffer, 0);
        CHECK_GL();
        GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        CHECK_GL();
        if (status != GL_FRAMEBUFFER_COMPLETE)
        {
            std::cerr << color_target << " - " << color_buffer << std::endl;
            std::cerr << "Framebuffer error : ";
            switch(status)
            {
            case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
                std::cerr << "GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT" << std::endl;
                break;
            case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
                std::cerr << "GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT" << std::endl;
                break;
            case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
                std::cerr << "GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER" << std::endl;
                break;
            case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
                std::cerr << "GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER" << std::endl;
                break;
            case GL_FRAMEBUFFER_UNSUPPORTED:
                std::cerr << "GL_FRAMEBUFFER_UNSUPPORTED" << std::endl;
                break;
            default:
                std::cerr << status << std::endl;
            }
        }
        glViewport(0,0,w,h);
        CHECK_GL();
    }

    void FBO::release() const
    {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        CHECK_GL();
    }

    void FBO::setColorTarget(GLuint target, GLuint color_buffer, GLint nw, GLint nh)
    {
        if (color_buffer == this->color_buffer && target == color_target)
            return;
        if (bAutoDeleteColorBuffer && color_buffer != this->color_buffer)
        {
            glDeleteTextures(1, &(this->color_buffer));
            CHECK_GL();
        }

        this->color_buffer = color_buffer;
        color_target = target;
        bAutoDeleteColorBuffer = false;

        if (nw != w || nh != h)
        {
            if (!bAutoDeleteDepthBuffer)
            {
                glGenTextures(1, &depth_buffer);
                CHECK_GL();
                bAutoDeleteDepthBuffer = true;
            }
            w = nw;
            h = nh;
            glBindTexture(GL_TEXTURE_2D, depth_buffer);
            CHECK_GL();
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            CHECK_GL();
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            CHECK_GL();
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
            CHECK_GL();
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            CHECK_GL();
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            CHECK_GL();
            glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, w, h, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
            CHECK_GL();
        }
    }

    void FBO::setDepthTarget(GLuint depth_buffer, GLint nw, GLint nh)
    {
        if (depth_buffer == this->depth_buffer)
            return;
        if (bAutoDeleteDepthBuffer && depth_buffer != this->depth_buffer)
        {
            glDeleteTextures(1, &(this->depth_buffer));
            CHECK_GL();
        }

        this->depth_buffer = depth_buffer;
        bAutoDeleteDepthBuffer = false;

        if (nw != w || nh != h)
        {
            if (!bAutoDeleteColorBuffer)
            {
                glGenTextures(1, &color_buffer);
                CHECK_GL();
                bAutoDeleteColorBuffer = true;
            }
            w = nw;
            h = nh;
            color_target = GL_TEXTURE_2D;
            glBindTexture(GL_TEXTURE_2D, color_buffer);
            CHECK_GL();
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            CHECK_GL();
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            CHECK_GL();
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
            CHECK_GL();
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            CHECK_GL();
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            CHECK_GL();
            glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
            CHECK_GL();
        }
    }

    GLuint FBO::getWidth() const
    {
        return w;
    }

    GLuint FBO::getHeight() const
    {
        return h;
    }
}
