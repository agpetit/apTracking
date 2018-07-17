#ifndef FBO_H
#define FBO_H

#if defined(__APPLE__)
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif 
#include <cstddef>

namespace luxifer
{

    /** \brief Wrapper around OpenGL FrameBuffer Object API
      *
      * This wrapper handles buffer allocation and deallocation automatically.
      * When binding a FBO object, the viewport is updated to fit the render target.
      */
    class FBO
    {
    public:
        FBO(GLint w, GLint h, GLuint format);
        FBO(GLint target, GLint color_buffer, GLint nw, GLint nh);
        ~FBO();

        GLuint getColorTexture() const;
        GLuint getColorTarget() const;
        GLuint getDepthTexture() const;

        void setColorTarget(GLuint target, GLuint color_buffer, GLint nw, GLint nh);
        void setDepthTarget(GLuint depth_buffer, GLint nw, GLint nh);

        void bind() const;
        void release() const;

        GLuint getWidth() const;
        GLuint getHeight() const;

    private:
        GLint w, h;
        GLuint hFBO;
        bool bAutoDeleteColorBuffer;
        bool bAutoDeleteDepthBuffer;
        GLuint color_target;
        GLuint color_buffer;
        GLuint depth_buffer;
    };

}

#endif // FBO_H
