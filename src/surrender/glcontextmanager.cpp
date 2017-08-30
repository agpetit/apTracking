#include <GL/glew.h>
#include "glcontextmanager.h"
#include <osg/Image>
#include <iostream>
#include "logs.h"

namespace luxifer
{

    GLContextManager::GLContextManager(const QGLContext *pContext)
        : pContext(const_cast<QGLContext*>(pContext))
    {
        bOwn = false;
        makeCurrent();
        glewInit();
        bHasVBOSupport = GLEW_ARB_vertex_buffer_object;
    }

    GLContextManager::~GLContextManager()
    {
        clear();
        if (bOwn)
            delete pContext;
    }

    GLuint GLContextManager::getTextureID(const osg::Texture2D *ptex)
    {
        if (!ptex || !pContext->isValid())
            return 0;

        if (hTextures.count(ptex))
            return hTextures[ptex];

        makeCurrent();
        GLuint id;
        glGenTextures(1, &id);
        CHECK_GL();
        glBindTexture(GL_TEXTURE_2D, id);
        CHECK_GL();
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, ptex->getFilter(osg::Texture2D::MIN_FILTER));
        CHECK_GL();
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, ptex->getFilter(osg::Texture2D::MAG_FILTER));
        CHECK_GL();
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, ptex->getWrap(osg::Texture2D::WRAP_S));
        CHECK_GL();
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, ptex->getWrap(osg::Texture2D::WRAP_T));
        CHECK_GL();
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, ptex->getWrap(osg::Texture2D::WRAP_R));
        CHECK_GL();

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, ptex->getMaxAnisotropy());
        CHECK_GL();

        bool bUseMipmaps = false;

        switch(ptex->getFilter(osg::Texture2D::MIN_FILTER))
        {
        case GL_LINEAR_MIPMAP_LINEAR:
        case GL_LINEAR_MIPMAP_NEAREST:
        case GL_NEAREST_MIPMAP_LINEAR:
        case GL_NEAREST_MIPMAP_NEAREST:
            bUseMipmaps = true;
            break;
        }

        switch(ptex->getFilter(osg::Texture2D::MAG_FILTER))
        {
        case GL_LINEAR_MIPMAP_LINEAR:
        case GL_LINEAR_MIPMAP_NEAREST:
        case GL_NEAREST_MIPMAP_LINEAR:
        case GL_NEAREST_MIPMAP_NEAREST:
            bUseMipmaps = true;
            break;
        }

        glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, bUseMipmaps ? GL_TRUE : GL_FALSE);
        CHECK_GL();

        const osg::Image *image = ptex->getImage();
        if (!image)
        {
            switch(ptex->getInternalFormat())
            {
            case GL_DEPTH_COMPONENT:
            case GL_DEPTH_COMPONENT16:
            case GL_DEPTH_COMPONENT24:
            case GL_DEPTH_COMPONENT32:
            case GL_DEPTH_COMPONENT32F:
                if (ptex->getShadowComparison())
                {
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);
                    CHECK_GL();
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LESS);
                    CHECK_GL();
                }
                else
                {
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);
                    CHECK_GL();
                }
                glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, ptex->getShadowTextureMode());
                CHECK_GL();

                glTexImage2D(GL_TEXTURE_2D, 0, ptex->getInternalFormat(), ptex->getTextureWidth(), ptex->getTextureHeight(), 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
                CHECK_GL();
                break;
            default:
                glTexImage2D(GL_TEXTURE_2D, 0, ptex->getInternalFormat(), ptex->getTextureWidth(), ptex->getTextureHeight(), 0, GL_LUMINANCE, GL_FLOAT, NULL);
                CHECK_GL();
            }
        }
        else
        {
            glTexImage2D(GL_TEXTURE_2D, 0, image->getInternalTextureFormat(), image->s(), image->t(), 0, image->getPixelFormat(), image->getDataType(), image->data());
            CHECK_GL();
        }

        hTextures[ptex] = id;

        return id;
    }

    GLuint GLContextManager::getTextureID(const osg::TextureCubeMap *ptex)
    {
        if (!ptex || !pContext->isValid())
            return 0;

        if (hTextures.count(ptex))
            return hTextures[ptex];

        makeCurrent();
        GLuint id;
        glGenTextures(1, &id);
        CHECK_GL();
        glBindTexture(GL_TEXTURE_CUBE_MAP, id);
        CHECK_GL();
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, ptex->getFilter(osg::TextureCubeMap::MIN_FILTER));
        CHECK_GL();
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, ptex->getFilter(osg::TextureCubeMap::MAG_FILTER));
        CHECK_GL();
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, ptex->getWrap(osg::TextureCubeMap::WRAP_S));
        CHECK_GL();
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, ptex->getWrap(osg::TextureCubeMap::WRAP_T));
        CHECK_GL();
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, ptex->getWrap(osg::TextureCubeMap::WRAP_R));
        CHECK_GL();
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAX_ANISOTROPY_EXT, ptex->getMaxAnisotropy());
        CHECK_GL();
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_GENERATE_MIPMAP, GL_FALSE);
        CHECK_GL();

        for(int i = 0 ; i < 6 ; ++i)
        {
            const GLuint target = GL_TEXTURE_CUBE_MAP_POSITIVE_X + i;
            glTexImage2D(target, 0, ptex->getInternalFormat(), ptex->getTextureWidth(), ptex->getTextureHeight(), 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
            CHECK_GL();
        }

        hTextures[ptex] = id;

        return id;
    }

    GLuint GLContextManager::getProgramID(const osg::Program *program)
    {
        if (!program || !pContext->isValid())
            return 0;

        if (hPrograms.count(program))
            return hPrograms[program];

        makeCurrent();
        GLuint pid = glCreateProgram();
        CHECK_GL();
        hPrograms[program] = pid;

        for(unsigned int i = 0 ; i < program->getNumShaders() ; ++i)
        {
            const osg::Shader *shader = program->getShader(i);
            GLuint sid = getShaderID(shader);
            glAttachShader(pid, sid);
            CHECK_GL();
        }

        glLinkProgram(pid);
        CHECK_GL();
        GLint linked;
        glGetProgramiv(pid, GL_LINK_STATUS, &linked);
        CHECK_GL();
        if (!linked)
            std::cerr << "program link error" << std::endl;

        const GLsizei max_len = 16384;
        GLchar *buffer = new GLchar[max_len + 1];
        memset(buffer, 0, max_len + 1);
        GLsizei len;
        glGetProgramInfoLog(pid, max_len, &len, buffer);
        CHECK_GL();

        if (len > 0)
        {
            std::cerr.write(buffer, len);
            std::cerr << std::endl;
            std::cerr << std::endl;
        }

        delete[] buffer;

        return pid;
    }

    GLuint GLContextManager::getShaderID(const osg::Shader *shader)
    {
        if (!shader || !pContext->isValid())
            return 0;

        if (hShaders.count(shader))
            return hShaders[shader];

        makeCurrent();
        GLuint sid = glCreateShader(shader->getType());
        CHECK_GL();
        hShaders[shader] = sid;

        const std::string &source = shader->getShaderSource();
        const GLchar *source_data = source.data();
        const GLint source_size = source.size();
        glShaderSource(sid, 1, &source_data, &source_size);
        CHECK_GL();
        glCompileShader(sid);
        CHECK_GL();

        return sid;
    }

    void GLContextManager::clear()
    {
        makeCurrent();
        for(globject_table_t::const_iterator it = hTextures.begin(), end = hTextures.end() ; it != end ; ++it)
        {
            glDeleteTextures(1, &it->second);
            CHECK_GL();
        }
        for(globject_table_t::const_iterator it = hPrograms.begin(), end = hPrograms.end() ; it != end ; ++it)
        {
            glDeleteProgram(it->second);
            CHECK_GL();
        }
        for(globject_table_t::const_iterator it = hShaders.begin(), end = hShaders.end() ; it != end ; ++it)
        {
            glDeleteShader(it->second);
            CHECK_GL();
        }
        hTextures.clear();
        hPrograms.clear();
        hShaders.clear();
    }

    GLuint GLContextManager::getDataVBOID(const void *data, const size_t size)
    {
        if (hVBO.count(data))
            return hVBO[data];

        makeCurrent();

        GLuint vbo;
        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, size, data, GL_STATIC_DRAW);

        hVBO[data] = vbo;

        return vbo;
    }

    GLuint GLContextManager::getIndexVBOID(const void *data, const size_t size)
    {
        if (hVBO.count(data))
            return hVBO[data];

        makeCurrent();

        GLuint vbo;
        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, size, data, GL_STATIC_DRAW);

        hVBO[data] = vbo;

        return vbo;
    }
}
