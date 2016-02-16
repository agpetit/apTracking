#ifndef GLCONTEXTMANAGER_H
#define GLCONTEXTMANAGER_H

#include <QGLContext>
#include <map>
#include <osg/Texture2D>
#include <osg/TextureCubeMap>
#include <osg/Program>
#include <osg/Referenced>

namespace luxifer
{
    /** \brief OpenGL resource manager
      *
      * This class automatically generates OpenGL objects associated with OpenSceneGraph
      * textures, programs, shaders and buffers (VBO). This allows working directly with
      * an OpenSceneGraph representation of the scene.
      */
    class GLContextManager : public osg::Referenced
    {
        typedef std::map<const void*, GLuint>   globject_table_t;
    public:
        GLContextManager(const QGLContext *pContext);
        virtual ~GLContextManager();

        /** \brief Get the OpenGL texture handle associated with an osg::Texture2D object
          *
          * The OpenGL handle is created if needed:
          *   - texture data is uploaded
          *   - sampler state is set from osg::Texture2D properties
          */
        GLuint getTextureID(const osg::Texture2D *ptex);
        /** \brief Get the OpenGL texture handle associated with an osg::TextureCubeMap object
          *
          * The OpenGL handle is created if needed:
          *   - texture data is uploaded
          *   - sampler state is set from osg::TextureCubeMap properties
          */
        GLuint getTextureID(const osg::TextureCubeMap *ptex);
        /** \brief Get the OpenGL program handle associated with an osg::Program object
          *
          * The OpenGL handle is created if needed:
          *   - associated shader handles are created if needed with getShaderID
          *   - the program is linked
          *   - in case of failure, logs are written to stderr
          */
        GLuint getProgramID(const osg::Program *program);
        /** \brief Get the OpenGL shader handle associated with an osg::Shader object
          *
          * The OpenGL handle is created if needed:
          *   - the source code is sent to OpenGL
          *   - the shader is compiled
          */
        GLuint getShaderID(const osg::Shader *shader);
        /** \brief Get the OpenGL VBO (for Vertex Data) handle associated with a pointer
          *
          * Size must not change between calls.
          *
          * The VBO is created if needed:
          *   - the VBO is allocated
          *   - data is uploaded to video memory
          */
        GLuint getDataVBOID(const void *data, const size_t size);
        /** \brief Get the OpenGL VBO (for Index Data) handle associated with a pointer
          *
          * Size must not change between calls.
          *
          * The VBO is created if needed:
          *   - the VBO is allocated
          *   - data is uploaded to video memory
          */
        GLuint getIndexVBOID(const void *data, const size_t size);

        /** \brief Returns true if the associated OpenGL context supports VBO
          *
          * \return true is the associated OpenGL context supports VBO, false otherwise
          */
        bool hasVBOSupport() const  {   return bHasVBOSupport;  }

        /** \brief Destroy all OpenGL resources allocated by this GLContextManager
          *
          * This is the only way to release associated resources (it is called by ~GLContextManager).
          */
        void clear();

        inline void makeCurrent() const
        {
            if (pContext->isValid())
                pContext->makeCurrent();
        }

        inline const QGLContext *getContext() const  {   return pContext;    }

        /** \brief Set wether or not the GLContextManager owns the QGLContext
          *
          * If the GLContextManager owns the QGLContext, it will be destroyed by ~GLContextManager.
          */
        void setOwnContext(bool bOwn)   {   this->bOwn = bOwn;  }

    private:
        QGLContext *pContext;
        globject_table_t hTextures;
        globject_table_t hPrograms;
        globject_table_t hShaders;
        globject_table_t hVBO;
        bool bOwn;
        bool bHasVBOSupport;
    };
}

#endif // GLCONTEXTMANAGER_H
