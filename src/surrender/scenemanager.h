#ifndef SCENEMANAGER_H
#define SCENEMANAGER_H

#include <map>
#include <string>
#include <osg/Node>
#include <osg/Geode>
#include <osg/Group>
#include <osg/Vec3d>
#include <osg/Uniform>
#include <osg/TextureCubeMap>
#include <osg/Texture2D>
#include <osg/PositionAttitudeTransform>
#include <osg/Switch>
#include "camera.h"
#include "glcontextmanager.h"
#include <QObject>
#include <visp/vpImage.h>
#include <osg/LightSource>
#include "modelloader.h"

#ifdef ENABLE_LIBRARY_COMPILE
#include <config.h>
#endif // DENABLE_LIBRARY_COMPILE

class vpHomogeneousMatrix;
class apRend;


namespace luxifer
{

    /** \brief Stores scene description and handles camera moves
      *
      * Before rendering each frame the whole scene is translated to have the camera
      * at the origin (0,0,0) in order to maximize precision around it (and avoid
      * 3D models jittering).
      */
    class SceneManager : public QObject
    {
        Q_OBJECT
    public:
        SceneManager(QGLContext *pQContext);
        ~SceneManager();

        const QGLContext *getQGLContext() const    {   return pContext->getContext();  }
        const GLContextManager *getContext() const    {   return pContext;  }
        QGLContext *getQGLContext() {   return const_cast<QGLContext*>(pContext->getContext());  }
        GLContextManager *getContext()  {   return pContext;  }

        void clear();

        /** \brief load a 3D model
          */
#ifdef ENABLE_LIBRARY_COMPILE
        void load(const std::string &filename, const std::string &shaderLocationPath = aptracking::config::ShaderLocation);
#else
        void load(const std::string &filename, const std::string &shaderLocationPath = std::string());
#endif // ENABLE_LIBRARY_COMPILE

        void load(std::vector<point3d> &vertices, std::vector<point3d> &normals, std::vector<triangle> &triangles);

        void update(std::vector<point3d> &vertices, std::vector<point3d> &normals, std::vector<triangle> &triangles);

        //! \brief Returns a node which describes the scene
        osg::ref_ptr<osg::Node> getSceneNode();

        //! \brief Set camera matric
        void setCamera(const osg::Matrixd &mat);
        //! \brief Set camera position
        void setCameraPosition(const osg::Vec3d &pos);
        //! \brief Set camera attitude
        void setCameraAttitude(const osg::Quat &attitude);

        osg::Vec3d getCameraPosition();
        osg::Quat getCameraAttitude();
        osg::Matrixd getCameraMatrix();

        GLuint getColorBuffer();
        GLuint getNormalBuffer();

        void updateCameraParameters (const vpHomogeneousMatrix &cMo);
        void updateCameraParameters (const vpHomogeneousMatrix &cMo, const double Znear, const double Zfar);
        void updateRTT(vpImage<vpRGBa> &I1, vpImage<unsigned char> &I0, vpHomogeneousMatrix *CMo);
        void updateRTT(vpImage<vpRGBa> &I1, vpImage<unsigned char> &I0, vpHomogeneousMatrix *CMo, const double znear, const double zfar);
        void updateRTTCol(vpImage<vpRGBa> &I2, vpImage<vpRGBa> &I1, vpImage<unsigned char> &I0, vpHomogeneousMatrix *CMo);
        void updateRTTCol(vpImage<vpRGBa> &I2, vpImage<vpRGBa> &I1, vpImage<unsigned char> &I0, vpHomogeneousMatrix *CMo, const double znear, const double zfar);

        void updateRTTColAR(vpImage<vpRGBa> &I2,vpHomogeneousMatrix *CMo);


        void setApRend(const apRend *mrend);
    signals:
        void updated();

    public slots:
        void render();
        void renderAR();
        double getFOV() { return fov; }
        void setFOV(const double fov);
        void setAspectRatio(const double aspect_ratio);
        void setImageSize(const size_t image_w, const size_t image_h);
        void postprocess(osg::ref_ptr<osg::Texture2D> in, osg::ref_ptr<osg::Texture2D> out, osg::ref_ptr<osg::Program> program);

    private:
        //! \brief Copy osg::Texture2D data into a system memory buffer in the given format (you must allocate enough memory for image data)
        void copyTextureToMemory(osg::ref_ptr<osg::Texture2D> ptex, void *data, GLuint format, GLuint type);
        //! \brief Copy osg::Texture2D data into another osg::Texture2D object
        void copyTextureToTexture(osg::ref_ptr<osg::Texture2D> dst, osg::ref_ptr<osg::Texture2D> src);

        void updateClipDistances(const double Zd);
        void updateClipDistances(const double Znear, const double Zfar);
    private:
        osg::ref_ptr<osg::Node> scene_node;

        osg::ref_ptr<osg::Texture2D> color_buffer;
        osg::ref_ptr<osg::Texture2D> edge_buffer;
        osg::ref_ptr<osg::Texture2D> normal_buffer;
        osg::ref_ptr<osg::Texture2D> normal_buffer_big;

        osg::ref_ptr<osg::Program> fsaa_program;
        osg::ref_ptr<osg::Program> compositor_program;
        osg::ref_ptr<osg::Uniform> u_fNear, u_fFar;

        osg::ref_ptr<Camera> color_cam;
        osg::ref_ptr<Camera> normal_cam;
        osg::Matrixd camera_matrix;
        osg::LightSource *lightsource;

        osg::ref_ptr<GLContextManager> pContext;
        FBO *pFBO_postproc;
        FBO *pFBO_tmp;

        //! \brief Resolution of image
        size_t image_w, image_h;
        //! \brief Camera parameters (can be changed dynamically)
        double fov, aspect_ratio;
        float fNear,fFar;

        const apRend *mrend;                                  /** Rendering Parameters  */
    };
}

#endif // SCENEMANAGER_H
