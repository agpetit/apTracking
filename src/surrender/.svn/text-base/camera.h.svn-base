#ifndef CAMERA_H
#define CAMERA_H

#include <osg/Node>
#include <osg/Matrixd>
#include "fbo.h"
#include <deque>
#include "glcontextmanager.h"
#include "stateset.h"

namespace luxifer
{

    /** \brief Abstract camera renderer (renders image from a virtual camera)
      *
      * A Camera object is an OpenGL renderer which renders a scene as seen by a pinhole camera whose position,
      * attitude, field of view and aspect ratio is set through \b setViewMatrix, \b setFOV and \b setAspectRatio.
      *
      * This is an offscreen renderer, which means it doesn't render directly on screen but to a texture. An OpenGL camera
      * usually has a near plane and a far plane, since it would be too restrictive here a Camera only has a near plane (you
      * can set it with the \b setMinimalDepth method). In order to remove the far plane limitation rendering is done in multiple
      * passes and scene depth is split into a minimal set of sorted depth ranges in order to optimize depth buffer use and avoid
      * poor precision artifacts (such as Sun being rendered through Earth).
      */
    class Camera : public osg::Referenced
    {
    protected:
        Camera(osg::ref_ptr<GLContextManager> pContext, GLuint depth_buffer, GLint nw, GLint nh, void *);
    public:
        Camera(osg::ref_ptr<GLContextManager> pContext, size_t w, size_t h, GLuint format);
        Camera(osg::ref_ptr<GLContextManager> pContext, GLuint target, GLuint color_buffer, GLint nw, GLint nh);
        Camera(osg::ref_ptr<GLContextManager> pContext, osg::ref_ptr<osg::Texture2D> ptex);
        virtual ~Camera();

        void setColorTarget(GLuint target, GLuint color_buffer, GLint nw, GLint nh);
        GLuint getColorTexture() const;
        GLuint getDepthTexture() const;

        virtual void operator()();

        void setScene(osg::ref_ptr<osg::Node> scene);

        void setFOV(double fov) {   this->fov = fov;    }
        void setAspectRatio(double aspectRatio) {   this->aspectRatio = aspectRatio;    }
        void setViewMatrix(const osg::Matrixd &viewMatrix);
        const osg::Matrixd &getViewMatrix() const;
        virtual void setProjectionMatrix(const osg::Matrixd &projectionMatrix);
        virtual osg::Matrixd getProjectionMatrix() const;

        void setBackground(osg::ref_ptr<osg::Texture2D> background);
        void setBackground(osg::ref_ptr<osg::Vec3Array> starmap);
        void setMinimalDepth(const double minimalz) {   this->minimalz = minimalz;  }

        void setCubeMapFaceTarget(osg::ref_ptr<osg::TextureCubeMap> ptexcube, unsigned int cube_face_id);

        void setDoublePrecisionMode(bool bDoublePrecisionMode);

    protected:
        void renderGraph(const osg::Node *graph);
        void pushMatrix(const osg::Matrixd &mat);
        void popMatrix();
        void clearMatrixStack();
        void processStateSet(const osg::StateSet *stateset, StateSet &s);
        void computeDepthRanges(const osg::Node *graph, std::vector<std::pair<double, double> > &ranges);
        void computeMinimumRanges(std::vector<std::pair<double, double> > &ranges);

    protected:
        osg::ref_ptr<osg::Node> scene;
        FBO *fbo;
        osg::Matrixd viewMatrix;
        osg::Matrixd projectionMatrix;
        bool bDoublePrecisionMode;
        std::vector<osg::Vec4f> processed_vertex_array;
        double fov, aspectRatio;
        std::deque<osg::Matrixd> qMatrix;
        std::deque<StateSet> qStateSet;
        osg::ref_ptr<GLContextManager> pContext;
        osg::ref_ptr<osg::Texture2D> ptex;
        osg::ref_ptr<osg::TextureCubeMap> ptexcube;
        unsigned int cube_face_id;
        osg::ref_ptr<osg::Texture2D> background;
        osg::ref_ptr<osg::Vec3Array> starmap;
        osg::ref_ptr<osg::Program> background_program;
        double znear, zfar;
        double minimalz;
    };

}
#endif // CAMERA_H
