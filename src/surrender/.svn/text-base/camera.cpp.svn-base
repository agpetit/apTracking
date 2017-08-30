#include <GL/glew.h>
#include "camera.h"
#include <osg/Group>
#include <osg/Switch>
#include <osg/Transform>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Array>
#include <osg/Texture2D>
#include <osg/Program>
#include "logs.h"
#include <GL/glu.h>
#include <algorithm>
#include "utils.h"

using namespace osg;

namespace luxifer
{

    Camera::Camera(osg::ref_ptr<GLContextManager> pContext, size_t w, size_t h, GLuint format)
        : pContext(pContext)
    {
        minimalz = 1.0;
        pContext->makeCurrent();
        CHECK_GL();
        fbo = new FBO(w, h, format);
        fov = 30.0;
        aspectRatio = 4.0 / 3.0;
        bDoublePrecisionMode = false;
    }

    Camera::Camera(osg::ref_ptr<GLContextManager> pContext, GLuint target, GLuint color_buffer, GLint nw, GLint nh)
        : pContext(pContext)
    {
        minimalz = 1.0;
        pContext->makeCurrent();
        CHECK_GL();
        fbo = new FBO(target, color_buffer, nw, nh);
        fov = 30.0;
        aspectRatio = 4.0 / 3.0;
        bDoublePrecisionMode = false;
    }

    Camera::Camera(osg::ref_ptr<GLContextManager> pContext, GLuint depth_buffer, GLint nw, GLint nh, void *)
        : pContext(pContext)
    {
        minimalz = 1.0;
        pContext->makeCurrent();
        CHECK_GL();
        fbo = new FBO(nw, nh, GL_LUMINANCE);
        fbo->setDepthTarget(depth_buffer, nw, nh);
        fov = 30.0;
        aspectRatio = 4.0 / 3.0;
        bDoublePrecisionMode = false;
    }

    Camera::Camera(osg::ref_ptr<GLContextManager> pContext, osg::ref_ptr<osg::Texture2D> ptex)
        : pContext(pContext),
        ptex(ptex)
    {
        minimalz = 1.0;
        pContext->makeCurrent();
        CHECK_GL();
        fbo = new FBO(GL_TEXTURE_2D, pContext->getTextureID(ptex.get()), ptex->getTextureWidth(), ptex->getTextureHeight());
        fov = 30.0;
        aspectRatio = 4.0 / 3.0;
        bDoublePrecisionMode = false;
    }

    Camera::~Camera()
    {
        pContext->makeCurrent();
        delete fbo;
    }

    void Camera::setColorTarget(GLuint target, GLuint color_buffer, GLint nw, GLint nh)
    {
        fbo->setColorTarget(target, color_buffer, nw, nh);
    }

    GLuint Camera::getColorTexture() const
    {
        return fbo->getColorTexture();
    }

    GLuint Camera::getDepthTexture() const
    {
        return fbo->getDepthTexture();
    }

    void Camera::processStateSet(const osg::StateSet *stateset, StateSet &s)
    {
        if (!stateset)
            return;
        for(int i = 15 ; i >= 0 ; --i)
        {
            const osg::StateSet::RefAttributePair *attr = stateset->getTextureAttributePair(i, StateAttribute::TEXTURE);
            if (!attr)
                continue;
            const Texture2D *ptex2D = dynamic_cast<const Texture2D*>(attr->first.get());
            const unsigned int mode = attr->second;
            if (ptex2D)
                s.setTexture(i, GL_TEXTURE_2D, pContext->getTextureID(ptex2D), mode);
            else
            {
                const TextureCubeMap *ptexCM = dynamic_cast<const TextureCubeMap*>(attr->first.get());
                if (ptexCM)
                    s.setTexture(i, GL_TEXTURE_CUBE_MAP, pContext->getTextureID(ptexCM), mode);
            }
        }
        s.setUniforms(stateset->getUniformList());
        const osg::StateSet::RefAttributePair *attr = stateset->getAttributePair(StateAttribute::PROGRAM, 0);
        if (attr && attr->first)
        {
            s.setProgram(pContext->getProgramID(dynamic_cast<const osg::Program*>(attr->first.get())), attr->second);
        }

        attr = stateset->getAttributePair(StateAttribute::MATERIAL);
        if (attr && attr->first)
        {
            s.setMaterial(dynamic_cast<const osg::Material*>(attr->first.get()), attr->second);
        }
    }

    void Camera::computeDepthRanges(const osg::Node *graph, std::vector<std::pair<double, double> > &ranges)
    {
        if (!graph || dynamic_cast<const osg::Camera*>(graph))
            return;

        bool bPopMatrix = false;

        const PositionAttitudeTransform *ptrans = dynamic_cast<const PositionAttitudeTransform*>(graph);
        if (ptrans)
        {
            const osg::Matrixd mat = Matrixd::translate(-ptrans->getPivotPoint())
                                   * Matrixd::rotate(ptrans->getAttitude())
                                   * Matrixd::scale(ptrans->getScale())
                                   * Matrixd::translate(ptrans->getPosition());
            pushMatrix(mat);
            bPopMatrix = true;
        }
        const MatrixTransform *mtrans = dynamic_cast<const MatrixTransform*>(graph);
        if (mtrans)
        {
            pushMatrix(mtrans->getMatrix());
            bPopMatrix = true;
        }

        const Switch* sgroup = dynamic_cast<const Switch*>(graph);
        if (sgroup)
        {
            for(unsigned int i = 0, nb = sgroup->getNumChildren() ; i < nb ; ++i)
                if (sgroup->getValue(i))
                    computeDepthRanges(sgroup->getChild(i), ranges);
        }
        else
        {
            const Group* group = dynamic_cast<const Group*>(graph);
            if (group)
            {
                for(unsigned int i = 0, nb = group->getNumChildren() ; i < nb ; ++i)
                    computeDepthRanges(group->getChild(i), ranges);
            }
        }

        const Geode *geode = dynamic_cast<const Geode*>(graph);
        if (geode)
        {
            const osg::BoundingSphered &bound = computeBoundsOf(graph);
            const double r = bound.radius();
            const Vec3d view_center = bound.center() * qMatrix.back();
            const double zmin = -view_center.z() - r;
            const double zmax = -view_center.z() + r;
            ranges.push_back(std::make_pair(zmin, zmax));
        }

        if (bPopMatrix)
            popMatrix();
    }

    void Camera::renderGraph(const Node *graph)
    {
        if (!graph || dynamic_cast<const osg::Camera*>(graph))
            return;

        const osg::BoundingSphered &bound = computeBoundsOf(graph);
        const double r = bound.radius();
        const Vec3d view_center = bound.center() * qMatrix.back();
        if (-view_center.z() + r < znear || -view_center.z() - r > zfar)
            return;

        bool bPopMatrix = false;

        const osg::StateSet *stateset = graph->getStateSet();
        if (stateset)
        {
            qStateSet.push_back(qStateSet.back());
            processStateSet(stateset, qStateSet.back());
        }

        const PositionAttitudeTransform *ptrans = dynamic_cast<const PositionAttitudeTransform*>(graph);
        if (ptrans)
        {
            const Matrixd mat = Matrixd::translate(-ptrans->getPivotPoint())
                               * Matrixd::rotate(ptrans->getAttitude())
                               * Matrixd::scale(ptrans->getScale())
                               * Matrixd::translate(ptrans->getPosition());
            pushMatrix(mat);
            bPopMatrix = true;
        }
        const MatrixTransform *mtrans = dynamic_cast<const MatrixTransform*>(graph);
        if (mtrans)
        {
            pushMatrix(mtrans->getMatrix());
            bPopMatrix = true;
        }

        const Switch* sgroup = dynamic_cast<const Switch*>(graph);
        if (sgroup)
        {
            for(unsigned int i = 0, nb = sgroup->getNumChildren() ; i < nb ; ++i)
                if (sgroup->getValue(i))
                    renderGraph(sgroup->getChild(i));
        }
        else
        {
            const Group* group = dynamic_cast<const Group*>(graph);
            if (group)
            {
                for(unsigned int i = 0, nb = group->getNumChildren() ; i < nb ; ++i)
                    renderGraph(group->getChild(i));
            }
        }

        const Geode *geode = dynamic_cast<const Geode*>(graph);
        if (geode)
        {
            glEnableClientState(GL_VERTEX_ARRAY);
            CHECK_GL();
            bool bParentStateApplied = false;
            for(unsigned int i = 0, nb = geode->getNumDrawables() ; i < nb ; ++i)
            {
                const Geometry * const geom = geode->getDrawable(i)->asGeometry();
                if (!geom)
                {
                    LOG_ERROR() << "[Camera] unsupported drawable : " << geode->getDrawable(i)->className() << std::endl;
                    continue;
                }

                const Array * const vtx = geom->getVertexArray();
                const Array * const normals = geom->getNormalArray();
                const Array * const tcoord = geom->getTexCoordArray(0);

                if (!vtx)
                {
                    LOG_ERROR() << "[Camera] no vertex array in Geometry object" << std::endl;
                    continue;
                }

                StateSet *local_state = NULL;
                char tmp[sizeof(StateSet)];     //! Avoid dynamic allocation
                const osg::StateSet *geom_state = geom->getStateSet();

                if (geom_state)
                {
                    local_state = new(tmp) StateSet(qStateSet.back());
                    processStateSet(geom_state, *local_state);
                }

                if (local_state)
                {
                    local_state->apply();
                    bParentStateApplied = false;
                }
                else if (!bParentStateApplied)
                {
                    bParentStateApplied = true;
                    qStateSet.back().apply();
                }

                if (bDoublePrecisionMode)
                {
                    if (processed_vertex_array.size() < vtx->getNumElements())
                        processed_vertex_array.resize(vtx->getNumElements());
                    const void * const ptr = vtx->getDataPointer();
                    const osg::Matrixd &mat = qMatrix.back() * projectionMatrix;
                    switch(vtx->getDataType())
                    {
                    case GL_FLOAT:
                        if (vtx->getDataSize() == 4)
                        {
                            const size_t nb = vtx->getNumElements();
                            for(size_t i = 0 ; i < nb ; ++i)
                            {
                                const Vec4d &p = Vec4d(((const Vec4f*)ptr)[i]) * mat;
                                processed_vertex_array[i] = p / p.w();
                            }
                        }
                        else if (vtx->getDataSize() == 3)
                        {
                            const size_t nb = vtx->getNumElements();
                            for(size_t i = 0 ; i < nb ; ++i)
                            {
                                const Vec4d &p = Vec4d(((const Vec3f*)ptr)[i], 1.0) * mat;
                                processed_vertex_array[i] = p / p.w();
                            }
                        }
                        break;
                    }

                    glEnableClientState(GL_COLOR_ARRAY);
                    CHECK_GL();
                    if (pContext->hasVBOSupport())
                    {
                        glBindBuffer(GL_ARRAY_BUFFER, 0);
                        CHECK_GL();
                    }
                    glColorPointer(4, GL_FLOAT, 0, &(processed_vertex_array.front()));
                    CHECK_GL();
                }

                if (pContext->hasVBOSupport())
                {
                    glBindBuffer(GL_ARRAY_BUFFER, pContext->getDataVBOID(vtx->getDataPointer(), vtx->getTotalDataSize()));
                    CHECK_GL();
                    glVertexPointer(vtx->getDataSize(), vtx->getDataType(), 0, 0);
                    CHECK_GL();
                    if (normals)
                    {
                        glEnableClientState(GL_NORMAL_ARRAY);
                        CHECK_GL();
                        glBindBuffer(GL_ARRAY_BUFFER, pContext->getDataVBOID(normals->getDataPointer(), normals->getTotalDataSize()));
                        CHECK_GL();
                        glNormalPointer(normals->getDataType(), 0, 0);
                        CHECK_GL();
                    }
                    else
                    {
                        glDisableClientState(GL_NORMAL_ARRAY);
                        CHECK_GL();
                    }
                    if (tcoord)
                    {
                        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
                        CHECK_GL();
                        glBindBuffer(GL_ARRAY_BUFFER, pContext->getDataVBOID(tcoord->getDataPointer(), tcoord->getTotalDataSize()));
                        CHECK_GL();
                        glTexCoordPointer(tcoord->getDataSize(), tcoord->getDataType(), 0, 0);
                        CHECK_GL();
                    }
                    else
                    {
                        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
                        CHECK_GL();
                    }

                    const size_t nb_prim_set = geom->getNumPrimitiveSets();
                    for(size_t j = 0 ; j < nb_prim_set ; ++j)
                    {
                        const PrimitiveSet * const pset = geom->getPrimitiveSet(j);
                        const DrawElements * const elts = pset->getDrawElements();
                        switch(pset->getType())
                        {
                        case DrawElements::PrimitiveType:
                            break;
                        case DrawElements::DrawArraysPrimitiveType:
                            glDrawArrays(pset->getMode(), pset->index(0), pset->getNumIndices());
                            CHECK_GL();
                            break;
                        case DrawElements::DrawArrayLengthsPrimitiveType:   break;
                        case DrawElements::DrawElementsUBytePrimitiveType:
                            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, pContext->getIndexVBOID(elts->getDataPointer(), elts->getTotalDataSize()));
                            CHECK_GL();
                            glDrawRangeElements(elts->getMode(), 0, vtx->getNumElements() - 1, elts->getNumIndices(), GL_UNSIGNED_BYTE, 0);
                            CHECK_GL();
                            break;
                        case DrawElements::DrawElementsUShortPrimitiveType:
                            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, pContext->getIndexVBOID(elts->getDataPointer(), elts->getTotalDataSize()));
                            CHECK_GL();
                            glDrawRangeElements(elts->getMode(), 0, vtx->getNumElements() - 1, elts->getNumIndices(), GL_UNSIGNED_SHORT, 0);
                            CHECK_GL();
                            break;
                        case DrawElements::DrawElementsUIntPrimitiveType:
                            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, pContext->getIndexVBOID(elts->getDataPointer(), elts->getTotalDataSize()));
                            CHECK_GL();
                            glDrawRangeElements(elts->getMode(), 0, vtx->getNumElements() - 1, elts->getNumIndices(), GL_UNSIGNED_INT, 0);
                            CHECK_GL();
                            break;
                        }
                    }
                }
                else
                {
                    glVertexPointer(vtx->getDataSize(), vtx->getDataType(), 0, vtx->getDataPointer());
                    CHECK_GL();
                    if (normals)
                    {
                        glEnableClientState(GL_NORMAL_ARRAY);
                        CHECK_GL();
                        glNormalPointer(normals->getDataType(), 0, normals->getDataPointer());
                        CHECK_GL();
                    }
                    else
                    {
                        glDisableClientState(GL_NORMAL_ARRAY);
                        CHECK_GL();
                    }
                    if (tcoord)
                    {
                        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
                        CHECK_GL();
                        glTexCoordPointer(tcoord->getDataSize(), tcoord->getDataType(), 0, tcoord->getDataPointer());
                        CHECK_GL();
                    }
                    else
                    {
                        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
                        CHECK_GL();
                    }

                    const size_t nb_prim_set = geom->getNumPrimitiveSets();
                    for(size_t j = 0 ; j < nb_prim_set ; ++j)
                    {
                        const PrimitiveSet * const pset = geom->getPrimitiveSet(j);
                        const DrawElements * const elts = pset->getDrawElements();
                        switch(pset->getType())
                        {
                        case DrawElements::PrimitiveType:
                            break;
                        case DrawElements::DrawArraysPrimitiveType:
                            glDrawArrays(pset->getMode(), pset->index(0), pset->getNumIndices());
                            CHECK_GL();
                            break;
                        case DrawElements::DrawArrayLengthsPrimitiveType:   break;
                        case DrawElements::DrawElementsUBytePrimitiveType:
                            glDrawRangeElements(elts->getMode(), 0, vtx->getNumElements() - 1, elts->getNumIndices(), GL_UNSIGNED_BYTE, elts->getDataPointer());
                            CHECK_GL();
                            break;
                        case DrawElements::DrawElementsUShortPrimitiveType:
                            glDrawRangeElements(elts->getMode(), 0, vtx->getNumElements() - 1, elts->getNumIndices(), GL_UNSIGNED_SHORT, elts->getDataPointer());
                            CHECK_GL();
                            break;
                        case DrawElements::DrawElementsUIntPrimitiveType:
                            glDrawRangeElements(elts->getMode(), 0, vtx->getNumElements() - 1, elts->getNumIndices(), GL_UNSIGNED_INT, elts->getDataPointer());
                            CHECK_GL();
                            break;
                        }
                    }
                }
                if (local_state)
                    local_state->~StateSet();
            }
        }

        if (bPopMatrix)
            popMatrix();
        if (stateset)
            qStateSet.pop_back();
    }

    void Camera::operator ()()
    {
        if (!pContext->getContext()->isValid())
            return;
        pContext->makeCurrent();

        CHECK_GL();

        if (ptex)
        {
            if (pContext->getTextureID(ptex) != fbo->getColorTexture())
                fbo->setColorTarget(GL_TEXTURE_2D, pContext->getTextureID(ptex), ptex->getTextureWidth(), ptex->getTextureHeight());
        }

        fbo->bind();
        CHECK_GL();
        glClearColor(0,0,0,0);
        CHECK_GL();
        glClear(GL_COLOR_BUFFER_BIT);
        CHECK_GL();

        if (background)     // background image
        {
            glClear(GL_DEPTH_BUFFER_BIT);
            CHECK_GL();
            glMatrixMode(GL_PROJECTION);
            CHECK_GL();
            glLoadIdentity();
            CHECK_GL();
            gluOrtho2D(0.0, 1.0, 1.0, 0.0);
            CHECK_GL();
            glMatrixMode(GL_MODELVIEW);
            CHECK_GL();
            glLoadIdentity();
            CHECK_GL();

            const GLuint pid = pContext->getProgramID(background_program);
            glUseProgram(pid);
            glUniform1i(glGetUniformLocation(pid, "tex"), 0);
            const osg::Matrixf mat(viewMatrix);
            glUniformMatrix4fv(glGetUniformLocation(pid, "viewMatrix"), 1, GL_FALSE, mat.ptr());
            glUniform1f(glGetUniformLocation(pid, "fov"), fov * M_PI / 180.0);
            glUniform1f(glGetUniformLocation(pid, "aspectRatio"), aspectRatio);

            glActiveTexture(GL_TEXTURE0);
            CHECK_GL();
            glDisable(GL_TEXTURE_CUBE_MAP);
            CHECK_GL();
            glEnable(GL_TEXTURE_2D);
            CHECK_GL();
            glBindTexture(GL_TEXTURE_2D, pContext->getTextureID(background));
            CHECK_GL();
            glDisable(GL_CULL_FACE);
            CHECK_GL();

            glBegin(GL_QUADS);
            glVertex2f(0.f,0.f);
            glVertex2f(1.f,0.f);
            glVertex2f(1.f,1.f);
            glVertex2f(0.f,1.f);
            glEnd();

            glUseProgram(0);
        }
        else if (starmap)   // background star map
        {
            glClear(GL_DEPTH_BUFFER_BIT);
            CHECK_GL();
            glMatrixMode(GL_PROJECTION);
            CHECK_GL();
            projectionMatrix = Matrixd::perspective(fov, aspectRatio, 0.5f, 2.f);
            glLoadMatrixd(projectionMatrix.ptr());
            CHECK_GL();
            glMatrixMode(GL_MODELVIEW);
            CHECK_GL();
            glLoadMatrixd(viewMatrix.ptr());
            CHECK_GL();

            const GLuint pid = pContext->getProgramID(background_program);
            glUseProgram(pid);
            CHECK_GL();

            glActiveTexture(GL_TEXTURE0);
            CHECK_GL();
            glDisable(GL_TEXTURE_CUBE_MAP);
            CHECK_GL();
            glDisable(GL_TEXTURE_2D);
            CHECK_GL();
            glDisable(GL_CULL_FACE);
            CHECK_GL();

            glEnableClientState(GL_VERTEX_ARRAY);
            CHECK_GL();
            glEnableClientState(GL_COLOR_ARRAY);
            CHECK_GL();
            glDisableClientState(GL_NORMAL_ARRAY);
            CHECK_GL();
            glDisableClientState(GL_TEXTURE_COORD_ARRAY);
            CHECK_GL();
            if (pContext->hasVBOSupport())
            {
                glBindBuffer(GL_ARRAY_BUFFER, pContext->getDataVBOID(starmap->getDataPointer(), starmap->getTotalDataSize()));
                CHECK_GL();
                glVertexPointer(3,GL_FLOAT,12,0);
                CHECK_GL();
                glColorPointer(3,GL_FLOAT,12,(const GLvoid*)12);
                CHECK_GL();
            }
            else
            {
                glVertexPointer(3,GL_FLOAT,12,starmap->getDataPointer());
                CHECK_GL();
                glColorPointer(3,GL_FLOAT,12,(const char*)(starmap->getDataPointer()) + 12);
                CHECK_GL();
            }

            glEnable(GL_COLOR_MATERIAL);
            CHECK_GL();
            glEnable(GL_POINT_SPRITE);
            CHECK_GL();
            glPointSize(4.0f);
            CHECK_GL();

            glDrawArrays(GL_POINTS, 0, starmap->size() / 2);
            CHECK_GL();

            glDisableClientState(GL_COLOR_ARRAY);
            CHECK_GL();
            glPointSize(1.0f);
            CHECK_GL();
            glDisable(GL_POINT_SPRITE);
            CHECK_GL();
            glDisable(GL_COLOR_MATERIAL);
            CHECK_GL();

            if (pContext->hasVBOSupport())
            {
                glBindBuffer(GL_ARRAY_BUFFER, 0);
                CHECK_GL();
            }

            glUseProgram(0);
        }

        std::vector<std::pair<double, double> > depthRange;
        qMatrix.clear();
        qMatrix.push_back(viewMatrix);

        computeDepthRanges(scene, depthRange);
        computeMinimumRanges(depthRange);

        for(std::vector<std::pair<double, double> >::const_iterator range = depthRange.begin()
            ; range != depthRange.end()
            ; ++range)
        {
            glClear(GL_DEPTH_BUFFER_BIT);
            CHECK_GL();
            glMatrixMode(GL_PROJECTION);
            CHECK_GL();
            projectionMatrix = Matrixd::perspective(fov, aspectRatio, range->first, range->second);
            glLoadMatrixd(projectionMatrix.ptr());
            CHECK_GL();

            znear = range->first;
            zfar = range->second;

            glMatrixMode(GL_MODELVIEW);
            CHECK_GL();
            clearMatrixStack();
            CHECK_GL();

            glDisable(GL_TEXTURE_2D);
            CHECK_GL();
            glDisable(GL_TEXTURE_CUBE_MAP);
            CHECK_GL();
            glDisable(GL_COLOR_MATERIAL);
            CHECK_GL();
            glColor4ub(0xFF, 0xFF, 0xFF, 0xFF);
            CHECK_GL();
            glDisable(GL_BLEND);
            CHECK_GL();
            glDisable(GL_LIGHTING);
            CHECK_GL();
            glDisable(GL_CULL_FACE);
            CHECK_GL();
            glShadeModel(GL_SMOOTH);
            CHECK_GL();
            glEnable(GL_DEPTH_TEST);
            CHECK_GL();
            glDepthFunc(GL_LESS);
            CHECK_GL();

            qStateSet.clear();
            qStateSet.push_back(StateSet());
            /** Cubemaps cannot be used to override the background texture
              * which is a GL_TEXTURE_2D so we just render to a GL_TEXTURE_2D,
              * use it as background texture and copy it to the cubemap
              */
            if (ptexcube)
                qStateSet.back().setTexture(7, GL_TEXTURE_2D, fbo->getColorTexture(), osg::StateAttribute::OVERRIDE);

            renderGraph(scene);
        }
        glDisableClientState(GL_COLOR_ARRAY);
        CHECK_GL();

        if (ptexcube)
        {
            glBindTexture(GL_TEXTURE_CUBE_MAP, pContext->getTextureID(ptexcube));
            CHECK_GL();
            glCopyTexSubImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + cube_face_id, 0, 0, 0, 0, 0, fbo->getWidth(), fbo->getHeight());
            CHECK_GL();
        }

        fbo->release();

        glUseProgram(0);
        CHECK_GL();
        for(int i = 15 ; i >= 0 ; --i)
        {
            glActiveTexture(GL_TEXTURE0 + i);
            CHECK_GL();
            glDisable(GL_TEXTURE_2D);
            CHECK_GL();
            glDisable(GL_TEXTURE_CUBE_MAP);
            CHECK_GL();
        }
    }

    void Camera::setScene(osg::ref_ptr<osg::Node> scene)
    {
        this->scene = scene;
    }

    void Camera::setViewMatrix(const osg::Matrixd &viewMatrix)
    {
        this->viewMatrix = viewMatrix;
    }

    const osg::Matrixd &Camera::getViewMatrix() const
    {
        return viewMatrix;
    }

    osg::Matrixd Camera::getProjectionMatrix() const
    {
        return Matrixd::perspective(fov, aspectRatio, 1.0, 1e4);
    }

    void Camera::pushMatrix(const osg::Matrixd &mat)
    {
        const Matrixd m = mat * qMatrix.back();
        glLoadMatrixd(m.ptr());
        CHECK_GL();
        qMatrix.push_back(m);
    }

    void Camera::popMatrix()
    {
        qMatrix.pop_back();
        glLoadMatrixd(qMatrix.back().ptr());
        CHECK_GL();
    }

    void Camera::clearMatrixStack()
    {
        qMatrix.clear();
        qMatrix.push_back(viewMatrix);
        glLoadMatrixd(viewMatrix.ptr());
        CHECK_GL();
    }

    void Camera::setProjectionMatrix(const osg::Matrixd &/*projectionMatrix*/)
    {

    }

    void Camera::computeMinimumRanges(std::vector<std::pair<double, double> > &ranges)
    {
        const int START = 0, END = 1;
        std::vector<std::pair<double, int> > ends;
        for(size_t i = 0 ; i < ranges.size() ; ++i)
        {
            ends.push_back(std::make_pair(ranges[i].first, START));
            ends.push_back(std::make_pair(ranges[i].second, END));
        }

        std::sort(ends.begin(), ends.end());
        int n = 0;
        std::pair<double, double> cur;
        ranges.clear();
        const double max_ratio = 1e4;
        for(std::vector<std::pair<double, int> >::const_iterator it = ends.begin() ; it != ends.end() ; ++it)
        {
            if (it->second == START)
            {
                ++n;
                if (n == 1)
                    cur.first = std::max(minimalz, it->first);
            }
            else
            {
                --n;
                if (n == 0)
                {
                    cur.second = it->first;
                    if (cur.second > minimalz)
                    {
                        while(cur.second / cur.first > max_ratio)
                        {
                            ranges.push_back(std::make_pair(cur.first, cur.first * max_ratio));
                            cur.first *= max_ratio;
                        }
                        if (ranges.size() > 0 && cur.second / ranges.back().first <= max_ratio)
                            ranges.back().second = cur.second;
                        else
                            ranges.push_back(cur);
                    }
                }
            }
        }

        std::reverse(ranges.begin(), ranges.end());
    }

    void Camera::setBackground(osg::ref_ptr<osg::Texture2D> background)
    {
        this->background = background;
        this->starmap = NULL;

        // Background shader
        background_program = new Program;
        osg::ref_ptr<osg::Shader> background_vshader = new Shader(Shader::VERTEX);
        osg::ref_ptr<osg::Shader> background_fshader = new Shader(Shader::FRAGMENT);
        background_program->addShader(background_vshader);
        background_program->addShader(background_fshader);

        background_vshader->loadShaderSourceFromFile("shaders/background.vert");
        background_fshader->loadShaderSourceFromFile("shaders/background.frag");
    }

    void Camera::setBackground(osg::ref_ptr<osg::Vec3Array> starmap)
    {
        this->background = NULL;
        this->starmap = starmap;

        // Starmap shader
        background_program = new Program;
        osg::ref_ptr<osg::Shader> background_vshader = new Shader(Shader::VERTEX);
        osg::ref_ptr<osg::Shader> background_fshader = new Shader(Shader::FRAGMENT);
        background_program->addShader(background_vshader);
        background_program->addShader(background_fshader);

        background_vshader->loadShaderSourceFromFile("shaders/starmap.vert");
        background_fshader->loadShaderSourceFromFile("shaders/starmap.frag");
    }

    void Camera::setCubeMapFaceTarget(osg::ref_ptr<osg::TextureCubeMap> ptexcube, unsigned int cube_face_id)
    {
        this->ptexcube = ptexcube;
        this->cube_face_id = cube_face_id;
    }

    void Camera::setDoublePrecisionMode(bool bDoublePrecisionMode)
    {
        this->bDoublePrecisionMode = bDoublePrecisionMode;
    }
}
